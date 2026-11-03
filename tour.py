"""tour.py -- the /robot command: visit the 8 corners of the workspace box, one after another, at the fastest
speed the limits allow, then (by default) return to the exact pose where the tour started.

No AI is involved. This module only BUILDS a plan (the same plan format the planner produces). The normal
resolve -> template code -> deterministic validator -> approval -> execute path does the rest, so the tour is held to
exactly the same safety rules as any other program.

Why an IK check: the workspace is a box in the base frame, but the arm cannot reach every corner of a box with a fixed
tool orientation (the Z_MAX of 1.09 m, for example, is above the UR5e reach). A move to an unreachable pose is rejected
by the controller in the middle of the run. So before proposing anything, every straight segment is sampled and solved
with a small damped-least-squares inverse kinematics seeded from the live joints (the controller also follows the
nearest solution). Corners that cannot be reached with the current tool orientation are SKIPPED and reported.
"""
import math

import limits as L
import robot_sender as R


class TourError(Exception):
    """The tour cannot be built (the message says why). Nothing was sent anywhere."""


class Cancelled(Exception):
    """The operator pressed stop while the tour was being checked."""


# --- inverse kinematics, tool model and the reach envelope live in robot_sender.py (ONE implementation, shared with the validator) ---
solve_ik = R.ik_solve
_ToolModel = R.ToolModel


def _assumed_flange_tcp(tcp_pose):
    """Dry-run without joint data only: pretend the TCP IS the flange (optimistic: a real tool is longer)."""
    t = R.ToolModel.__new__(R.ToolModel)
    t.offset = [0.0, 0.0, 0.0]
    t.length = 0.0
    t.r_tcp = R.rotvec_to_matrix(tcp_pose[3:])
    t.r_tool = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    return t


def _guess_q(tool, tcp_pose):
    """A plausible 'normal' arm configuration (shoulder up, elbow bent) for a pose. Dry-run only."""
    import random
    rng = random.Random(7)
    pf, rf = tool.flange_target(tcp_pose[:3])
    for _ in range(300):
        seed = [rng.uniform(-0.6, 0.6) + (math.pi if rng.random() < 0.5 else 0.0), rng.uniform(-2.0, -0.7),
                rng.uniform(0.8, 2.4), rng.uniform(-3.0, -1.0), rng.uniform(-2.0, -1.2), rng.uniform(-3.2, 3.2)]
        q = solve_ik(seed, pf, rf)
        if q and -2.2 < q[1] < -0.5 and 0.4 < q[2] < 2.6 and -3.3 < q[3] < -0.5:
            return q
    return None


def _segment_ok(tool, q, p_from, p_to, should_stop, r_from=None, r_to=None):
    """Walk the straight segment p_from -> p_to through the reach envelope; the orientation turns smoothly from r_from to
    r_to (both default to the live tool orientation = kept). Returns (ok, q_end, reason)."""
    r_from = tool.r_tcp if r_from is None else r_from
    r_to = r_from if r_to is None else r_to
    pts = R.straight_points(p_from, p_to, L.TOUR_SAMPLE_STEP_M, r_from, r_to)
    ok, q_end, why, t = R.reach_walk(tool, q, pts, r_from, r_to, should_stop)
    if why == "stopped":
        raise Cancelled()
    return ok, q_end, ("" if ok else f"{why} ({t * 100:.0f}% along the way)")


# --- the tour -------------------------------------------------------------------------------------
_CYCLE = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 0, 1), (0, 0, 1)]   # each step = one edge


def _corner_name(c):
    return ", ".join(f"{ax} {'max' if b else 'min'}" for ax, b in zip("XYZ", c))


def build_tour_plan(state, speed=None, stay=False, should_stop=lambda: False, log=print, follow_yaw=True):
    """Returns (plan, notes). Raises TourError / Cancelled. `speed` (m/s) defaults to the active maximum.
    follow_yaw=True: the tool turns about the vertical with the arm's azimuth around the base axis (the wrist never winds
    up, corners behind the base stay reachable). follow_yaw=False: the live tool orientation is kept at every corner."""
    if state is None or state.pose is None:
        raise TourError("No fresh live robot state.")
    lim = L.get_limits()
    bad = L.check_consistency(lim)
    if bad:
        raise TourError("Configuration problem: " + " | ".join(bad))
    v = lim["max_speed"] if speed is None else float(speed)
    if not 0 < v <= lim["max_speed"] + 1e-12:
        raise TourError(f"Speed {v} m/s is outside (0, {lim['max_speed']}]. Raise max_speed first, e.g. "
                        f"/limit max_speed {v} (session) or MAX_TCP_SPEED in limits.py (permanent).")
    a = lim["max_accel"]
    m = L.LIMIT_MARGIN_M
    pose = list(state.pose)
    if not R._in_box(pose, lim):
        raise TourError("The TCP is outside the workspace box right now; every motion is refused until it is inside.")

    def corner_xyz(c):
        return [lim["x_min"] + m if c[0] == 0 else lim["x_max"] - m,
                lim["y_min"] + m if c[1] == 0 else lim["y_max"] - m,
                lim["z_min"] + m if c[2] == 0 else lim["z_max"] - m]

    # start the cycle at the corner nearest to the robot
    first = min(range(8), key=lambda i: R._dist(pose, corner_xyz(_CYCLE[i])))
    order = [_CYCLE[(first + k) % 8] for k in range(8)]

    notes, steps, skipped = [], [], []
    tool = q = None
    if state.q is not None:
        tool, q = _ToolModel(state.q, pose), list(state.q)
    elif getattr(state, "source", "") == "SIMULATED":
        tool = _assumed_flange_tcp(pose)
        q = _guess_q(tool, pose)
        if q is not None:
            notes.append("DRY-RUN estimate: no joint data, so the arm configuration is guessed from the pose and the tool tip "
                         "is assumed to be the flange. The real robot uses its live joints and real tool: expect fewer "
                         "reachable points there.")
    check_reach = q is not None
    if check_reach:
        log("[reach] checking every straight segment with inverse kinematics ("
            + ("tool turns with the arm" if follow_yaw else "tool orientation kept") + ") ...")
    else:
        notes.append("WARNING: no joint data, reachability was NOT checked.")
    r0 = R.rotvec_to_matrix(pose[3:])
    az0 = math.atan2(pose[1], pose[0])

    def corner_r(p):
        """Tool orientation at a corner: follows the azimuth of the arm (or stays as it is)."""
        if not follow_yaw:
            return r0
        return R._mm3(R.rotvec_to_matrix([0.0, 0.0, math.atan2(p[1], p[0]) - az0]), r0)

    cur, r_cur = pose[:3], r0
    path = []                                     # accepted corners in visiting order: (xyz, orientation matrix, label)
    for n, c in enumerate(order, 1):
        tgt = corner_xyz(c)
        r_c = corner_r(tgt)
        extra = ""
        if check_reach:
            ok, q_new, why = _segment_ok(tool, q, cur, tgt, should_stop, r_cur, r_c)
            if not ok:
                # The corner is out of comfortable reach from where the arm stands: ADJUST it with the same search as
                # to_limits (the robot is not obliged to reach the maximum) instead of skipping it.
                log(f"[reach] corner {n}/8 ({_corner_name(c)}) is not comfortably reachable ({why}), adjusting ...")
                rc_t = type("ReachCtx", (), {})()
                rc_t.tool, rc_t.q = tool, list(q)
                new6, _info = R._adjust_target(rc_t, list(cur) + R.matrix_to_rotvec(r_cur), list(tgt) + R.matrix_to_rotvec(r_c), lim)
                if new6 is not None:
                    tgt_new = [round(x, 4) for x in new6[:3]]
                    r_new = corner_r(tgt_new)
                    ok2, q_new2, why2 = _segment_ok(tool, q, cur, tgt_new, should_stop, r_cur, r_new)
                    if ok2:
                        extra = (f" (adjusted: the arm cannot comfortably reach ({tgt[0]:.3f}, {tgt[1]:.3f}, {tgt[2]:.3f}); "
                                 f"goes to ({tgt_new[0]:.3f}, {tgt_new[1]:.3f}, {tgt_new[2]:.3f}))")
                        notes.append(f"Corner ({_corner_name(c)}) ADJUSTED: it is not comfortably reachable from here "
                                     f"({why}); the robot goes to ({tgt_new[0]:.3f}, {tgt_new[1]:.3f}, {tgt_new[2]:.3f}) m "
                                     f"instead of ({tgt[0]:.3f}, {tgt[1]:.3f}, {tgt[2]:.3f}) m.")
                        tgt, r_c, q_new, ok = tgt_new, r_new, q_new2, True
            if not ok:
                skipped.append((c, why))
                log(f"[reach] corner {n}/8 ({_corner_name(c)}) SKIPPED: {why}")
                continue
            q = q_new
            log(f"[reach] corner {n}/8 ({_corner_name(c)}) reachable{extra}")
        cur, r_cur = tgt, r_c
        path.append((list(tgt), r_c, f"corner {n}/8"))
        rot = pose[3:] if not follow_yaw else R.matrix_to_rotvec(r_c)
        steps.append({"name": f"Corner {n}/8: {_corner_name(c)}{extra}", "action": "movel",
                      "description": f"Straight move to ({tgt[0]:.4f}, {tgt[1]:.4f}, {tgt[2]:.4f}) m, "
                                     + ("tool turns with the arm" if follow_yaw else "tool orientation kept")
                                     + f", {v:g} m/s, 1 mm inside the limits.",
                      "params": {"pose": [round(x, 9) for x in tgt] + [round(x, 9) for x in rot],
                                 "speed_m_s": v, "acceleration_m_s2": a}})
    if not steps:
        raise TourError("None of the 8 corners can be reached with the current tool orientation. Rotate the tool "
                        "(/ai rotate the tool ...) or reduce the workspace limits, then try again.\n  "
                        + "\n  ".join(f"{_corner_name(c)}: {w}" for c, w in skipped))
    if not stay:
        if check_reach:
            # Straight home if that is verified. Otherwise retrace the way we came (those segments were verified in the other
            # direction) and turn for home at the first corner from which the straight way is clear. The robot is never
            # left somewhere the software cannot bring it back from.
            idx = len(path) - 1
            while True:
                ok, _q, why = _segment_ok(tool, q, cur, pose[:3], should_stop, r_cur, r0)
                if ok:
                    break
                idx -= 1
                if idx < 0:
                    raise TourError(f"There is no verified way back to the start pose ({why}). Use '/robot stay' only if you "
                                    "can bring the robot back yourself.")
                tgt_b, r_b, label_b = path[idx]
                ok1, q_b, why1 = _segment_ok(tool, q, cur, tgt_b, should_stop, r_cur, r_b)
                if not ok1:
                    raise TourError(f"The way back through {label_b} cannot be verified ({why1}).")
                steps.append({"name": f"Back via {label_b}", "action": "movel",
                              "description": "The straight way to the start would touch the robot's own base; retrace the "
                                             "verified path instead.",
                              "params": {"pose": [round(x, 9) for x in tgt_b] + [round(x, 9) for x in R.matrix_to_rotvec(r_b)],
                                         "speed_m_s": v, "acceleration_m_s2": a}})
                notes.append(f"The straight way back from the last corner is blocked ({why}): the tour retraces its path "
                             f"through {label_b}.")
                cur, r_cur, q = tgt_b, r_b, q_b
        steps.append({"name": "Return to start", "action": "movel",
                      "description": "Go back to the exact pose the tour started from.",
                      "params": {"pose_ref": "start", "speed_m_s": v, "acceleration_m_s2": a}})
    visited = len([s for s in steps if s["name"].startswith("Corner")])
    notes.insert(0, f"{visited} of 8 corners will be visited at {v:g} m/s (max allowed {lim['max_speed']:g} m/s), "
                    f"acceleration {a:g} m/s^2." + ("" if stay else " The tour ends back at the start pose."))
    for c, why in skipped:
        notes.append(f"SKIPPED corner ({_corner_name(c)}): {why}")
    plan = {"interpretation": f"Visit the workspace corners in sequence at {v:g} m/s" + ("" if stay else " and come back to the start."),
            "operations": ["movel"],
            "assumptions": ["Built-in /robot command (no AI). Corner order follows the edges of the box, starting at the "
                            "corner nearest to the robot.",
                            "The tool turns about the vertical with the arm's azimuth around the base (the wrist does not wind up)."
                            if follow_yaw else "Tool orientation is kept at every corner.",
                            f"Every corner lies {m * 1000:g} mm inside the workspace limits."] + notes[1:],
            "steps": steps,
            "self_check": {"covers_all_requested_actions": True, "end_matches_request": True, "no_extra_steps": True}}
    return plan, notes
