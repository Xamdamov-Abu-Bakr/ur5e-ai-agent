"""robot_sender.py -- real robot access, deterministic validator, execution, home, stop.

Standard library only (Python 3.14 compatible). Live state comes from RTDE (port 30004, read only,
implemented here with sockets). URScript is sent with a plain socket to port 30002 exactly like
your proven gripper example. Dashboard (port 29999) is used ONLY for read-only status and "stop".
Nothing here powers on, releases brakes, unlocks a protective stop or resets a fault.
"""
import hashlib
from collections import deque
import math
import re
import socket
import struct
import threading
import time

import limits as L

# ----------------------------------------------------------------------------------------------
# Small math helpers (no numpy)
# ----------------------------------------------------------------------------------------------
def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


# UR5e standard DH parameters
_DH_A = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
_DH_D = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
_DH_ALPHA = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]


def fk_flange(q):
    """Forward kinematics of the UR5e flange: 4x4 matrix in the robot base frame."""
    t = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    for i in range(6):
        ct, st = math.cos(q[i]), math.sin(q[i])
        ca, sa = math.cos(_DH_ALPHA[i]), math.sin(_DH_ALPHA[i])
        t = _matmul(t, [[ct, -st * ca, st * sa, _DH_A[i] * ct],
                        [st, ct * ca, -ct * sa, _DH_A[i] * st],
                        [0, sa, ca, _DH_D[i]],
                        [0, 0, 0, 1]])
    return t


def rotvec_to_matrix(r):
    th = math.sqrt(r[0] ** 2 + r[1] ** 2 + r[2] ** 2)
    if th < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = r[0] / th, r[1] / th, r[2] / th
    c, s, v = math.cos(th), math.sin(th), 1 - math.cos(th)
    return [[c + x * x * v, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, c + z * z * v]]


def rotation_angle_between(r1, r2):
    """Angle (rad) of the rotation that takes orientation r1 to r2 (both rotation vectors)."""
    a, b = rotvec_to_matrix(r1), rotvec_to_matrix(r2)
    tr = sum(sum(a[k][i] * b[k][i] for k in range(3)) for i in range(3))   # trace(A^T B)
    return math.acos(max(-1.0, min(1.0, (tr - 1) / 2)))


def tool_offset_from_state(q, tcp_pose):
    """Position of the TCP in the flange frame, derived from the LIVE robot (no config needed)."""
    f = fk_flange(q)
    d = [tcp_pose[i] - f[i][3] for i in range(3)]
    return [sum(f[k][i] * d[k] for k in range(3)) for i in range(3)]   # R^T * d


def tcp_position_from_joints(q, tool):
    f = fk_flange(q)
    return [f[i][3] + sum(f[i][k] * tool[k] for k in range(3)) for i in range(3)]


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _in_box(p, lim, eps=1e-9):
    return (lim["x_min"] - eps <= p[0] <= lim["x_max"] + eps and lim["y_min"] - eps <= p[1] <= lim["y_max"] + eps
            and lim["z_min"] - eps <= p[2] <= lim["z_max"] + eps)


def _box_text(lim):
    return (f"X[{lim['x_min']:.4f},{lim['x_max']:.4f}] Y[{lim['y_min']:.4f},{lim['y_max']:.4f}] "
            f"Z[{lim['z_min']:.4f},{lim['z_max']:.4f}] m")


# ----------------------------------------------------------------------------------------------
# Robot state
# ----------------------------------------------------------------------------------------------
ROBOT_MODES = {-1: "NO_CONTROLLER", 0: "DISCONNECTED", 1: "CONFIRM_SAFETY", 2: "BOOTING", 3: "POWER_OFF",
               4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE", 7: "RUNNING"}
SAFETY_MODES = {1: "NORMAL", 2: "REDUCED", 3: "PROTECTIVE_STOP", 4: "RECOVERY", 5: "SAFEGUARD_STOP",
                6: "SYSTEM_EMERGENCY_STOP", 7: "ROBOT_EMERGENCY_STOP", 8: "VIOLATION", 9: "FAULT",
                10: "VALIDATE_JOINT_ID", 11: "UNDEFINED", 12: "AUTOMATIC_MODE_SAFEGUARD_STOP",
                13: "SYSTEM_THREE_POSITION_ENABLING_STOP"}
RUNTIME_STATES = {0: "STOPPING", 1: "STOPPED", 2: "PLAYING", 3: "PAUSING", 4: "PAUSED", 5: "RESUMING"}


class RobotState:
    """A snapshot. Any field the interface did not deliver is None (shown as 'unavailable')."""

    def __init__(self, source, q=None, pose=None, tcp_speed=None, qd=None, robot_mode=None,
                 safety_mode=None, runtime_state=None, token_register=None, received=None, double_register_0=None):
        self.source, self.q, self.pose, self.tcp_speed, self.qd = source, q, pose, tcp_speed, qd
        self.robot_mode, self.safety_mode, self.runtime_state = robot_mode, safety_mode, runtime_state
        self.token_register = token_register
        self.double_register_0 = double_register_0   # gripper width written by the program (-1.0 = not yet)
        self.received = time.monotonic() if received is None else received

    @property
    def age(self):
        return time.monotonic() - self.received

    def speed_norm(self):
        return None if self.tcp_speed is None else math.sqrt(sum(v * v for v in self.tcp_speed[:3]))

    def joint_speed_max(self):
        return None if self.qd is None else max(abs(v) for v in self.qd)

    def to_text(self):
        def v6(v, f="{:.5f}"):
            return "unavailable" if v is None else "[" + ", ".join(f.format(x) for x in v) + "]"
        mode = "unavailable" if self.robot_mode is None else ROBOT_MODES.get(self.robot_mode, str(self.robot_mode))
        safe = "unavailable" if self.safety_mode is None else SAFETY_MODES.get(self.safety_mode, str(self.safety_mode))
        run = "unavailable" if self.runtime_state is None else RUNTIME_STATES.get(self.runtime_state, str(self.runtime_state))
        tag = "SIMULATED (dry-run, NOT the real robot)" if self.source == "SIMULATED" else f"{self.source} (age {self.age:.2f} s)"
        return (f"source: {tag}\n  robot mode: {mode}\n  safety mode: {safe}\n  program/runtime state: {run}\n"
                f"  TCP pose p[x,y,z (m), rx,ry,rz (rad)]: {v6(self.pose, '{:.6f}')}\n"
                f"  joints (rad): {v6(self.q, '{:.5f}')}\n"
                f"  TCP speed (m/s,rad/s): {v6(self.tcp_speed, '{:.4f}')}")


# ----------------------------------------------------------------------------------------------
# RTDE client (read only) -- protocol version 2, documented UR binary protocol
# ----------------------------------------------------------------------------------------------
_RTDE_FMT = {"VECTOR6D": ">6d", "VECTOR3D": ">3d", "DOUBLE": ">d", "INT32": ">i", "UINT32": ">I",
             "UINT8": ">B", "BOOL": ">B", "UINT64": ">Q", "VECTOR6INT32": ">6i", "VECTOR6UINT32": ">6I"}
_RTDE_REQUIRED = [("actual_q", "VECTOR6D"), ("actual_TCP_pose", "VECTOR6D")]
_RTDE_OPTIONAL = [("actual_TCP_speed", "VECTOR6D"), ("actual_qd", "VECTOR6D"), ("robot_mode", "INT32"),
                  ("safety_mode", "INT32"), ("runtime_state", "UINT32"), ("output_int_register_0", "INT32"),
                  ("output_double_register_0", "DOUBLE")]


class RtdeReader:
    def __init__(self):
        self.sock = None
        self.fields = []            # [(name, type)] in packet order
        self.latest = None
        self.error = None           # last reason the stream is not working
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self.token_available = False
        self.width_available = False
        self.last_text_message = None

    # --- low level ---
    def _send(self, ptype, payload=b""):
        self.sock.sendall(struct.pack(">HB", len(payload) + 3, ptype) + payload)

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("RTDE connection closed by the controller")
            buf += chunk
        return buf

    def _recv_packet(self):
        size, ptype = struct.unpack(">HB", self._recv_exact(3))
        return ptype, self._recv_exact(size - 3)

    def _expect(self, ptype):
        while True:
            t, p = self._recv_packet()
            if t == 77:                      # text message, informational
                self.last_text_message = p[1:].decode("utf-8", "replace")
                continue
            if t == 85:                      # stray data package from an earlier recipe
                continue
            if t != ptype:
                raise ConnectionError(f"unexpected RTDE packet type {t} (wanted {ptype})")
            return p

    # --- connect ---
    def connect(self):
        self.close()
        self.error = None
        try:
            self.sock = socket.create_connection((L.ROBOT_IP, L.RTDE_PORT), timeout=L.ROBOT_CONNECT_TIMEOUT_SECONDS)
            self.sock.settimeout(L.ROBOT_READ_TIMEOUT_SECONDS)
            self._send(86, struct.pack(">H", 2))
            if self._expect(86)[0] != 1:
                raise ConnectionError("controller refused RTDE protocol version 2")
            wanted = list(_RTDE_REQUIRED) + list(_RTDE_OPTIONAL)
            for _ in range(2):
                names = ",".join(n for n, _t in wanted)
                self._send(79, struct.pack(">d", float(L.RTDE_FREQUENCY_HZ)) + names.encode())
                resp = self._expect(79)
                types = resp[1:].decode().split(",")
                missing = [n for (n, _t), ty in zip(wanted, types) if ty == "NOT_FOUND"]
                req_missing = [n for n in missing if n in dict(_RTDE_REQUIRED)]
                if req_missing:
                    raise ConnectionError(f"RTDE does not provide required fields: {req_missing}")
                if not missing:
                    break
                wanted = [(n, t) for n, t in wanted if n not in missing]      # drop optional ones
            else:
                raise ConnectionError("RTDE output setup failed")
            self.fields = wanted
            for (n, t), ty in zip(wanted, types):
                if ty != t:
                    raise ConnectionError(f"RTDE type mismatch for {n}: expected {t}, got {ty}")
            self.token_available = any(n == "output_int_register_0" for n, _t in wanted)
            self.width_available = any(n == "output_double_register_0" for n, _t in wanted)
            self._send(83)
            if self._expect(83)[0] != 1:
                raise ConnectionError("controller did not start the RTDE stream")
            self._recipe_id = resp[0]
        except (OSError, ConnectionError, struct.error, ValueError) as e:
            self.error = f"RTDE connect failed: {e}"
            self.close()
            raise ConnectionError(self.error)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="rtde-reader", daemon=True)
        self._thread.start()

    def _parse_data(self, payload):
        if not payload or payload[0] != self._recipe_id:
            return None
        off, vals = 1, {}
        for name, ty in self.fields:
            fmt = _RTDE_FMT[ty]
            n = struct.calcsize(fmt)
            v = struct.unpack(fmt, payload[off:off + n])
            vals[name] = list(v) if len(v) > 1 else v[0]
            off += n
        for key in ("actual_q", "actual_TCP_pose", "actual_TCP_speed", "actual_qd"):
            if key in vals and not all(math.isfinite(x) for x in vals[key]):
                return None
        return RobotState("RTDE", q=vals.get("actual_q"), pose=vals.get("actual_TCP_pose"),
                          tcp_speed=vals.get("actual_TCP_speed"), qd=vals.get("actual_qd"),
                          robot_mode=vals.get("robot_mode"), safety_mode=vals.get("safety_mode"),
                          runtime_state=vals.get("runtime_state"),
                          token_register=vals.get("output_int_register_0"),
                          double_register_0=vals.get("output_double_register_0"))

    def _run(self):
        try:
            while not self._stop.is_set():
                t, p = self._recv_packet()
                if t == 77:
                    self.last_text_message = p[1:].decode("utf-8", "replace")
                elif t == 85:
                    st = self._parse_data(p)
                    if st:
                        with self._lock:
                            self.latest = st
        except (OSError, ConnectionError, struct.error) as e:
            if not self._stop.is_set():
                self.error = f"RTDE stream lost: {e}"
                with self._lock:
                    self.latest = None

    def get(self, max_age=None):
        max_age = L.STATE_MAX_AGE_SECONDS if max_age is None else max_age
        with self._lock:
            st = self.latest
        return st if st is not None and st.age <= max_age else None

    def close(self):
        self._stop.set()
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        with self._lock:
            self.latest = None


# ----------------------------------------------------------------------------------------------
# Dashboard (whitelist: read-only queries + "stop")
# ----------------------------------------------------------------------------------------------
_DASH_ALLOWED = {"robotmode", "safetystatus", "programState", "stop"}


def dashboard_query(cmd, timeout=None):
    if cmd not in _DASH_ALLOWED:
        raise ValueError(f"dashboard command '{cmd}' is not whitelisted")
    timeout = timeout or L.ROBOT_READ_TIMEOUT_SECONDS
    with socket.create_connection((L.ROBOT_IP, L.DASHBOARD_PORT), timeout=timeout) as s:
        s.settimeout(timeout)
        s.recv(1024)                                  # greeting line
        s.sendall((cmd + "\n").encode())
        return s.recv(1024).decode("utf-8", "replace").strip()


# ----------------------------------------------------------------------------------------------
# Reach envelope: inverse kinematics + comfortable-reach limit + capsule self-collision model.
# Why: the workspace is a BOX in the base frame, but the arm cannot reach every point of a box with a fixed tool
# orientation, and some reachable poses press the wrist or tool into the robot's own base or the table. Sending
# such a target makes the controller stop mid-run (protective stop / fault). Everything here is deterministic, uses
# the LIVE joints and the LIVE tool offset (nothing is configured by hand), and is an APPROXIMATION: link radii are
# generous round numbers from limits.py, not the real housing shapes. It is not certified collision checking.
# ----------------------------------------------------------------------------------------------
def _t3(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def _mv(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def _solve6(a, b):
    """Gauss elimination with partial pivoting for a 6x6 system. Returns None if (nearly) singular."""
    n = 6
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            return None
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            for k in range(c, n + 1):
                m[r][k] -= f * m[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


def _flange(q):
    f = fk_flange(q)
    return [[f[i][j] for j in range(3)] for i in range(3)], [f[i][3] for i in range(3)]


def _rot_err(r_cur, r_tar):
    """Rotation vector (base frame) that turns the current orientation into the target one."""
    return matrix_to_rotvec(_mm3(r_tar, _t3(r_cur)))


def _pose_error(q, p_t, r_t):
    r, p = _flange(q)
    return [p_t[i] - p[i] for i in range(3)] + _rot_err(r, r_t)


def _jacobian(q):
    """Numerical 6x6 Jacobian of the flange pose (position, base-frame angular velocity)."""
    h = 1e-6
    r0, p0 = _flange(q)
    cols = []
    for j in range(6):
        qj = list(q)
        qj[j] += h
        r1, p1 = _flange(qj)
        dp = [(p1[i] - p0[i]) / h for i in range(3)]
        dw = [c / h for c in _rot_err(r0, r1)]
        cols.append(dp + dw)
    return [[cols[j][i] for j in range(6)] for i in range(6)]


def ik_solve(q_seed, p_t, r_t):
    """Damped least squares from q_seed to the flange pose (p_t, r_t). Returns joints or None."""
    q = list(q_seed)
    lam2 = 1e-4
    for _ in range(int(L.REACH_IK_MAX_ITER)):
        e = _pose_error(q, p_t, r_t)
        if math.sqrt(sum(x * x for x in e[:3])) <= L.REACH_IK_POS_TOL_M and \
                math.sqrt(sum(x * x for x in e[3:])) <= L.REACH_IK_ROT_TOL_RAD:
            return q
        j = _jacobian(q)
        a = [[sum(j[i][k] * j[m][k] for k in range(6)) + (lam2 if i == m else 0.0) for m in range(6)] for i in range(6)]
        y = _solve6(a, e)
        if y is None:
            return None
        dq = [sum(j[i][k] * y[i] for i in range(6)) for k in range(6)]
        step = max(abs(x) for x in dq)
        if step > 0.3:                                   # limit the step: keeps us on the same solution branch
            dq = [x * 0.3 / step for x in dq]
        q = [q[k] + dq[k] for k in range(6)]
    return None


class ToolModel:
    """TCP <-> flange conversion learned from the LIVE robot (no tool configuration is needed)."""

    def __init__(self, q, tcp_pose):
        r_f, _p_f = _flange(q)
        self.offset = tool_offset_from_state(q, tcp_pose)                    # TCP position in the flange frame
        self.r_tool = _mm3(_t3(r_f), rotvec_to_matrix(tcp_pose[3:]))         # TCP orientation in the flange frame
        self.r_tcp = rotvec_to_matrix(tcp_pose[3:])
        self.length = math.sqrt(sum(c * c for c in self.offset))

    def flange_target(self, p_tcp, r_tcp=None):
        r_tcp = self.r_tcp if r_tcp is None else r_tcp
        r_f = _mm3(r_tcp, _t3(self.r_tool))
        off = _mv(r_f, self.offset)
        return [p_tcp[i] - off[i] for i in range(3)], r_f


def tool_from_state(state):
    """ToolModel from a live state, or None when joints/pose are missing or the offset is implausible (> 0.6 m)."""
    if state is None or state.q is None or state.pose is None:
        return None
    t = ToolModel(state.q, state.pose)
    return t if t.length <= 0.6 else None


def fk_points(q):
    """Origins of joint frames 1..6 (shoulder, elbow, wrist1, wrist2, wrist3, flange) in the base frame."""
    t = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    pts = []
    for i in range(6):
        ct, st_ = math.cos(q[i]), math.sin(q[i])
        ca, sa = math.cos(_DH_ALPHA[i]), math.sin(_DH_ALPHA[i])
        t = _matmul(t, [[ct, -st_ * ca, st_ * sa, _DH_A[i] * ct], [st_, ct * ca, -ct * sa, _DH_A[i] * st_],
                        [0, sa, ca, _DH_D[i]], [0, 0, 0, 1]])
        pts.append([t[0][3], t[1][3], t[2][3]])
    return pts, t


def _seg_seg_dist(p1, q1, p2, q2):
    """Shortest distance between segments p1-q1 and p2-q2 (3D)."""
    d1, d2, r = _sub(q1, p1), _sub(q2, p2), _sub(p1, p2)
    a, e, f = _dot(d1, d1), _dot(d2, d2), _dot(d2, r)
    eps = 1e-12
    clamp = lambda x: 0.0 if x < 0.0 else 1.0 if x > 1.0 else x
    if a <= eps and e <= eps:
        return _norm(r)
    if a <= eps:
        s, t = 0.0, clamp(f / e)
    else:
        c = _dot(d1, r)
        if e <= eps:
            t, s = 0.0, clamp(-c / a)
        else:
            b = _dot(d1, d2)
            den = a * e - b * b
            s = clamp((b * f - c * e) / den) if den > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, clamp(-c / a)
            elif t > 1.0:
                t, s = 1.0, clamp((b - c) / a)
    c1 = _add(p1, _scale(d1, s))
    c2 = _add(p2, _scale(d2, t))
    return _norm(_sub(c1, c2))


def link_capsules(q, tool):
    """Capsule model of the arm: {name: (point_a, point_b, radius)} from the joint origins and the live tool offset."""
    pts, t = fk_points(q)
    r = [[t[i][j] for j in range(3)] for i in range(3)]
    tip = [pts[5][i] + sum(r[i][k] * tool.offset[k] for k in range(3)) for i in range(3)]
    return {
        "column": ([0.0, 0.0, 0.0], [0.0, 0.0, L.BASE_COLUMN_HEIGHT_M], L.BASE_COLUMN_RADIUS_M),
        "upper arm": (pts[0], pts[1], L.UPPER_ARM_RADIUS_M),
        "forearm": (pts[1], pts[2], L.FOREARM_RADIUS_M),
        "wrist 1": (pts[2], pts[3], L.WRIST_RADIUS_M),
        "wrist 2": (pts[3], pts[4], L.WRIST_RADIUS_M),
        "wrist 3": (pts[4], pts[5], L.WRIST_RADIUS_M),
        "tool": (pts[5], tip, L.TOOL_RADIUS_M),
    }


# pairs that can actually touch (neighbouring links are always in contact and are skipped)
_COLLISION_PAIRS = [("column", "forearm"), ("column", "wrist 1"), ("column", "wrist 2"), ("column", "wrist 3"),
                    ("column", "tool"), ("upper arm", "wrist 2"), ("upper arm", "wrist 3"), ("upper arm", "tool"),
                    ("forearm", "wrist 3"), ("forearm", "tool")]


def self_collision_problem(q, tool):
    """None if clear, else a short description of the first contact found (links, floor)."""
    caps = link_capsules(q, tool)
    m = L.SELF_COLLISION_MARGIN_M
    for a, b in _COLLISION_PAIRS:
        pa, qa, ra = caps[a]
        pb, qb, rb = caps[b]
        gap = _seg_seg_dist(pa, qa, pb, qb) - ra - rb - m
        if gap < 0:
            return f"self-collision: {a} against {b} (about {-gap * 1000:.0f} mm too close)"
    for name, (pa, qa, ra) in caps.items():
        if name == "column":
            continue
        low = min(pa[2], qa[2]) - (0.0 if name == "tool" else ra)    # the tool tip is limited by Z_MIN instead
        if low < L.FLOOR_Z_M + (0.0 if name == "tool" else m):
            return f"{name} would touch the table (about {(L.FLOOR_Z_M - low) * 1000 + 1:.0f} mm too low)"
    return None


def stretch_fraction(q):
    """How far the arm is stretched: shoulder-to-wrist-1 distance / (upper arm + forearm). 1.0 = fully stretched."""
    pts, _t = fk_points(q)
    return _dist(pts[0], pts[2]) / (abs(_DH_A[1]) + abs(_DH_A[2]))


def envelope_problem(q, tool, q_prev=None):
    """None when the pose is inside the envelope, else WHY not (joint limits, singularity, too stretched, collision)."""
    lo = [a for a, _b in L.JOINT_LIMITS_RAD]
    hi = [b for _a, b in L.JOINT_LIMITS_RAD]
    for i in range(6):
        if not lo[i] <= q[i] <= hi[i]:
            return (f"joint {i + 1} would leave its allowed range (it would need {q[i]:.2f} rad, the limit is "
                    f"[{lo[i]:g}, {hi[i]:g}] rad)")
    if q_prev is not None:
        jumps = [abs(q[i] - q_prev[i]) for i in range(6)]
        if max(jumps) > L.REACH_MAX_JOINT_JUMP_RAD:
            j = jumps.index(max(jumps))
            return (f"joint {j + 1} would have to jump {math.degrees(jumps[j]):.0f} degrees in one step (limit "
                    f"{math.degrees(L.REACH_MAX_JOINT_JUMP_RAD):.0f}): the arm would have to flip its configuration, "
                    f"or the tool rotation per step may be too large")
    sm = math.sin(L.REACH_SINGULARITY_MARGIN_RAD)
    if abs(math.sin(q[2])) < sm:
        return "the elbow would be almost fully stretched or folded (singular)"
    if abs(math.sin(q[4])) < sm:
        return "wrist 2 would be close to a singular pose"
    sf = stretch_fraction(q)
    if sf > L.REACH_COMFORT_FRACTION:
        return (f"the arm would be stretched to {sf * 100:.0f}% of its maximum reach (comfort limit "
                f"{L.REACH_COMFORT_FRACTION * 100:.0f}%)")
    return self_collision_problem(q, tool)


def reach_pose(tool, q_seed, p_tcp, r_tcp=None):
    """(q, None) if the TCP pose is inside the envelope, else (None, reason). r_tcp: 3x3 matrix or None (keep live one)."""
    pf, rf = tool.flange_target(p_tcp, r_tcp)
    q = ik_solve(q_seed, pf, rf)
    if q is None:
        return None, "no joint solution (outside the arm's reach or a singularity)"
    why = envelope_problem(q, tool, q_seed)
    return (None, why) if why else (q, None)


def _slerp(r0, r1, t):
    w = _rot_err(r0, r1)
    return _mm3(rotvec_to_matrix([c * t for c in w]), r0)


def reach_walk(tool, q, pts, r_from, r_to, should_stop=None):
    """Walk a list of TCP points (orientation turns smoothly from r_from to r_to). Returns (ok, q_end, reason, t_fail)
    where t_fail in 0..1 is how far along the path the first problem is."""
    n = len(pts) - 1
    need = math.ceil(_matrix_angle(r_from, r_to) / L.REACH_MAX_ROT_STEP_RAD - 1e-9)
    if n < 1 and need > 0:
        pts, n = [pts[0], pts[0]], 1
    if n >= 1 and need > n:                               # a long tool turn on a short path: add samples (no big jumps)
        m = math.ceil(need / n)
        dense = [pts[0]]
        for k in range(n):
            for j in range(1, m + 1):
                dense.append([pts[k][c] + (pts[k + 1][c] - pts[k][c]) * j / m for c in range(3)])
        pts, n = dense, len(dense) - 1
    for k in range(1, n + 1):
        if should_stop is not None and should_stop():
            return False, q, "stopped", k / max(n, 1)
        rr = r_to if k == n else _slerp(r_from, r_to, k / n)
        qn, why = reach_pose(tool, q, pts[k], rr)
        if qn is None:
            return False, q, why, (0.0 if k == 1 else k / n)    # t = 0: the very start of the move
        q = qn
    return True, q, "", 1.0


def keepout_hit(pts, start=None):
    """First TCP point that enters the base keep-out cylinder (independent of REACH_CHECK_ENABLED), or None.
    A path that STARTS inside (e.g. jogged there by hand) may leave it, but never get closer to the axis than it began."""
    r_start = math.hypot(start[0], start[1]) if start is not None else None
    lim_r = L.BASE_KEEPOUT_RADIUS_M if r_start is None else min(L.BASE_KEEPOUT_RADIUS_M, r_start - 1e-6)
    for pt in pts:
        if pt[2] < L.BASE_KEEPOUT_HEIGHT_M and math.hypot(pt[0], pt[1]) < lim_r:
            return pt
    return None


def _matrix_angle(a, b):
    """Angle (rad) between two orientations given as 3x3 matrices: acos((trace(A^T B) - 1) / 2)."""
    tr = sum(a[k][i] * b[k][i] for k in range(3) for i in range(3))
    return math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))


def straight_points(p_from, p_to, step=None, r_from=None, r_to=None):
    """Sample points of a straight move. With orientations (3x3 matrices) the count also grows with the rotation angle
    (at most REACH_MAX_ROT_STEP_RAD per step), so an in-place tool turn is checked in small steps, not in one jump."""
    step = step or L.REACH_SAMPLE_STEP_M
    n = max(1, math.ceil(_dist(p_from, p_to) / step))
    if r_from is not None and r_to is not None:
        n = max(n, math.ceil(_matrix_angle(r_from, r_to) / L.REACH_MAX_ROT_STEP_RAD - 1e-9))
    return [[p_from[i] + (p_to[i] - p_from[i]) * k / n for i in range(3)] for k in range(n + 1)]


def reach_straight(tool, q, p_from, r_from, p_to, r_to, should_stop=None):
    return reach_walk(tool, q, straight_points(p_from, p_to, None, r_from, r_to), r_from, r_to, should_stop)


def reach_edge(tool, q, p_from, r, dvec, max_dist, resolution=0.001):
    """How far the TCP can travel from p_from along the unit vector dvec (orientation kept) before the envelope ends.
    Returns (distance_m, q_at_that_distance, reason_at_the_edge or '' when max_dist is reachable)."""
    step = L.REACH_SAMPLE_STEP_M
    good, q_good, why, d = 0.0, q, "", 0.0
    while d < max_dist - 1e-12:
        d2 = min(max_dist, d + step)
        p = [p_from[i] + dvec[i] * d2 for i in range(3)]
        qn, w = reach_pose(tool, q_good, p, r)
        if qn is None:
            why = w
            lo, hi = good, d2                                  # refine inside the last coarse step
            while hi - lo > resolution:
                mid = (lo + hi) / 2.0
                qm, _w = reach_pose(tool, q_good, [p_from[i] + dvec[i] * mid for i in range(3)], r)
                if qm is not None:
                    lo, q_good = mid, qm
                else:
                    hi = mid
            return lo, q_good, why
        good, q_good, d = d2, qn, d2
    return good, q_good, ""


# ----------------------------------------------------------------------------------------------
# More geometry (rotations, circles) -- pure stdlib, deterministic. The LLM never does this math.
# ----------------------------------------------------------------------------------------------
def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


def _scale(a, k):
    return [a[i] * k for i in range(3)]


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _norm(a):
    return math.sqrt(_dot(a, a))


def _mm3(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matrix_to_rotvec(m):
    """3x3 rotation matrix -> rotation vector (rad), robust near 0 and pi."""
    cosang = max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1.0) / 2.0))
    ang = math.acos(cosang)
    if ang < 1e-9:
        return [0.0, 0.0, 0.0]
    if abs(ang - math.pi) < 1e-5:                       # axis from the symmetric part
        x, y, z = (math.sqrt(max(0.0, (m[i][i] + 1.0) / 2.0)) for i in range(3))
        big = max(range(3), key=lambda i: (x, y, z)[i])
        v = [x, y, z]
        for j in range(3):
            if j != big:
                v[j] = math.copysign(v[j], m[big][j] + m[j][big])
        n = _norm(v)
        return [ang * c / n for c in v]
    k = ang / (2.0 * math.sin(ang))
    return [k * (m[2][1] - m[1][2]), k * (m[0][2] - m[2][0]), k * (m[1][0] - m[0][1])]


def rotate_about_base_axis(rotvec, axis, angle_rad):
    """New TCP orientation = rotation about a BASE-frame axis applied to the current orientation."""
    ax = {"X": [1, 0, 0], "Y": [0, 1, 0], "Z": [0, 0, 1]}[axis]
    rd = rotvec_to_matrix([c * angle_rad for c in ax])
    return matrix_to_rotvec(_mm3(rd, rotvec_to_matrix(rotvec)))


def circle_through(a, b, c):
    """Circle through three 3D points, traversed a -> b -> c. Returns dict or None if degenerate."""
    ab, ac = _sub(b, a), _sub(c, a)
    n = _cross(ab, ac)
    nn, lab, lac = _norm(n), _norm(ab), _norm(ac)
    if lab < 1e-4 or lac < 1e-4 or nn / (lab * lac) < 1e-3:
        return None
    n2 = nn * nn
    t1 = _scale(_cross(n, ab), _dot(ac, ac))
    t2 = _scale(_cross(ac, n), _dot(ab, ab))
    center = _add(a, _scale(_add(t1, t2), 1.0 / (2.0 * n2)))
    r = _norm(_sub(a, center))
    u = _scale(_sub(a, center), 1.0 / r)
    w = _cross(_scale(n, 1.0 / nn), u)

    def ang(p):
        d = _sub(p, center)
        return math.atan2(_dot(d, w), _dot(d, u)) % (2 * math.pi)
    tb, tc = ang(b), ang(c)
    sign, sweep = (1, tc) if tb < tc else (-1, 2 * math.pi - tc)
    return {"center": center, "radius": r, "u": u, "w": w, "sign": sign, "sweep": sweep, "length": r * sweep}


def arc_samples(g, step):
    n = max(8, math.ceil(g["length"] / step))
    pts = []
    for k in range(n + 1):
        phi = g["sign"] * g["sweep"] * k / n
        pts.append([g["center"][i] + g["radius"] * (g["u"][i] * math.cos(phi) + g["w"][i] * math.sin(phi)) for i in range(3)])
    return pts


def build_arc_points(start_pose, arc):
    """Deterministic arc builder used by the planner's `arc` spec.
    plane XY|XZ|YZ; center_dir = axis direction from the START point to the circle centre (e.g. "+X");
    turn_toward = in-plane axis direction (perpendicular to center_dir) in which the path first moves (e.g. "+Z");
    radius_m; sweep_deg (default 180). Returns (via_pose, to_pose) or raises ValueError."""
    axes = {"X": 0, "Y": 1, "Z": 2}

    def d(text, what):
        if not (isinstance(text, str) and len(text) == 2 and text[0] in "+-" and text[1] in axes):
            raise ValueError(f"arc.{what} must look like '+X', '-Z' ...")
        v = [0.0, 0.0, 0.0]
        v[axes[text[1]]] = 1.0 if text[0] == "+" else -1.0
        return v, text[1]
    plane = arc.get("plane")
    if plane not in ("XY", "XZ", "YZ"):
        raise ValueError("arc.plane must be XY, XZ or YZ")
    d1, a1 = d(arc.get("center_dir"), "center_dir")
    d2, a2 = d(arc.get("turn_toward"), "turn_toward")
    if a1 == a2 or a1 not in plane or a2 not in plane:
        raise ValueError(f"center_dir and turn_toward must be two DIFFERENT axes of plane {plane}")
    r = arc.get("radius_m")
    sweep = arc.get("sweep_deg", 180.0)
    if not (isinstance(r, (int, float)) and not isinstance(r, bool) and math.isfinite(r)):
        raise ValueError("arc.radius_m must be a number (metres)")
    if not (isinstance(sweep, (int, float)) and not isinstance(sweep, bool) and math.isfinite(sweep)):
        raise ValueError("arc.sweep_deg must be a number (degrees)")
    s = start_pose[:3]
    center = _add(s, _scale(d1, r))

    def pt(phi):
        return [center[i] + r * (-d1[i] * math.cos(phi) + d2[i] * math.sin(phi)) for i in range(3)]
    via = pt(math.radians(sweep) / 2.0) + list(start_pose[3:])
    to = pt(math.radians(sweep)) + list(start_pose[3:])
    return [round(v, 6) for v in via], [round(v, 6) for v in to]


# ----------------------------------------------------------------------------------------------
# Directions and geometry (deterministic; the model names a shape, software computes every point)
# ----------------------------------------------------------------------------------------------
AXIS_VEC = {"X": [1.0, 0.0, 0.0], "Y": [0.0, 1.0, 0.0], "Z": [0.0, 0.0, 1.0]}


def _parse_axis_dir(text):
    """'+X' / '-Z' -> (vector, letter) or None."""
    if isinstance(text, str) and len(text) == 2 and text[0] in "+-" and text[1] in AXIS_VEC:
        sign = 1.0 if text[0] == "+" else -1.0
        return [sign * c for c in AXIS_VEC[text[1]]], text[1]
    return None


def direction_vector(text):
    """Axis text ('+X') or a word defined in DIRECTION_CONVENTION ('forward' -> '+X' once the user set it).
    Returns a unit vector in the BASE frame, or None if the word is not defined (the planner must then ask)."""
    if isinstance(text, str) and text in L.DIRECTION_CONVENTION:
        text = L.DIRECTION_CONVENTION[text]
        if text is None:
            return None
    hit = _parse_axis_dir(text)
    return None if hit is None else hit[0]


def _axis_unit(dvec):
    """(axis_index, sign) if dvec is a single base-axis unit vector, else None."""
    nz = [i for i in range(3) if abs(dvec[i]) > 1e-9]
    if len(nz) != 1:
        return None
    return nz[0], (1.0 if dvec[nz[0]] > 0 else -1.0)


class _Reach:
    """Reach bookkeeping for ONE resolve/validate run: the live tool model and the joint configuration, advanced along
    the plan. `on` is False when joints or tool are unknown, REACH_CHECK_ENABLED is off, or an earlier step already failed
    (later steps would only show consequences of that first failure)."""

    def __init__(self, state):
        self.tool = tool_from_state(state) if (L.REACH_CHECK_ENABLED and state is not None) else None
        self.q = list(state.q) if self.tool is not None else None
        self.broken = False

    @property
    def on(self):
        return self.tool is not None and self.q is not None and not self.broken

    def advance(self, p_from, p_to):
        """Follow a straight move (TCP poses with rotation vectors). Returns (ok, why, t_fail)."""
        if not self.on:
            return True, "", 1.0
        ok, qe, why, t = reach_straight(self.tool, self.q, p_from[:3], rotvec_to_matrix(p_from[3:]),
                                        p_to[:3], rotvec_to_matrix(p_to[3:]))
        if ok:
            self.q = qe
        else:
            self.broken = True
        return ok, why, t

    def advance_points(self, pts, rv_from, rv_to):
        if not self.on:
            return True, "", 1.0
        ok, qe, why, t = reach_walk(self.tool, self.q, pts, rotvec_to_matrix(rv_from), rotvec_to_matrix(rv_to))
        if ok:
            self.q = qe
        else:
            self.broken = True
        return ok, why, t


def room_detail(pose, dvec, rc=None):
    """How far the TCP can still travel along ONE base axis. dict(room, by, why, box):
    by = 'workspace' (box edge minus LIMIT_MARGIN_M) or 'reach' (the arm's comfortable envelope ends first; room is then
    measured to REACH_ADJUST_BUFFER_M before that edge). room is None for a non-axis direction."""
    au = _axis_unit(dvec) if dvec is not None else None
    if au is None or pose is None:
        return {"room": None, "by": "", "why": "", "box": None}
    ax, sg = au
    lim = L.get_limits()
    name = "xyz"[ax]
    box = ((lim[f"{name}_max"] - L.LIMIT_MARGIN_M) - pose[ax]) if sg > 0 else (pose[ax] - (lim[f"{name}_min"] + L.LIMIT_MARGIN_M))
    if box <= 0:
        return {"room": box, "by": "workspace", "why": "", "box": box}
    # base keep-out cylinder (checked even when REACH_CHECK_ENABLED is False): where along this axis does the path enter it?
    step, ko_dist, k = 0.01, box, 0
    pts = [[pose[c] + float(dvec[c]) * j * step for c in range(3)] for j in range(int(box / step) + 2)]
    hit = keepout_hit(pts, pose[:3])
    if hit is not None:
        k = pts.index(hit)
        ko_dist = min(box, max(0.0, (k - 1) * step))
    limit_d = min(box, ko_dist)
    if rc is not None and rc.on:
        dist, _q, why = reach_edge(rc.tool, rc.q, pose[:3], rotvec_to_matrix(pose[3:]), [float(c) for c in dvec], limit_d)
        if why:
            return {"room": max(0.0, dist - L.REACH_ADJUST_BUFFER_M), "by": "reach", "why": why, "box": box}
    if ko_dist < box:
        return {"room": max(0.0, ko_dist - L.REACH_ADJUST_BUFFER_M), "by": "reach",
                "why": f"the robot's own base column (keep-out radius {L.BASE_KEEPOUT_RADIUS_M:.2f} m)", "box": box}
    return {"room": box, "by": "workspace", "why": "", "box": box}


def edge_name(det):
    """What ends the room, in exact words (room_detail marks every non-workspace reason with by='reach')."""
    if det["by"] != "reach":
        return "the workspace edge"
    why = det.get("why", "")
    if "base column" in why:
        return "the robot's own base column"
    if "self-collision" in why:
        return "the point where the arm would touch itself"
    return "the edge of the arm's comfortable reach"


def room_along(pose, dvec, rc=None):
    """Metres the TCP can still travel along one base axis (workspace edge, or the arm's comfortable reach if that
    ends first). None for a non-axis direction. Zero or negative when already at / beyond the edge."""
    return room_detail(pose, dvec, rc)["room"]


_ROOM_CACHE = {}


def room_report(state):
    """One line for the planner: how far the TCP can go in each base direction (marked '(arm reach)' where the arm's
    comfortable reach ends before the workspace limit does)."""
    if state is None or state.pose is None:
        return "unavailable"
    key = (tuple(round(x, 4) for x in state.pose), tuple(round(x, 3) for x in (state.q or [])),
           tuple(sorted(L.get_limits().items())), L.REACH_CHECK_ENABLED)
    if key in _ROOM_CACHE:
        return _ROOM_CACHE[key]
    rc = _Reach(state)
    parts = []
    for label, vec in (("+X", (1, 0, 0)), ("-X", (-1, 0, 0)), ("+Y", (0, 1, 0)), ("-Y", (0, -1, 0)),
                       ("+Z", (0, 0, 1)), ("-Z", (0, 0, -1))):
        d = room_detail(state.pose, list(vec), rc)
        tag = ""
        if d["by"] == "reach":
            tag = (" (base column)" if "base column" in d["why"] else " (self-collision)" if "self-collision" in d["why"]
                   else " (arm reach)")
        parts.append(f"{label} {max(d['room'], 0.0) * 100:.1f} cm" + tag)
    if len(_ROOM_CACHE) > 16:
        _ROOM_CACHE.clear()
    _ROOM_CACHE[key] = ", ".join(parts)
    return _ROOM_CACHE[key]


def reach_adjust_text(requested_what, stopped_what, why, box_text=None):
    """The sentence the operator reads in the plan whenever a target was moved because of the arm's reach."""
    return (f"ADJUSTED: {requested_what}; {stopped_what}. This is NOT the workspace limit"
            + (f" ({box_text})" if box_text else "")
            + f": the robot finds that position hard to reach ({why}), so it goes as far as it comfortably can and stops "
              f"{L.REACH_ADJUST_BUFFER_M * 100:.0f} cm before the edge of its reach (it never goes to the maximum).")


def _endpoint_ok(tool, q_seed, p, r_tcp):
    """Cheap screening: is the pose itself inside the envelope (IK + capsules)? The way TO it is NOT checked here."""
    pf, rf = tool.flange_target(p, r_tcp)
    q = ik_solve(q_seed, pf, rf)
    return q is not None and envelope_problem(q, tool, None) is None


def _adjust_target(rc, cur, tgt, lim):
    """Make a TCP target comfortable. Returns (new_target or None, info or None).
    If the straight way to tgt is inside the envelope it is returned unchanged. Otherwise the best compromise is searched
    in the vertical half-plane of the target (same azimuth around the base axis; the radius and the height may both
    shrink): for each radius, from the requested one inward, the height nearest to the request that the arm can reach is
    found by bisection (cheap endpoint screening), the best few candidates are then verified along the real path, and a
    pull-back along the straight way is the last candidate. The candidate NEAREST to the request (3D) wins; if keeping the
    requested X/Y ("height") is at most 15% worse than the best, it is preferred. Every result keeps REACH_ADJUST_BUFFER_M
    inside the edge of the arm's reach. new_target is None if nothing can be done."""
    r0, r1 = rotvec_to_matrix(cur[3:]), rotvec_to_matrix(tgt[3:])

    def walk(p):                                          # reach envelope AND the base keep-out cylinder
        res = reach_straight(rc.tool, rc.q, cur[:3], r0, p, r1)
        if res[0] and keepout_hit(straight_points(cur[:3], p), cur[:3]) is not None:
            return False, res[1], "the robot's own base column", 0.0
        return res

    ok, _qe, why, _t = walk(tgt[:3])
    if ok:
        return list(tgt), None
    buf = L.REACH_ADJUST_BUFFER_M
    z_floor = lim["z_min"] + L.LIMIT_MARGIN_M
    az, rho_req = math.atan2(tgt[1], tgt[0]), math.hypot(tgt[0], tgt[1])
    rho_min = max(L.BASE_KEEPOUT_RADIUS_M + 0.01, 0.15)
    at = lambda rho, z, same: [tgt[0], tgt[1], z] if same else [rho * math.cos(az), rho * math.sin(az), z]
    rough, best_d, k = [], float("inf"), 0
    while True:
        rho = rho_req - k * 0.04
        if rho < rho_min or (k > 0 and rho_req - rho >= best_d):
            break
        same = k == 0
        for za in (cur[2], z_floor, (cur[2] + z_floor) / 2.0):
            if abs(za - tgt[2]) < 1e-6 or not _endpoint_ok(rc.tool, rc.q, at(rho, za, same), r1):
                continue
            good, bad = za, tgt[2]
            for _ in range(6):
                mid = (good + bad) / 2.0
                if _endpoint_ok(rc.tool, rc.q, at(rho, mid, same), r1):
                    good = mid
                else:
                    bad = mid
            p = at(rho, good, same)
            d = _dist(p, tgt[:3])
            rough.append((d, p, same))
            best_d = min(best_d, d)
            break
        k += 1
    cands = {}                                             # mode -> (new target, 3D distance to the request)

    def offer(mode, new):
        d = _dist(new[:3], tgt[:3])
        if mode not in cands or d < cands[mode][1]:
            cands[mode] = (new, d)

    verified = 0
    for d, p, same in sorted(rough, key=lambda x: x[0]):   # verify the best few along the real path
        if verified >= 4 or ("height" in cands and "inward" in cands):
            break
        mode = "height" if same else "inward"
        if mode in cands or not walk(p)[0]:
            continue
        verified += 1
        v = _sub(cur[:3], p)
        nv = _norm(v)
        p2 = _add(p, _scale(v, buf / nv)) if nv > buf else p   # keep REACH_ADJUST_BUFFER_M away from the edge
        offer(mode, (p2 if walk(p2)[0] else p) + list(tgt[3:]))
    d = _sub(tgt[:3], cur[:3])
    total = _norm(d)
    why2 = ""
    if total >= 1e-6:
        u = _scale(d, 1.0 / total)
        dist, _q, why2 = reach_edge(rc.tool, rc.q, cur[:3], r0, u, total)
        hit = keepout_hit(straight_points(cur[:3], tgt[:3]), cur[:3])      # stop before the base keep-out cylinder too
        if hit is not None:
            dist = min(dist, _dist(cur[:3], hit))
        d_ok = max(0.0, dist - buf)
        if d_ok >= 0.002:
            frac = d_ok / total
            new = _add(cur[:3], _scale(u, d_ok)) + (matrix_to_rotvec(_slerp(r0, r1, frac)) if frac < 1.0 else list(tgt[3:]))
            if walk(new[:3])[0]:
                offer("pull_back", new)
    if not cands:
        return None, {"mode": "none", "requested": list(tgt[:3]), "adjusted": list(cur[:3]), "why": why2 or why}
    best_mode = min(cands, key=lambda m: cands[m][1])
    if "height" in cands and cands["height"][1] <= cands[best_mode][1] * 1.15 + 1e-9:
        best_mode = "height"
    new = cands[best_mode][0]
    return new, {"mode": best_mode, "requested": list(tgt[:3]), "adjusted": new[:3], "why": why2 or why}


def adjust_note(info):
    """What the operator reads when a target was moved because of the arm's reach (plain words, numbers included)."""
    req, new = info["requested"], info["adjusted"]
    r_req = math.hypot(req[0], req[1])
    if info["mode"] == "height":
        asked = f"you asked for height {req[2]:.3f} m at X={req[0]:.3f}, Y={req[1]:.3f} ({r_req:.2f} m from the base axis)"
        got = f"the robot goes to height {new[2]:.3f} m there (X and Y unchanged)"
    else:
        asked = f"you asked for ({req[0]:.3f}, {req[1]:.3f}, {req[2]:.3f}) m ({r_req:.2f} m from the base axis)"
        got = f"the robot goes to ({new[0]:.3f}, {new[1]:.3f}, {new[2]:.3f}) m instead"
    return reach_adjust_text(asked, got, info["why"], _box_text(L.get_limits()))


def base_rotate_joints(br, q, tool):
    """Joint targets for "turn the base". Only joint 1 changes, so the arm keeps its shape and the TCP travels on a circle
    around the base axis. `via` is the side the arm swings through (+Y or -Y, never over the top). Returns (joints, text)
    or (None, error). Needs the LIVE joints, so it is only possible as the first motion of a plan."""
    if not isinstance(br, dict):
        return None, 'base_rotate must be {"angle_deg": 180, "via": "+Y" or "-Y"}.'
    ang = br.get("angle_deg", 180)
    via = str(br.get("via", "")).replace(" ", "").lower()
    if not _num(ang) or not 0 < ang <= 360:
        return None, "base_rotate.angle_deg must be a number in (0, 360]."
    if via not in ("+y", "-y"):
        return None, 'base_rotate.via must be "+Y" or "-Y": the side the arm swings through (not over +Z).'
    if q is None or tool is None:
        return None, ("base_rotate needs the live joint angles and tool, so it must be the FIRST motion of the plan "
                      "(after a movel/movec the joints are unknown).")
    want_pos = via == "+y"
    best = None
    for s in (1.0, -1.0):
        ys = []
        for k in range(37):
            qq = list(q)
            qq[0] = q[0] + s * math.radians(ang) * k / 36.0
            ys.append(tcp_position_from_joints(qq, tool)[1])
        score = max(ys) if want_pos else -min(ys)
        if best is None or score > best[0] + 1e-9:
            best = (score, s)
    target = list(q)
    target[0] = q[0] + best[1] * math.radians(ang)
    lo1, hi1 = L.JOINT_LIMITS_RAD[0]
    if not lo1 <= target[0] <= hi1:
        return None, (f"base_rotate through {'+Y' if want_pos else '-Y'} would need joint 1 = {target[0]:.3f} rad, but the "
                      f"joint 1 limit is [{lo1:g}, {hi1:g}] rad (it is at {q[0]:.3f} rad now). Return the base by turning "
                      f"through the SAME side (via) it came from, or turn by a smaller angle.")
    p0, p1 = tcp_position_from_joints(q, tool), tcp_position_from_joints(target, tool)
    radius = math.hypot(p0[0], p0[1])
    return target, (f"Base rotation {ang:g} degrees through {'+Y' if want_pos else '-Y'}: only joint 1 turns "
                    f"({q[0]:.3f} -> {target[0]:.3f} rad), the arm keeps its shape. TCP goes from ({p0[0]:.3f}, {p0[1]:.3f}, "
                    f"{p0[2]:.3f}) to ({p1[0]:.3f}, {p1[1]:.3f}, {p1[2]:.3f}) m on a circle of radius {radius:.3f} m "
                    f"around the base axis.")


def fit_problems(plan, state, refs=None):
    """Messages for moves that do not fit (workspace room or the arm's reach). The planner must ask the user first."""
    steps, findings = resolve_plan(plan, state, refs=refs or {})
    out = [f.message for f in findings if f.param == "fit"]
    if out or not steps or not L.REACH_CHECK_ENABLED:
        return out
    # nothing wrong with the room: check whether the resolved program is inside the reach envelope
    codes = {st["id"]: expected_line(st) for st in steps}
    res, _m = validate_candidate(steps, codes, state, [])
    for f in res.findings:
        if f.param == "reach" and f.kind != "fatal":
            out.append(f.message + " Ask the user: shapes, arcs, returns to a saved pose and joint moves cannot be adjusted "
                       "automatically. Choose a smaller, nearer or lower target, or cancel.")
            break
    return out


def _neg_axis_text(text):
    return ("-" if text[0] == "+" else "+") + text[1]


def expand_polygon(cur, poly, speed, accel):
    """Regular polygon through the CURRENT TCP point. Returns (list of movel step dicts, error or None).
    poly: {"sides": n, "side_m": s, "plane": "XY|XZ|YZ", "start_direction": "+X", "turn": "ccw|cw"}.
    The first side goes along start_direction; every corner turns by 360/n in the chosen sense.
    The last corner closes exactly on the start point (checked to 1 micrometre)."""
    if not isinstance(poly, dict):
        return None, "polygon must be an object."
    n, side, plane = poly.get("sides"), poly.get("side_m"), poly.get("plane")
    sd, turn = poly.get("start_direction"), poly.get("turn", "ccw")
    if not (isinstance(n, int) and not isinstance(n, bool) and 3 <= n <= 12):
        return None, "polygon.sides must be an integer 3..12."
    if not (_num(side) and 0.0 < side <= 0.5):
        return None, "polygon.side_m must be a number in (0, 0.5] metres."
    if plane not in ("XY", "XZ", "YZ"):
        return None, "polygon.plane must be XY, XZ or YZ."
    if turn not in ("ccw", "cw"):
        return None, 'polygon.turn must be "ccw" or "cw" (counter-clockwise / clockwise seen from the plane normal).'
    d = _parse_axis_dir(sd)
    if d is None or d[1] not in plane:
        return None, f"polygon.start_direction must be an axis of the plane {plane}, e.g. '+X' or '-Y'."
    a_ax, b_ax = plane[0], plane[1]
    va, vb = AXIS_VEC[a_ax], AXIS_VEC[b_ax]
    da = sum(d[0][i] * va[i] for i in range(3))
    db = sum(d[0][i] * vb[i] for i in range(3))
    phi0 = math.atan2(db, da)
    sign = 1.0 if turn == "ccw" else -1.0
    ext = 2.0 * math.pi / n
    p0 = list(cur[:3])
    v = list(p0)
    kids = []
    for k in range(n):
        phi = phi0 + sign * ext * k
        step = [side * (math.cos(phi) * va[i] + math.sin(phi) * vb[i]) for i in range(3)]
        nxt = [v[i] + step[i] for i in range(3)]
        if k == n - 1:
            if max(abs(nxt[i] - p0[i]) for i in range(3)) > 1e-6:
                return None, "internal: polygon does not close on the start point."
            nxt = p0                               # exact closure
        kids.append({"name": f"Side {k + 1}", "action": "movel",
                     "description": f"{side * 1000:.1f} mm side {k + 1} of {n}.",
                     "params": {"pose": [round(x, 9) for x in nxt] + [round(x, 9) for x in cur[3:]],
                                "speed_m_s": speed, "acceleration_m_s2": accel}})
        v = nxt
    return kids, None


def expand_circle(cur, circ, speed, accel):
    """Full circle through the CURRENT TCP point as two 180-degree movec arcs (each within the arc limits).
    circ: {"plane": "XZ", "center_dir": "+X", "turn_toward": "+Z", "radius_m": r}. The centre lies radius_m from the
    start along center_dir; the first half moves toward turn_toward. The second half continues around the circle."""
    if not isinstance(circ, dict):
        return None, "circle must be an object."
    plane, cdir, tdir, r = circ.get("plane"), circ.get("center_dir"), circ.get("turn_toward"), circ.get("radius_m")
    if not (_num(r) and r > 0):
        return None, "circle.radius_m must be a positive number (metres)."
    arc1 = {"plane": plane, "center_dir": cdir, "turn_toward": tdir, "radius_m": r, "sweep_deg": 180.0}
    try:
        via1, to1 = build_arc_points(cur, arc1)
        arc2 = {"plane": plane, "center_dir": _neg_axis_text(cdir), "turn_toward": _neg_axis_text(tdir),
                "radius_m": r, "sweep_deg": 180.0}
        via2, to2 = build_arc_points(to1, arc2)
    except (ValueError, TypeError) as e:
        return None, f"Invalid circle: {e}"
    if max(abs(to2[i] - cur[i]) for i in range(3)) > 1e-5:
        return None, "internal: circle does not close on the start point."
    end_pose = [round(x, 9) for x in cur[:3]] + [round(x, 9) for x in cur[3:]]
    kids = [
        {"name": "Half circle 1", "action": "movec", "description": f"180 deg arc, radius {r * 1000:.1f} mm.",
         "params": {"via_pose": via1, "to_pose": to1, "speed_m_s": speed, "acceleration_m_s2": accel,
                    "blend_m": min(CIRCLE_JOINT_BLEND_M, 0.25 * r)}},
        {"name": "Half circle 2", "action": "movec", "description": "Other 180 deg, back to the start point.",
         "params": {"via_pose": via2, "to_pose": end_pose, "speed_m_s": speed, "acceleration_m_s2": accel,
                    "blend_m": 0.0}},
    ]
    return kids, None


GEOMETRY_ACTIONS = ("polygon", "circle")


# ----------------------------------------------------------------------------------------------
# Capability registry: the ONLY primitives that exist. The planner prompt and the code templates
# are generated from this, so nothing can be advertised that the executor does not implement.
# ----------------------------------------------------------------------------------------------
CAPABILITIES = {
    "movel": {
        "what": "Straight-line TCP move.",
        "when": "Any straight Cartesian move, relative shift, return to a saved pose, or rotating the tool in place.",
        "params": 'EXACTLY ONE of: {"delta_xyz_m":[dx,dy,dz]} | {"pose":[x,y,z,rx,ry,rz]} | {"xyz_m":[x,y,z]} (absolute position, keeps the CURRENT tracked orientation: use it after base_rotate, never pose) | {"pose_ref":"start"|"session_start"} | '
                  '{"move":{"direction":"+X|-X|+Y|-Y|+Z|-Z or a defined word","distance_m":d}} (or distance_mm; if the WORKSPACE room is too small the user is asked, add "stop_at_limit":true to stop at the edge instead; if the arm reach ends first the move is shortened automatically and marked ADJUSTED) | {"move":{"direction":"...","until_limit":true}} (go to the workspace edge in that direction) | '
                  '{"rotate":{"axis":"X"|"Y"|"Z","angle_deg":deg}} | {"until_limit":{"axis":"X"|"Y"|"Z","side":"min"|"max"}} '
                  '(until_limit moves only that axis to the workspace limit minus LIMIT_MARGIN_M, never onto the limit) | '
                  '{"to_limits":{"x":"max","y":"max","z":"min"}} (go to workspace limit positions on any of x/y/z; use it for "go to xmax ymax zmin", '
                  '"to the corner", "as far as possible"; the software ADJUSTS to what the arm can comfortably reach and says so); '
                  'a target the arm cannot comfortably reach (pose, pose_ref, delta_xyz_m, move) is ADJUSTED automatically (height first, then pulled back along the path) and the plan says so; no flag is needed and the user is never asked about it; '
                  'optional "speed_m_s","acceleration_m_s2".',
        "units": "metres, radians (rotation vector), degrees ONLY inside rotate.angle_deg, m/s, m/s^2.",
        "template": "movel(p[X, Y, Z, RX, RY, RZ], a=A, v=V)",
        "example": "movel(p[0.4, -0.1, 0.25, 1.049, 0.509, 1.623], a=0.05, v=0.02)",
        "validation": "speed/accel caps; whole straight path sampled inside the X/Y/Z workspace; rotation vector <= 2*pi.",
        "limits": "rotate keeps the TCP position and uses the controller's own inverse kinematics; no joint angles are guessed. "
                  "The arm's reach is checked (not above 90% of its stretch, no contact with its own base, wrist or the table); "
                  "which points are reachable depends on distance and direction, so far/high targets are ADJUSTED automatically (never to the maximum).",
    },
    "movej": {
        "what": "Joint-space move.",
        "when": "Explicit joint angles from the user, OR turning the BASE (joint 1) by an angle, e.g. 'turn the base 180 degrees'. Only as the first motion of a plan.",
        "params": '{"joints_rad":[q1..q6]} OR {"base_rotate":{"angle_deg":180,"via":"+Y"|"-Y"}} (only joint 1 turns; via = the side the arm swings through, never over +Z; '
                  'the software computes the joints and picks a joint speed that keeps the TCP under the TCP speed limit); optional "speed_rad_s","acceleration_rad_s2".',
        "units": "radians, rad/s, rad/s^2.",
        "template": "movej([Q1, Q2, Q3, Q4, Q5, Q6], a=A, v=V)",
        "example": "movej([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], a=0.3, v=0.2)",
        "validation": "joint bounds; TCP path from UR5e forward kinematics inside the workspace; TCP speed/accel estimate.",
        "limits": "rejected after any movel/movec (start joints unknown). Never invent joint angles for Cartesian goals; a base turn uses base_rotate.",
    },
    "movec": {
        "what": "Circular TCP arc from the current pose through a via point to an end point.",
        "when": "ANY circle, arc, semicircle or curved path. Never approximate an arc with many movel.",
        "params": '{"arc":{"plane":"XY|XZ|YZ","center_dir":"+X","turn_toward":"+Z","radius_m":r,"sweep_deg":180}} '
                  'OR {"via_pose":[6],"to_pose":[6]}; optional "speed_m_s","acceleration_m_s2","blend_m" (see limits). '
                  "The arc STARTS at the current TCP; the circle centre is radius_m from the start along center_dir; "
                  "the path first moves toward turn_toward; end = start + 2*r*center_dir for a semicircle. Orientation is kept.",
        "units": "metres, degrees (sweep_deg only), m/s, m/s^2.",
        "template": "movec(p[X1, Y1, Z1, RX, RY, RZ], p[X2, Y2, Z2, RX, RY, RZ], a=A, v=V, r=R, mode=0)",
        "example": "movec(p[0.4, 0.0, 0.3, 1.0, 0.5, 1.6], p[0.45, 0.0, 0.25, 1.0, 0.5, 1.6], a=0.05, v=0.02, r=0.0, mode=0)",
        "validation": "circle must be definable (points not collinear); radius within limits; the WHOLE arc sampled inside the workspace.",
        "limits": "mode is always 0. Optional blend_m (0..0.05 m, at most 25 % of the shortest waypoint spacing): use it on the "
                  "FIRST of two movec in a row, because two back-to-back movec with r=0 fail on the controller (C171A1).",
    },
    "home": {
        "what": "Move to the configured HOME pose.",
        "when": "The user says home / go home / return to home.",
        "params": "{} (no parameters).",
        "units": "-",
        "template": "movel(p[HOME_X, HOME_Y, HOME_Z, HOME_RX, HOME_RY, HOME_RZ], a=HOME_A, v=HOME_V)",
        "example": "(rendered from HOME_POSE in limits.py)",
        "validation": "same checks as movel (home must be inside the workspace).",
        "limits": "uses the configured home speed/acceleration.",
    },
    "repeat": {
        "what": "Repeat a block of steps N times.",
        "when": "The user asks to repeat something N times.",
        "params": '{"count": N, "body": [step, step, ...]} (body steps cannot contain another repeat).',
        "units": "count is an integer.",
        "template": "(no code of its own: software expands it into the body's lines, one validated copy per iteration)",
        "example": "repeat 5 x [movel +10 mm X]",
        "validation": f"count <= MAX_REPEAT_COUNT; expanded program <= MAX_PROGRAM_LINES; every iteration validated with the pose accumulated so far.",
        "limits": "Executed as a flat validated program (no URScript while-loop) so each iteration is checked.",
    },
    "gripper": {
        "what": "OnRobot RG gripper move.",
        "when": "Open/close/grip.",
        "params": '{"width_mm": w, "force_n": f}',
        "units": "millimetres, newtons.",
        "template": "rg_grip(WIDTH_MM, FORCE_N)",
        "example": "rg_grip(106.0, 20.0)",
        "validation": "width within 0..GRIPPER_WIDTH_MAX_EFFECTIVE_MM (106 mm), force within the configured range; gripper must be enabled.",
        "limits": "sent through the OnRobot XML-RPC server (return code checked, fixed wait after). Effective width max is 106 mm.",
    },
    "sleep": {
        "what": "Wait.",
        "when": "The user asks to wait/hold/pause.",
        "params": '{"seconds": s}',
        "units": "seconds.",
        "template": "sleep(SECONDS)",
        "example": "sleep(2.0)",
        "validation": "0 < seconds <= MAX_SLEEP_SECONDS.",
        "limits": "-",
    },
}
CAPABILITIES.update({
    "polygon": {
        "what": "Regular polygon (n equal sides) through the CURRENT TCP point, closing exactly on it.",
        "when": "Triangle, square, pentagon, hexagon, any regular n-gon. Use this, never hand-computed corner coordinates.",
        "params": '{"polygon":{"sides":n (3..12),"side_m":s,"plane":"XY|XZ|YZ","start_direction":"+X" (in plane),'
                  '"turn":"ccw|cw"}}; optional "speed_m_s","acceleration_m_s2". Each side is one movel line.',
        "units": "metres, m/s, m/s^2.",
        "template": "(software expands it into n movel lines, one per side)",
        "example": '{"polygon":{"sides":5,"side_m":0.09,"plane":"XY","start_direction":"-Y","turn":"ccw"}}',
        "validation": "every side validated like movel (workspace path, speed, accel); program-size limit.",
        "limits": "the polygon starts where the TCP is when this step runs; cannot be placed inside a repeat.",
    },
    "circle": {
        "what": "Full circle through the CURRENT TCP point, built as two 180-degree arcs.",
        "when": "A full circle or ring. For a semicircle or a single arc use movec.",
        "params": '{"circle":{"plane":"XY|XZ|YZ","center_dir":"+X","turn_toward":"+Z","radius_m":r}}; '
                  "optional speed/acceleration. The centre lies radius_m from the current point along center_dir.",
        "units": "metres, m/s, m/s^2.",
        "template": "(software expands it into two movec lines)",
        "example": '{"circle":{"plane":"XZ","center_dir":"+X","turn_toward":"+Z","radius_m":0.025}}',
        "validation": "both arcs validated like movec (radius range, arc inside workspace).",
        "limits": "cannot be placed inside a repeat.",
    },
})
NOT_IMPLEMENTED = ("loop (use the repeat action instead)", "movep", "distance", "inverse-kinematics solver for joint goals",
                   "any other URScript (no variables, no while/for, no I/O, no sockets)")


def exposed_actions():
    return [a for a in list(L.ALLOWED_ACTIONS) + list(GEOMETRY_ACTIONS) if a in CAPABILITIES]


def check_capabilities():
    bad = [a for a in L.ALLOWED_ACTIONS if a not in CAPABILITIES]
    return [f"ALLOWED_ACTIONS lists '{a}' but robot_sender does not implement it." for a in bad]


# ----------------------------------------------------------------------------------------------
# Deterministic validator
# ----------------------------------------------------------------------------------------------
_NUM = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
_VEC6 = rf"({_NUM}(?:\s*,\s*{_NUM}){{5}})"
RE_MOVEL = re.compile(rf"^movel\(\s*p\[\s*{_VEC6}\s*\]\s*,\s*a\s*=\s*({_NUM})\s*,\s*v\s*=\s*({_NUM})\s*\)$")
RE_MOVEJ = re.compile(rf"^movej\(\s*\[\s*{_VEC6}\s*\]\s*,\s*a\s*=\s*({_NUM})\s*,\s*v\s*=\s*({_NUM})\s*\)$")
RE_MOVEC = re.compile(rf"^movec\(\s*p\[\s*{_VEC6}\s*\]\s*,\s*p\[\s*{_VEC6}\s*\]\s*,\s*a\s*=\s*({_NUM})\s*,\s*v\s*=\s*({_NUM})"
                      rf"\s*,\s*r\s*=\s*({_NUM})\s*,\s*mode\s*=\s*(\d+)\s*\)$")
RE_GRIP = re.compile(rf"^rg_grip\(\s*({_NUM})\s*,\s*({_NUM})\s*\)$")
RE_SLEEP = re.compile(rf"^sleep\(\s*({_NUM})\s*\)$")


class Finding:
    """One validation problem. kind: 'fatal' (environment/config, no repair possible), 'global' (whole plan),
    'plan' (a plan step's values are wrong), 'code' (the step's code does not match / is not allowed)."""

    def __init__(self, message, step=None, line=None, kind="plan", param=None, limit=None,
                 instruction=None, step_name=None, src=None):
        self.message, self.step, self.line, self.kind = message, step, line, kind
        self.param, self.limit, self.instruction, self.step_name, self.src = param, limit, instruction, step_name, src

    @staticmethod
    def src_text(src):
        return None if not src else ".".join(str(x) for x in src)

    def __str__(self):
        loc = (f"[step {self.step}] " if self.step else "") + (f"[line {self.line}] " if self.line else "")
        return loc + self.message

    def detailed(self):
        out = ["VALIDATION ERROR"]
        if self.step or self.src:
            where = str(self.step) if self.step else "-"
            if self.src:
                where += f"   (plan step {self.src_text(self.src)})"
            out.append(f"  Step:        {where}")
        if self.step_name:
            out.append(f"  Name:        {self.step_name}")
        if self.instruction:
            out.append(f"  Instruction: {self.instruction}")
        if self.param:
            out.append(f"  Parameter:   {self.param}")
        out.append(f"  Problem:     {self.message}")
        if self.limit:
            out.append(f"  Limit:       {self.limit}")
        return "\n".join(out)


def parse_instruction(text):
    """One line -> (kind, dict), or raises ValueError. Whole-line strict grammar."""
    s = text.strip()
    if not s or not s.isascii() or not s.isprintable():
        raise ValueError("empty or non-ASCII/non-printable line")
    for name, rx in (("movel", RE_MOVEL), ("movej", RE_MOVEJ)):
        m = rx.match(s)
        if m:
            vals = [float(x) for x in m.group(1).split(",")]
            a, v = float(m.group(2)), float(m.group(3))
            if not all(map(math.isfinite, vals + [a, v])):
                raise ValueError("non-finite number")
            return name, {"vec": vals, "a": a, "v": v}
    m = RE_MOVEC.match(s)
    if m:
        via, to = [float(x) for x in m.group(1).split(",")], [float(x) for x in m.group(2).split(",")]
        a, v, r, mode = float(m.group(3)), float(m.group(4)), float(m.group(5)), int(m.group(6))
        if not all(map(math.isfinite, via + to + [a, v, r])):
            raise ValueError("non-finite number")
        if r < 0.0 or mode != 0:
            raise ValueError("movec needs r >= 0 and mode=0 (only that form is implemented)")
        return "movec", {"via": via, "to": to, "a": a, "v": v, "r": r}
    m = RE_GRIP.match(s)
    if m:
        w, f = float(m.group(1)), float(m.group(2))
        if not (math.isfinite(w) and math.isfinite(f)):
            raise ValueError("non-finite number")
        return "gripper", {"width": w, "force": f}
    m = RE_SLEEP.match(s)
    if m:
        x = float(m.group(1))
        if not math.isfinite(x):
            raise ValueError("non-finite number")
        return "sleep", {"seconds": x}
    raise ValueError("not an allowed instruction (allowed: movel, movej, movec, rg_grip, sleep with the exact argument forms)")


def _movel_time(dist, rot, v, a):
    def trap(d, vel, acc):
        if d <= 0:
            return 0.0
        return d / vel + vel / acc if d >= vel * vel / acc else 2 * math.sqrt(d / acc)
    return max(trap(dist, v, a), trap(rot, v, a))


class ValidationResult:
    def __init__(self):
        self.findings, self.body_lines, self.sha256 = [], [], ""
        self.estimated_seconds = 0.0
        self.final_pose = None           # expected TCP pose after the program (None if unknown)
        self.has_motion = False
        self.has_gripper = False         # True if the program contains any rg_grip() call
        self.final_q = None              # joints (rad) expected after the program, from the reach walk (None if unknown)
        self.final_pose_est = None       # like final_pose, but also after a joint move (nominal kinematics). Used ONLY by the
                                         # dry-run simulation, never to confirm completion on the real robot (2 mm tolerance)
        self.numeric_notes = []          # human-readable targets/limits evidence
        self.start_state_pose = None

    @property
    def ok(self):
        return not self.findings

    def errors(self):
        return [str(f) for f in self.findings]

    def summary(self):
        if self.ok:
            return "PASSED. " + " | ".join(self.numeric_notes)
        return "FAILED: " + " ; ".join(self.errors())


def _check_state_for_validation(state, need_q):
    f = []
    if state is None:
        return [Finding("No live robot state available; cannot validate motion.", kind="fatal")]
    if state.pose is None:
        f.append(Finding("TCP pose unavailable; cannot validate motion.", kind="fatal"))
    if need_q and state.q is None:
        f.append(Finding("Joint positions unavailable; cannot validate.", kind="fatal"))
    return f


def validate_body(lines, state, steps=None, lim=None):
    """Validate instruction lines against live (or simulated) state. `steps` (optional) = list of dicts
    {"id","action","params","line_count"} describing which lines belong to which plan step.
    Returns ValidationResult. Never executes anything."""
    lim = lim or L.get_limits()
    res = ValidationResult()
    lines = [ln.strip() for ln in lines if ln.strip()]
    res.body_lines = lines
    res.sha256 = hashlib.sha256("\n".join(lines).encode()).hexdigest()

    line_step = {}
    if steps:
        n = 0
        for st in steps:
            for _ in range(st["line_count"]):
                n += 1
                line_step[n] = st["id"]

    def add(msg, ln=None, kind="plan", param=None, limit=None):
        res.findings.append(Finding(msg, step=line_step.get(ln) if ln else None, line=ln, kind=kind, param=param,
                                    limit=limit, instruction=lines[ln - 1] if ln else None))

    motion_prog = any(ln.lstrip().startswith(("movel", "movej", "movec")) for ln in lines)
    for p in L.check_consistency(lim, include_home=motion_prog):   # gripper-only programs are not arm motion
        add("Configuration problem: " + p, kind="fatal")
    for p in check_capabilities():
        add("Configuration problem: " + p, kind="fatal")
    if not lines:
        add("Program is empty.", kind="global")
    if len(lines) > L.MAX_PROGRAM_LINES:
        add(f"Program has {len(lines)} lines; maximum is {L.MAX_PROGRAM_LINES}.", kind="global",
            limit=f"MAX_PROGRAM_LINES = {L.MAX_PROGRAM_LINES}")

    parsed = []
    for i, ln in enumerate(lines, 1):
        try:
            parsed.append((i,) + parse_instruction(ln))
        except ValueError as e:
            add(f"Rejected: {e}", i, kind="code")
    if res.findings:
        return res

    uses_joints = any(k == "movej" for _i, k, _d in parsed)
    sf = _check_state_for_validation(state, need_q=uses_joints)
    if sf:
        res.findings.extend(sf)
        return res

    pose = list(state.pose)
    q = list(state.q) if state.q is not None else None
    tool = None
    if q is not None:
        tool = tool_offset_from_state(q, pose)
        if math.sqrt(sum(t * t for t in tool)) > 0.6:
            tool = None
            if uses_joints:
                add("TCP offset derived from live state is implausibly large (>0.6 m); cannot validate movej.", kind="fatal")
    res.start_state_pose = list(pose)
    if motion_prog and not _in_box(pose, lim):
        add(f"Current TCP position ({pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.4f}) m is outside the workspace "
            f"{_box_text(lim)}; every motion is refused until the robot is inside it.", kind="fatal", param="current TCP",
            limit=_box_text(lim))
    q_known = q is not None
    orient_known = True                  # False after a joint move that could not be converted to a TCP orientation
    total_t = 0.0
    rc = _Reach(state)
    if motion_prog and L.REACH_CHECK_ENABLED:
        if rc.tool is None or rc.q is None:
            add("The arm's reach cannot be verified: the live joint positions or the tool offset are unavailable. Motion is "
                "refused (fail closed).", kind="fatal", param="reach")
        else:
            why0 = envelope_problem(rc.q, rc.tool)
            if why0:
                add(f"The robot is currently outside the comfortable reach envelope ({why0}). Move it by hand (jog) into the "
                    f"normal working area first; every motion is refused until then.", kind="fatal", param="current pose")
                rc.broken = True

    def reach_fail(i, what, why, t, at):
        where = "at the very start of the move" if t <= 0 else f"{t * 100:.0f}% of the way"
        add(f"{what} leaves the arm's comfortable reach {where}, near ({at[0]:.3f}, {at[1]:.3f}, {at[2]:.3f}) m: "
            f"{why}. This is NOT a workspace limit: the robot would have to stretch or fold too far, which can stop it with a "
            f"fault.", i, param="reach",
            limit=f"reach envelope: at most {L.REACH_COMFORT_FRACTION * 100:.0f}% of the maximum stretch, "
                  f"{L.SELF_COLLISION_MARGIN_M * 1000:.0f} mm clearance between arm parts")

    def check_va(i, name, d):
        if not 0 < d["v"] <= lim["max_speed"] + 1e-12:
            add(f"{name} speed v={d['v']} m/s must be in (0, {lim['max_speed']}].", i, param="v",
                limit=f"max_speed = {lim['max_speed']} m/s")
        if not 0 < d["a"] <= lim["max_accel"] + 1e-12:
            add(f"{name} acceleration a={d['a']} m/s^2 must be in (0, {lim['max_accel']}].", i, param="a",
                limit=f"max_accel = {lim['max_accel']} m/s^2")

    def keepout_fail(i, what, pt):
        add(f"{what}: the TCP path goes through the robot's own base column ({math.hypot(pt[0], pt[1]):.3f} m from the base "
            f"axis at ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f}) m, keep-out {L.BASE_KEEPOUT_RADIUS_M:.3f} m below "
            f"{L.BASE_KEEPOUT_HEIGHT_M:.2f} m).", i, param="reach",
            limit=f"base keep-out radius {L.BASE_KEEPOUT_RADIUS_M:.3f} m, height {L.BASE_KEEPOUT_HEIGHT_M:.2f} m")

    def box_violation(i, what, pt, param):
        axis_info = []
        for ax, idx in (("x", 0), ("y", 1), ("z", 2)):
            lo, hi = lim[f"{ax}_min"], lim[f"{ax}_max"]
            if pt[idx] < lo - 1e-9:
                axis_info.append(f"{ax}={pt[idx]:.4f} below {ax}_min={lo:.4f}")
            elif pt[idx] > hi + 1e-9:
                axis_info.append(f"{ax}={pt[idx]:.4f} exceeds {ax}_max={hi:.4f}")
        add(f"{what} leaves the workspace at ({pt[0]:.4f}, {pt[1]:.4f}, {pt[2]:.4f}) m: " + "; ".join(axis_info),
            i, param=param, limit=_box_text(lim))

    prev_movec = None
    for i, kind, d in parsed:
        if kind != "movec":
            prev_movec = None           # any other instruction is a stop between arcs: the chain is broken
        if kind in ("movel", "movec") and not orient_known:
            add("The TCP orientation after the joint move is unknown (no tool model), so this motion cannot be verified. "
                "Refused (fail closed).", i, param="orientation")
        if kind == "movel":
            tgt = d["vec"]
            check_va(i, "movel", d)
            if math.sqrt(sum(x * x for x in tgt[3:])) > 2 * math.pi + 1e-9:
                add("movel rotation vector magnitude exceeds 2*pi rad (degrees given instead of radians?).", i, param="rx,ry,rz",
                    limit="|rotvec| <= 2*pi rad")
            dist = _dist(pose, tgt)
            n = max(1, math.ceil(dist / L.PATH_SAMPLE_STEP_M))
            bad = None
            for k in range(n + 1):
                f = k / n
                pt = [pose[j] + (tgt[j] - pose[j]) * f for j in range(3)]
                if not _in_box(pt, lim):
                    bad = pt
                    break
            if bad:
                box_violation(i, "movel path", bad, "target x,y,z")
            ko = keepout_hit([[pose[j] + (tgt[j] - pose[j]) * k / n for j in range(3)] for k in range(n + 1)], pose)
            if ko:
                keepout_fail(i, "movel path", ko)
            if rc.on:
                ok_r, why_r, t_r = rc.advance(pose, tgt)
                if not ok_r:
                    reach_fail(i, "movel path", why_r, t_r, [pose[j] + (tgt[j] - pose[j]) * t_r for j in range(3)])
            rot = rotation_angle_between(pose[3:], tgt[3:]) if d["v"] > 0 and d["a"] > 0 else 0.0
            if d["v"] > 0 and d["a"] > 0:
                total_t += _movel_time(dist, rot, d["v"], d["a"])
            res.numeric_notes.append(f"line {i} movel target ({tgt[0]:.4f},{tgt[1]:.4f},{tgt[2]:.4f}) m, "
                                     f"v={d['v']} a={d['a']}, path {dist * 1000:.1f} mm sampled every "
                                     f"{L.PATH_SAMPLE_STEP_M * 1000:.0f} mm")
            pose = list(tgt)
            q_known = False
            res.has_motion = True
        elif kind == "movec":
            via, to = d["via"], d["to"]
            check_va(i, "movec", d)
            blend = d.get("r", 0.0)
            seg1 = _dist(pose[:3], via[:3])
            seg2 = _dist(via[:3], to[:3])
            if blend > BLEND_MAX_FRACTION_OF_SEGMENT * min(seg1, seg2) + 1e-12:
                add(f"movec blend radius {blend * 1000:.2f} mm is too large for waypoints {seg1 * 1000:.1f} mm and "
                    f"{seg2 * 1000:.1f} mm apart (controller error C171A0).", i, param="r",
                    limit=f"r <= {BLEND_MAX_FRACTION_OF_SEGMENT} x shortest waypoint spacing")
            if prev_movec is not None and prev_movec[1] == 0.0:
                add("movec directly after another movec with blend r=0 is refused: the controller reports C171A1 "
                    "'Blend radius too small in a MoveC'. Give the first movec a blend radius > 0, or put a stop between them.",
                    prev_movec[0], param="r", limit="first movec of two in a row needs r > 0")
            prev_movec = (i, blend)
            for nm, pz in (("via", via), ("to", to)):
                if math.sqrt(sum(x * x for x in pz[3:])) > 2 * math.pi + 1e-9:
                    add(f"movec {nm} rotation vector magnitude exceeds 2*pi rad.", i, param=f"{nm}.rx,ry,rz",
                        limit="|rotvec| <= 2*pi rad")
            g = circle_through(pose[:3], via[:3], to[:3])
            if g is None:
                add("movec start, via and end points are (nearly) collinear or too close together, so no circle is defined. "
                    "Use movel for a straight segment or choose a real arc.", i, param="via/to")
            else:
                r_ = g["radius"]
                if not L.MIN_ARC_RADIUS_M <= r_ <= L.MAX_ARC_RADIUS_M:
                    add(f"movec circle radius {r_ * 1000:.1f} mm is outside the allowed range.", i, param="radius",
                        limit=f"{L.MIN_ARC_RADIUS_M * 1000:.0f}..{L.MAX_ARC_RADIUS_M * 1000:.0f} mm")
                if math.degrees(g["sweep"]) > L.MAX_ARC_SWEEP_DEG:
                    add(f"movec arc sweep {math.degrees(g['sweep']):.1f} deg is too large.", i, param="sweep",
                        limit=f"MAX_ARC_SWEEP_DEG = {L.MAX_ARC_SWEEP_DEG}")
                bad = next((pt for pt in arc_samples(g, L.PATH_SAMPLE_STEP_M) if not _in_box(pt, lim)), None)
                if bad:
                    box_violation(i, "movec arc", bad, "arc")
                ko = keepout_hit(arc_samples(g, L.PATH_SAMPLE_STEP_M), pose)
                if ko:
                    keepout_fail(i, "movec arc", ko)
                if rc.on:
                    arc_pts = arc_samples(g, L.REACH_SAMPLE_STEP_M)
                    ok_r, why_r, t_r = rc.advance_points(arc_pts, pose[3:], to[3:])
                    if not ok_r:
                        reach_fail(i, "movec arc", why_r, t_r, arc_pts[min(len(arc_pts) - 1, int(t_r * (len(arc_pts) - 1)))])
                if d["v"] > 0 and d["a"] > 0:
                    total_t += _movel_time(g["length"], rotation_angle_between(pose[3:], to[3:]), d["v"], d["a"])
                res.numeric_notes.append(f"line {i} movec radius {r_ * 1000:.1f} mm, sweep {math.degrees(g['sweep']):.1f} deg, "
                                         f"arc {g['length'] * 1000:.1f} mm, end ({to[0]:.4f},{to[1]:.4f},{to[2]:.4f}) m, "
                                         f"v={d['v']} a={d['a']}")
            pose = list(to)
            q_known = False
            res.has_motion = True
        elif kind == "movej":
            tq = d["vec"]
            if not 0 < d["v"] <= L.MAX_JOINT_SPEED + 1e-12:
                add(f"movej speed v={d['v']} rad/s must be in (0, {L.MAX_JOINT_SPEED}].", i, param="v",
                    limit=f"MAX_JOINT_SPEED = {L.MAX_JOINT_SPEED} rad/s")
            if not 0 < d["a"] <= L.MAX_JOINT_ACCELERATION + 1e-12:
                add(f"movej acceleration a={d['a']} rad/s^2 must be in (0, {L.MAX_JOINT_ACCELERATION}].", i, param="a",
                    limit=f"MAX_JOINT_ACCELERATION = {L.MAX_JOINT_ACCELERATION} rad/s^2")
            for j, (lo, hi) in enumerate(L.JOINT_LIMITS_RAD):
                if not lo <= tq[j] <= hi:
                    add(f"movej joint {j + 1} target {tq[j]:.4f} rad is outside [{lo}, {hi}] (degrees given instead of radians?).",
                        i, param=f"q{j + 1}", limit=f"[{lo}, {hi}] rad")
            if not q_known:
                add("movej start joint configuration is unknown (it follows a movel/movec); the workspace path cannot be "
                    "verified, so the move is rejected. Use movej only before any movel, or use movel.", i)
            elif tool is not None and d["v"] > 0 and d["a"] > 0:
                dq = max(abs(tq[j] - q[j]) for j in range(6))
                n = 200
                pts = []
                for k in range(n + 1):
                    f = k / n
                    qq = [q[j] + (tq[j] - q[j]) * f for j in range(6)]
                    pts.append(tcp_position_from_joints(qq, tool))
                for pt in pts:
                    if not _in_box(pt, lim):
                        box_violation(i, "movej path (UR5e kinematics, TCP only)", pt, "joints")
                        break
                ko = keepout_hit(pts, pts[0])
                if ko:
                    keepout_fail(i, "movej path", ko)
                if rc.tool is not None:
                    for k in range(0, n + 1, 4):
                        f = k / n
                        qq = [q[j] + (tq[j] - q[j]) * f for j in range(6)]
                        why_j = envelope_problem(qq, rc.tool)
                        if why_j:
                            reach_fail(i, "movej path", why_j, f, pts[k])
                            break
                if dq > 1e-9:
                    vpk = min(d["v"], math.sqrt(d["a"] * dq))
                    ds = 1.0 / n
                    v_tcp = max(_dist(pts[k], pts[k + 1]) / ds for k in range(n)) * vpk / dq
                    a_tcp = 0.0
                    for k in range(1, n):
                        sec = [pts[k + 1][j] - 2 * pts[k][j] + pts[k - 1][j] for j in range(3)]
                        a_tcp = max(a_tcp, math.sqrt(sum(x * x for x in sec)) / ds ** 2)
                    a_peak = (max(_dist(pts[k], pts[k + 1]) / ds for k in range(n)) * d["a"] / dq
                              + a_tcp * (vpk / dq) ** 2)
                    if v_tcp > lim["max_speed"] + 1e-9:
                        add(f"movej would move the TCP at about {v_tcp:.3f} m/s; lower v.", i, param="v",
                            limit=f"max_speed = {lim['max_speed']} m/s")
                    if a_peak > lim["max_accel"] + 1e-9:
                        add(f"movej TCP acceleration estimated at {a_peak:.3f} m/s^2; lower a.", i, param="a",
                            limit=f"max_accel = {lim['max_accel']} m/s^2")
                    total_t += dq / vpk + vpk / d["a"]
                    res.numeric_notes.append(f"line {i} movej est. peak TCP speed {v_tcp:.3f} m/s, accel {a_peak:.3f} m/s^2")
                tm_v = tool_from_state(state)
                if tm_v is not None:                # the real orientation after the joint move: R_flange(q) * R_tool
                    pose = pts[-1] + matrix_to_rotvec(_mm3(_flange(tq)[0], tm_v.r_tool))
                else:                               # no tool model: later TCP motion is refused (fail closed)
                    pose = pts[-1] + pose[3:]
                    orient_known = False
                q = list(tq)
                if rc.tool is not None:
                    rc.q = list(tq)
                res.has_motion = True
                res.final_pose = None
                continue
            q = list(tq)
            res.has_motion = True
        elif kind == "gripper":
            if not L.GRIPPER_ENABLED:
                add("Gripper is disabled (GRIPPER_ENABLED = False in limits.py).", i, kind="fatal")
            if not L.GRIPPER_WIDTH_MIN_MM <= d["width"] <= L.GRIPPER_WIDTH_MAX_EFFECTIVE_MM:
                add(f"rg_grip width {d['width']} mm outside the gripper range (the URCap clamps to "
                    f"{L.GRIPPER_WIDTH_MAX_EFFECTIVE_MM:g} mm with the {L.GRIPPER_FINGERTIP_OFFSET_MM:g} mm fingertip offset).",
                    i, param="width_mm", limit=f"[{L.GRIPPER_WIDTH_MIN_MM}, {L.GRIPPER_WIDTH_MAX_EFFECTIVE_MM:g}] mm")
            if not L.GRIPPER_FORCE_MIN_N <= d["force"] <= L.GRIPPER_FORCE_MAX_N:
                add(f"rg_grip force {d['force']} N outside the gripper range.", i, param="force_n",
                    limit=f"[{L.GRIPPER_FORCE_MIN_N}, {L.GRIPPER_FORCE_MAX_N}] N")
            total_t += L.GRIPPER_MOVE_ESTIMATE_SECONDS
            res.has_gripper = True
            res.numeric_notes.append(f"line {i} rg_grip {d['width']} mm / {d['force']} N")
        elif kind == "sleep":
            if not 0 < d["seconds"] <= L.MAX_SLEEP_SECONDS:
                add(f"sleep({d['seconds']}) is out of range.", i, param="seconds", limit=f"(0, {L.MAX_SLEEP_SECONDS}] s")
            else:
                total_t += d["seconds"]
    res.estimated_seconds = total_t
    res.final_q = list(rc.q) if (rc.tool is not None and rc.q is not None and not rc.broken) else None
    last_motion = [k for _i, k, _d in parsed if k in ("movel", "movej", "movec")]
    if last_motion and last_motion[-1] in ("movel", "movec"):
        res.final_pose = list(pose)
    if last_motion and orient_known:
        res.final_pose_est = list(pose)
    res.numeric_notes.insert(0, f"workspace {_box_text(lim)}, max v {lim['max_speed']} m/s, max a {lim['max_accel']} m/s^2")
    return res


# ----------------------------------------------------------------------------------------------
# Plan resolution: planner JSON -> exact numeric steps (all arithmetic done here, never by the LLM)
# ----------------------------------------------------------------------------------------------
def _num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _vec(x, n):
    return isinstance(x, list) and len(x) == n and all(_num(v) for v in x)


def resolve_plan(plan, state, lim=None, refs=None):
    """Fill defaults, expand `repeat`, turn delta/pose_ref/rotate/arc into absolute numbers.
    Returns (resolved_steps, findings). Every finding carries `src` = path of the offending PLAN step
    (e.g. (3,) or (2, 1) inside a repeat) so repairs can touch exactly that step."""
    refs = refs or {}
    findings, flat = [], []
    raw = plan.get("steps")
    if not isinstance(raw, list) or not raw:
        return [], [Finding("Plan has no steps.", kind="global")]
    if len(raw) > L.MAX_PLAN_STEPS:
        findings.append(Finding(f"Plan has {len(raw)} steps; maximum is {L.MAX_PLAN_STEPS}.", kind="global",
                                limit=f"MAX_PLAN_STEPS = {L.MAX_PLAN_STEPS}"))
        raw = raw[:L.MAX_PLAN_STEPS]
    avail = exposed_actions()

    def bad(msg, path, name=None, param=None, limit=None):
        findings.append(Finding(msg, kind="plan", param=param, limit=limit, src=tuple(path), step_name=name))

    for i, s in enumerate(raw, 1):
        if not isinstance(s, dict):
            bad("Step is not an object.", (i,))
            continue
        if s.get("action") == "repeat":
            p = s.get("params") if isinstance(s.get("params"), dict) else {}
            cnt, body = p.get("count"), p.get("body")
            nm = str(s.get("name", f"Step {i}"))[:80]
            if "repeat" not in avail:
                bad("repeat is not enabled.", (i,), nm, "action")
            elif not (isinstance(cnt, int) and not isinstance(cnt, bool) and 1 <= cnt <= L.MAX_REPEAT_COUNT):
                bad("repeat.count must be an integer.", (i,), nm, "count", f"1..{L.MAX_REPEAT_COUNT}")
            elif not isinstance(body, list) or not body or not all(isinstance(b, dict) for b in body):
                bad("repeat.body must be a non-empty list of steps.", (i,), nm, "body")
            elif any(b.get("action") == "repeat" for b in body):
                bad("Nested repeat is not supported.", (i,), nm, "body")
            elif any(b.get("action") in GEOMETRY_ACTIONS for b in body):
                bad("polygon / circle cannot be inside a repeat; put them at the top level.", (i,), nm, "body")
            else:
                for k in range(1, cnt + 1):
                    for j, b in enumerate(body, 1):
                        flat.append(((i, j), b, (k, cnt, nm, "repeat")))
        else:
            flat.append(((i,), s, None))
    if len(flat) > L.MAX_PROGRAM_LINES:
        findings.append(Finding(f"The plan expands to {len(flat)} steps; maximum program size is {L.MAX_PROGRAM_LINES} lines.",
                                kind="global", limit=f"MAX_PROGRAM_LINES = {L.MAX_PROGRAM_LINES}"))
        flat = flat[:L.MAX_PROGRAM_LINES]

    cur = list(state.pose) if state is not None and state.pose is not None else None
    q = list(state.q) if state is not None and state.q is not None else None
    tool = tool_offset_from_state(q, cur) if (q and cur) else None
    rc = _Reach(state)
    steps = []
    reported = set()
    pending = deque(flat)
    while pending:
        path, s, rep = pending.popleft()
        act, params = s.get("action"), s.get("params")
        name = str(s.get("name", f"Step {path[0]}"))[:80]
        desc = str(s.get("description", ""))[:400]
        note = None
        adjusted = False

        def fail(msg, param=None, limit=None, param_override=None):
            if path not in reported:           # one report per plan step, not per repeat iteration
                reported.add(path)
                bad(msg, path, name, param_override or param, limit)
        if act not in avail:
            fail(f"Action '{act}' is not implemented. Available: {', '.join(avail)}. "
                 f"Not implemented: {'; '.join(NOT_IMPLEMENTED)}.", "action")
            continue
        if act == "home":
            params = {}
        if not isinstance(params, dict):
            fail("params must be an object.", "params")
            continue
        if act in GEOMETRY_ACTIONS:
            if cur is None:
                fail(f"{act} needs the current TCP pose, which is unavailable.", "params")
                continue
            sp_g = params.get("speed_m_s", L.DEFAULT_TCP_SPEED)
            ac_g = params.get("acceleration_m_s2", L.DEFAULT_TCP_ACCELERATION)
            if not (_num(sp_g) and _num(ac_g)):
                fail("speed_m_s / acceleration_m_s2 must be numbers.", "speed_m_s")
                continue
            if act == "polygon":
                kids, err = expand_polygon(cur, params.get("polygon"), sp_g, ac_g)
            else:
                kids, err = expand_circle(cur, params.get("circle"), sp_g, ac_g)
            if err:
                fail(err, act)
                continue
            if len(steps) + len(pending) + len(kids) > L.MAX_PROGRAM_LINES:
                fail(f"{act} would make the program longer than {L.MAX_PROGRAM_LINES} lines.", act,
                     f"MAX_PROGRAM_LINES = {L.MAX_PROGRAM_LINES}")
                continue
            label = "side" if act == "polygon" else "arc"
            for kk, kid in reversed(list(enumerate(kids, 1))):
                pending.appendleft(((path[0],), kid, (kk, len(kids), None, label)))
            continue
        rp = {}
        out_action = act
        if act in ("movel", "movec", "home"):
            sp = params.get("speed_m_s", L.HOME_TCP_SPEED if act == "home" else L.DEFAULT_TCP_SPEED)
            ac = params.get("acceleration_m_s2", L.HOME_TCP_ACCELERATION if act == "home" else L.DEFAULT_TCP_ACCELERATION)
            if not (_num(sp) and _num(ac)):
                fail("speed_m_s / acceleration_m_s2 must be numbers.", "speed_m_s")
                continue
        if act == "home":
            out_action = "movel"
            new = list(L.HOME_POSE)
            rp = {"pose": [round(x, 9) for x in new], "speed_m_s": sp, "acceleration_m_s2": ac}
            if cur is not None:
                rc.advance(cur, new)
            cur, q = new, None
        elif act == "movel":
            modes = [k for k in ("delta_xyz_m", "pose", "xyz_m", "pose_ref", "rotate", "until_limit", "move", "to_limits") if k in params]
            if len(modes) != 1:
                fail("movel needs EXACTLY ONE of delta_xyz_m, pose, xyz_m, pose_ref, rotate, until_limit, move, to_limits.", "params")
                continue
            m = modes[0]
            if m == "delta_xyz_m":
                dl = params["delta_xyz_m"]
                if not _vec(dl, 3):
                    fail("delta_xyz_m must be three finite numbers (metres).", "delta_xyz_m")
                    continue
                if max(abs(x) for x in dl) > 1.0:
                    fail("delta_xyz_m larger than 1 m (millimetres given as metres?).", "delta_xyz_m")
                    continue
                if cur is None:
                    fail("delta_xyz_m needs the current TCP pose, which is unavailable.", "delta_xyz_m")
                    continue
                new = [cur[0] + dl[0], cur[1] + dl[1], cur[2] + dl[2]] + cur[3:]
                if rc.on:
                    new, info = _adjust_target(rc, cur, new, L.get_limits())
                    if new is None:
                        fail("the arm cannot comfortably move toward that point at all from here. Ask the user: choose a nearer \
or lower point, or cancel.", "delta_xyz_m", param_override="fit")
                        continue
                    if info:
                        note, adjusted = adjust_note(info), True
            elif m == "pose":
                if not _vec(params["pose"], 6):
                    fail("pose must be six finite numbers.", "pose")
                    continue
                new = list(params["pose"])
                if rc.on and cur is not None and not (rep and rep[3] == "side"):   # polygon sides are never adjusted
                    new, info = _adjust_target(rc, cur, new, L.get_limits())
                    if new is None:
                        fail("the arm cannot comfortably move toward that pose at all from here. Ask the user: choose a nearer "
                             "or lower point, or cancel.", "pose", param_override="fit")
                        continue
                    if info:
                        note, adjusted = adjust_note(info), True
            elif m == "xyz_m":
                xyz = params["xyz_m"]
                if not _vec(xyz, 3):
                    fail("xyz_m must be three finite numbers (metres).", "xyz_m")
                    continue
                if cur is None:
                    fail("xyz_m needs the current TCP pose (position and orientation), which is unavailable.", "xyz_m")
                    continue
                new = [float(c) for c in xyz] + list(cur[3:])          # absolute position, current tracked orientation
                if rc.on:
                    new, info = _adjust_target(rc, cur, new, L.get_limits())
                    if new is None:
                        fail("the arm cannot comfortably move toward that point at all from here. Ask the user: choose a nearer "
                             "or lower point, or cancel.", "xyz_m", param_override="fit")
                        continue
                    if info:
                        note, adjusted = adjust_note(info), True
            elif m == "pose_ref":
                ref = refs.get(params["pose_ref"])
                if ref is None:
                    fail(f"pose_ref '{params['pose_ref']}' is not available (use 'start' or 'session_start').", "pose_ref")
                    continue
                new = list(ref)
                if rc.on and cur is not None:       # a saved pose is returned to EXACTLY or not at all (never adjusted)
                    ok_ref, _qe, why_ref, _t = reach_straight(rc.tool, rc.q, cur[:3], rotvec_to_matrix(cur[3:]),
                                                              new[:3], rotvec_to_matrix(new[3:]))
                    if not ok_ref:
                        fail(f"the straight way back to the saved pose is not inside the arm's reach envelope ({why_ref}). "
                             "Ask the user: rotate the base back (base_rotate with the same side) or choose another target, "
                             "or cancel.", "pose_ref", param_override="fit")
                        continue
            elif m == "until_limit":
                ul = params["until_limit"]
                if not (isinstance(ul, dict) and ul.get("axis") in ("X", "Y", "Z") and ul.get("side") in ("min", "max")):
                    fail('until_limit must be {"axis":"X|Y|Z","side":"min|max"}.', "until_limit")
                    continue
                if cur is None:
                    fail("until_limit needs the current TCP pose, which is unavailable.", "until_limit")
                    continue
                dv_ul = [0.0, 0.0, 0.0]
                dv_ul["xyz".index(ul["axis"].lower())] = 1.0 if ul["side"] == "max" else -1.0
                det = room_detail(cur, dv_ul, rc)
                if det["room"] <= 1e-9:
                    fail(f"until_limit: already at {edge_name(det)} in that "
                         f"direction; nothing to move. Ask the user: choose another direction or cancel.", "until_limit",
                         param_override="fit")
                    continue
                new = [cur[i] + dv_ul[i] * det["room"] for i in range(3)] + list(cur[3:])
                if det["by"] == "reach":
                    note, adjusted = reach_adjust_text(
                        f"{ul['axis']} {ul['side']} limit ({det['box'] * 100:.1f} cm of room in the workspace)",
                        f"stopped after {det['room'] * 100:.1f} cm", det["why"], _box_text(L.get_limits())), True
            elif m == "move":
                mv = params["move"]
                if not isinstance(mv, dict):
                    fail('move must be {"direction": "+X|...", "distance_m": d}.', "move")
                    continue
                dvec = direction_vector(mv.get("direction"))
                if dvec is None:
                    fail(f"direction '{mv.get('direction')}' is not defined. Use +X/-X/+Y/-Y/+Z/-Z, or define the word in "
                         "DIRECTION_CONVENTION in limits.py. Ask the user instead of guessing.", "direction")
                    continue
                until_flag = mv.get("until_limit") is True
                stop_flag = mv.get("stop_at_limit") is True
                dk = [k for k in ("distance_m", "distance_mm") if k in mv]
                if until_flag:
                    if dk:
                        fail("move: use until_limit OR a distance, not both.", "distance")
                        continue
                    dist_m = None
                else:
                    if len(dk) != 1:
                        fail("move needs exactly one of distance_m or distance_mm (or until_limit: true).", "distance")
                        continue
                    dist = mv[dk[0]]
                    if not _num(dist) or dist <= 0:
                        fail("distance must be a positive number.", "distance")
                        continue
                    dist_m = dist if dk[0] == "distance_m" else dist / 1000.0
                    if dist_m > 1.0:
                        fail("move distance larger than 1 m (millimetres given as metres?).", "distance")
                        continue
                if cur is None:
                    fail("move needs the current TCP pose, which is unavailable.", "move")
                    continue
                det = room_detail(cur, dvec, rc)
                room = det["room"]
                if room is None:
                    fail("move with a distance or until_limit needs a single base axis (+X/-X/+Y/-Y/+Z/-Z).", "direction")
                    continue
                edge_word = edge_name(det)
                if until_flag:
                    if room <= 0:
                        fail(f"already at or beyond the limit in that direction (room {room * 1000:.1f} mm): nothing to move. "
                             f"Ask the user: choose another direction or cancel.", "distance", param_override="fit")
                        continue
                    dist_m = room
                    if det["by"] == "reach":
                        note, adjusted = reach_adjust_text("until the limit", f"stopped after {room * 100:.1f} cm", det["why"],
                                                           _box_text(L.get_limits())), True
                    else:
                        note = f"until the limit: {room * 100:.1f} cm"
                elif dist_m > room + 1e-9:
                    if (stop_flag or det["by"] == "reach") and room > 0:
                        if det["by"] == "reach":
                            note, adjusted = reach_adjust_text(f"requested {dist_m * 100:.1f} cm", f"stopped after {room * 100:.1f} cm",
                                                               det["why"], _box_text(L.get_limits())), True
                        else:
                            note = (f"requested {dist_m * 100:.1f} cm, stopped at the workspace limit after "
                                    f"{room * 100:.1f} cm")
                        dist_m = room
                    else:
                        extra = f" ({det['why']})" if det["by"] == "reach" else ""
                        fail(f"does not fit: {dist_m * 100:.1f} cm requested in that direction, only "
                             f"{max(room, 0.0) * 100:.1f} cm available before {edge_word}{extra}. Ask the user: shorten "
                             f"to the available distance (stop_at_limit) or cancel.", "distance", param_override="fit")
                        continue
                new = [cur[i] + dvec[i] * dist_m for i in range(3)] + cur[3:]
            elif m == "to_limits":
                tl = params["to_limits"]
                if not (isinstance(tl, dict) and tl and all(k in ("x", "y", "z") and v in ("min", "max") for k, v in tl.items())):
                    fail('to_limits must look like {"x":"max","y":"max","z":"min"} (any of x, y, z; each "min" or "max").', "to_limits")
                    continue
                if cur is None:
                    fail("to_limits needs the current TCP pose, which is unavailable.", "to_limits")
                    continue
                lim_now = L.get_limits()
                new = list(cur)
                for k, side in tl.items():
                    new[("x", "y", "z").index(k)] = lim_now[f"{k}_{side}"] + (L.LIMIT_MARGIN_M if side == "min" else -L.LIMIT_MARGIN_M)
                if max(abs(new[i] - cur[i]) for i in range(3)) < 0.001:
                    fail("already at that limit position; nothing to move. Ask the user: choose another position or cancel.",
                         "to_limits", param_override="fit")
                    continue
                want = list(new)
                if rc.on:
                    new, info = _adjust_target(rc, cur, new, lim_now)
                    if new is None:
                        fail("the arm cannot comfortably move toward that limit position at all from here "
                             f"({info['why'] if info else 'reach'}). Ask the user: choose another position or cancel.",
                             "to_limits", param_override="fit")
                        continue
                    if info:
                        note, adjusted = adjust_note(info), True
                    else:
                        note = f"to the workspace limit position ({want[0]:.3f}, {want[1]:.3f}, {want[2]:.3f}) m, {L.LIMIT_MARGIN_M * 1000:.0f} mm inside the limits"
                else:
                    note = "to the workspace limit position (the arm's reach was NOT checked)"
            else:
                r = params["rotate"]
                if not (isinstance(r, dict) and r.get("axis") in ("X", "Y", "Z") and _num(r.get("angle_deg"))):
                    fail('rotate must be {"axis":"X|Y|Z","angle_deg":number}.', "rotate")
                    continue
                if cur is None:
                    fail("rotate needs the current TCP pose, which is unavailable.", "rotate")
                    continue
                new = cur[:3] + rotate_about_base_axis(cur[3:], r["axis"], math.radians(r["angle_deg"]))
            rp = {"pose": [round(x, 9) for x in new], "speed_m_s": sp, "acceleration_m_s2": ac}
            if cur is not None:
                rc.advance(cur, new)
            cur, q = new, None
        elif act == "movec":
            if "arc" in params:
                if cur is None:
                    fail("arc needs the current TCP pose, which is unavailable.", "arc")
                    continue
                try:
                    via, to = build_arc_points(cur, params["arc"])
                except (ValueError, TypeError) as e:
                    fail(f"Invalid arc: {e}", "arc")
                    continue
            elif "via_pose" in params and "to_pose" in params:
                if not (_vec(params["via_pose"], 6) and _vec(params["to_pose"], 6)):
                    fail("via_pose and to_pose must be six finite numbers each.", "via_pose/to_pose")
                    continue
                via, to = [round(x, 6) for x in params["via_pose"]], [round(x, 6) for x in params["to_pose"]]
            else:
                fail("movec needs `arc` or both `via_pose` and `to_pose`.", "params")
                continue
            blend = params.get("blend_m", 0.0)
            if not (_num(blend) and 0.0 <= blend <= 0.05):
                fail("movec blend_m must be a number between 0 and 0.05 metres.", "blend_m")
                continue
            rp = {"via": via, "to": to, "speed_m_s": sp, "acceleration_m_s2": ac, "r": round(float(blend), 9)}
            if cur is not None and rc.on:
                g_arc = circle_through(cur[:3], via[:3], to[:3])
                if g_arc is not None:
                    rc.advance_points(arc_samples(g_arc, L.REACH_SAMPLE_STEP_M), cur[3:], to[3:])
            cur, q = list(to), None
        elif act == "movej":
            jr = params.get("joints_rad")
            sp = params.get("speed_rad_s", L.DEFAULT_JOINT_SPEED)
            ac = params.get("acceleration_rad_s2", L.DEFAULT_JOINT_ACCELERATION)
            if "base_rotate" in params:
                if "joints_rad" in params:
                    fail("movej: use joints_rad OR base_rotate, not both.", "base_rotate")
                    continue
                jr, br_text = base_rotate_joints(params["base_rotate"], q, tool)
                if jr is None:
                    fail(br_text, "base_rotate")
                    continue
                note = br_text
                if cur is not None:
                    # the TCP moves on a circle: pick joint speed/accel that keep it under the active TCP limits
                    radius = max(math.hypot(cur[0], cur[1]), 0.05)
                    lim_br = L.get_limits()
                    if "speed_rad_s" not in params:
                        sp = round(min(L.DEFAULT_JOINT_SPEED, 0.9 * lim_br["max_speed"] / radius), 4)
                    if "acceleration_rad_s2" not in params:
                        ac = round(min(L.DEFAULT_JOINT_ACCELERATION, 0.8 * lim_br["max_accel"] / radius), 4)
            if not _vec(jr, 6) or not (_num(sp) and _num(ac)):
                fail("movej needs joints_rad (six numbers, radians) or base_rotate, and numeric speed/acceleration.", "joints_rad")
                continue
            q = list(jr)
            tm_j = tool_from_state(state)       # orientation model (works even when the reach check is switched off)
            if tool is not None and tm_j is not None:
                cur = tcp_position_from_joints(q, tool) + matrix_to_rotvec(_mm3(_flange(q)[0], tm_j.r_tool))
            else:
                cur = None                      # unknown orientation: any later TCP step is refused (fail closed)
            rp = {"joints_rad": [round(x, 6) for x in q], "speed_rad_s": sp, "acceleration_rad_s2": ac}
            if rc.tool is not None:
                rc.q, rc.broken = list(q), False
        elif act == "gripper":
            if not (_num(params.get("width_mm")) and _num(params.get("force_n"))):
                fail("gripper needs numeric width_mm and force_n.", "width_mm/force_n")
                continue
            rp = {"width_mm": params["width_mm"], "force_n": params["force_n"]}
        else:
            if not _num(params.get("seconds")):
                fail("sleep needs numeric seconds.", "seconds")
                continue
            rp = {"seconds": params["seconds"]}
        if rep:
            name = f"{name} [{rep[3]} {rep[0]}/{rep[1]}]"
        steps.append({"id": len(steps) + 1, "src": path, "name": name, "description": desc, "action": out_action,
                      "plan_action": act, "params": rp, "repeat": rep, "note": note, "adjusted": adjusted})
    return steps, findings


def _fmt(x):
    # 9 decimals: poses restored from a saved start pose must match it exactly (6 decimals lost ~0.5 um)
    return repr(round(float(x), 9)) if abs(x) >= 1e-9 or x == 0 else repr(float(x))


def _p6(v):
    return "p[" + ", ".join(_fmt(x) for x in v) + "]"


def expected_line(step):
    """The one canonical line a step must produce. In CODEGEN_MODE 'template' this IS the step's code."""
    p, a = step["params"], step["action"]
    if a == "movel":
        return f"movel({_p6(p['pose'])}, a={_fmt(p['acceleration_m_s2'])}, v={_fmt(p['speed_m_s'])})"
    if a == "movec":
        return (f"movec({_p6(p['via'])}, {_p6(p['to'])}, a={_fmt(p['acceleration_m_s2'])}, "
                f"v={_fmt(p['speed_m_s'])}, r={_fmt(p.get('r', 0.0))}, mode=0)")
    if a == "movej":
        return f"movej([{', '.join(_fmt(x) for x in p['joints_rad'])}], a={_fmt(p['acceleration_rad_s2'])}, v={_fmt(p['speed_rad_s'])})"
    if a == "gripper":
        return f"rg_grip({_fmt(p['width_mm'])}, {_fmt(p['force_n'])})"
    return f"sleep({_fmt(p['seconds'])})"


def check_step_code_matches(step, code_line):
    """Code must be exactly one instruction of the step's action with the plan's numbers."""
    try:
        kind, d = parse_instruction(code_line)
    except ValueError as e:
        return [f"Code rejected: {e}"]
    exp_kind, exp = parse_instruction(expected_line(step))
    if kind != exp_kind:
        return [f"Step action is {step['action']} (expected {exp_kind}) but the code is {kind}."]
    errs = []
    keys = {"movel": ("vec", "a", "v"), "movej": ("vec", "a", "v"), "movec": ("via", "to", "a", "v", "r"),
            "gripper": ("width", "force"), "sleep": ("seconds",)}[kind]
    for k in keys:
        a_, b_ = d[k], exp[k]
        pairs = zip(a_, b_) if isinstance(a_, list) else [(a_, b_)]
        if any(abs(x - y) > 1e-6 for x, y in pairs):
            errs.append(f"Code value '{k}' does not match the plan step ({a_} vs {b_}).")
    return errs


def validate_candidate(steps, step_codes, state, plan_findings=None):
    """steps: resolved steps. step_codes: {step_id: code string}. Whole deterministic check of the FULL program.
    Returns (ValidationResult, step_line_meta)."""
    findings = list(plan_findings or [])
    lines, meta = [], []
    by_id = {st["id"]: st for st in steps}

    def deco(f, st, instr=None):
        f.step, f.step_name, f.src = st["id"], st["name"], st["src"]
        f.instruction = f.instruction or instr
        return f
    for st in steps:
        code = (step_codes.get(st["id"]) or "").strip()
        code_lines = [ln for ln in code.splitlines() if ln.strip() and not ln.strip().startswith("#")]
        if len(code_lines) != 1:
            findings.append(deco(Finding(f"Step must produce exactly one instruction, got {len(code_lines)}.", kind="code",
                                         param="code"), st, code[:120] or "(empty)"))
            meta.append({"id": st["id"], "action": st["action"], "params": st["params"], "line_count": 0})
            continue
        for e in check_step_code_matches(st, code_lines[0]):
            findings.append(deco(Finding(e, kind="code", param="code"), st, code_lines[0]))
        lines.append(code_lines[0])
        meta.append({"id": st["id"], "action": st["action"], "params": st["params"], "line_count": 1})
    res = validate_body(lines, state, steps=meta)
    for f in res.findings:
        if f.step in by_id:
            deco(f, by_id[f.step])
    res.findings = findings + res.findings
    return res, meta



# ----------------------------------------------------------------------------------------------
# Robot controller object
# ----------------------------------------------------------------------------------------------
# movec blending. A movec followed by another movec needs a blend radius > 0 on the FIRST one, otherwise the
# controller reports C171A1 "Blend radius too small in a MoveC". The blend must also stay well below the waypoint
# spacing (C171A0 "blend too large"). The circle uses a 1 mm blend ONLY at the joint between its two halves; its
# final point is a plain stop, so the return to the start point is exact.
CIRCLE_JOINT_BLEND_M = 0.001
BLEND_MAX_FRACTION_OF_SEGMENT = 0.25
GRIPPER_RPC_URL = "http://localhost:41414"
GRIPPER_WIDTH_READ_TIMEOUT_S = 5.0           # s to wait for the width readback after the timed gripper wait
GRIPPER_WIDTH_TOL_MM_DEFAULT = 2.0           # mm; overridden by GRIPPER_WIDTH_TOL_MM in limits.py if present   # OnRobot URCap XML-RPC server on the controller (the same server the URCap uses)


def _urscript_body(body_lines, readback=False):
    """Gripper lines become an XML-RPC call to the OnRobot URCap server. Plain `rg_grip(...)` cannot be used on the
    secondary port: URCap functions exist only in the program that the pendant loads (the 'boilerplate'), so the
    controller reports them as unknown. rpc_factory is a built-in URScript function, so no boilerplate is needed.
    The return code is checked; a fixed wait follows, because the XML-RPC call returns before the gripper has moved.
    readback=True (gripper-only programs): the measured width is read back and written to output float register 0,
    where the controller sends it to us over RTDE (output_double_register_0)."""
    out = []
    for ln in body_lines:
        m = RE_GRIP.match(ln.strip())
        if m:
            out.append(f"  grip_rc = rpc_grip.rg_grip(0, {m.group(1)} + 0.0, {m.group(2)} + 0.0)")
            out.append("  if grip_rc != 0:")
            out.append('    popup("OnRobot RG grip returned an error. Program halted.", title="AI gripper", error=True, blocking=False)')
            out.append("    halt")
            out.append("  end")
            out.append(f"  sleep({L.GRIPPER_MOVE_ESTIMATE_SECONDS})")
            if readback:
                out.append("  grip_w = rpc_grip.rg_get_width(0)")
                out.append("  write_output_float_register(0, grip_w)")
        else:
            out.append("  " + ln)
    return out


def build_urscript(body_lines, token, marker=True):
    """marker=True (arm motion present): the end-of-program register write is appended and RTDE confirms it.
    marker=False (gripper-only): no end marker; the measured width is read back instead (see execute)."""
    has_grip = any(RE_GRIP.match(ln.strip()) for ln in body_lines)
    decl = f'  rpc_grip = rpc_factory("xmlrpc", "{GRIPPER_RPC_URL}")\n' if has_grip else ""
    readback = has_grip and not marker
    if readback:
        decl += "  write_output_float_register(0, -1.0)\n"     # sentinel: -1.0 means 'no width read yet'
    body = "\n".join(_urscript_body(body_lines, readback=readback))
    tail = f"  write_output_integer_register(0, {int(token)})\n" if marker else ""
    return f"def ai_task():\n{decl}{body}\n{tail}end\nai_task()\n"


def last_grip_target(body_lines):
    """Width (mm) of the last rg_grip line, i.e. the value the gripper should end at."""
    for ln in reversed(body_lines):
        m = RE_GRIP.match(ln.strip())
        if m:
            return float(m.group(1))
    return None


def script_preview(body_lines, has_motion, has_gripper=False):
    """The exact URScript that execute() would send (token shown as TOKEN). For the approved-code file.
    Uses the same marker rule as Robot.execute(): no end marker only for gripper-only programs."""
    gripper_only = bool(has_gripper) and not has_motion
    script = build_urscript(body_lines, 0, marker=not gripper_only)   # build_urscript needs an int token
    return script.replace("write_output_integer_register(0, 0)", "write_output_integer_register(0, TOKEN)")


class ExecutionResult:
    def __init__(self, status, detail):
        self.status, self.detail = status, detail   # COMPLETED_CONFIRMED | COMPLETED_TIMED | SENT_NOT_CONFIRMED | STOPPED | REFUSED | FAILED | SIMULATED

    def __str__(self):
        return f"{self.status}: {self.detail}"


class Robot:
    """LIVE robot (dry_run=False) or clearly-labelled simulation (dry_run=True)."""

    def __init__(self, dry_run=False):
        self.dry_run = dry_run
        self.rtde = None if dry_run else RtdeReader()
        self.fault_latched = None        # reason string; blocks all movement until /clear-fault style reconcile
        self.stop_requested = threading.Event()
        self._busy = threading.Lock()    # only one program at a time
        self._token_counter = int(time.time()) % 1_000_000 + 1000
        self.session_start_pose = None   # TCP pose when this program first saw the robot ("session_start")
        self._sim = None
        if dry_run:
            q = list(L.SIM_START_JOINTS)
            self._sim = RobotState("SIMULATED", q=q, pose=None, tcp_speed=[0.0] * 6, qd=[0.0] * 6,
                                   robot_mode=7, safety_mode=1, runtime_state=1)
            self._sim.pose = list(L.HOME_POSE)       # SIM_START_JOINTS must be the joints the real robot has at HOME_POSE

    # --- connection and state ---
    def connect(self):
        if self.dry_run:
            return "SIMULATED robot (dry-run): no connection is made."
        self.rtde.connect()
        return f"RTDE live state connected to {L.ROBOT_IP}:{L.RTDE_PORT}."

    def capture_session_start(self):
        """Remember where the robot was when this application started (before any homing)."""
        st = self.get_state()
        if st is not None and st.pose is not None and self.session_start_pose is None:
            self.session_start_pose = list(st.pose)
        return self.session_start_pose

    def get_state(self, max_age=None):
        if self.dry_run:
            return self._sim
        return self.rtde.get(max_age)

    def dashboard_status(self):
        if self.dry_run:
            return {"note": "SIMULATED: dashboard not used in dry-run"}
        out = {}
        for key, cmd in (("robotmode", "robotmode"), ("safetystatus", "safetystatus"), ("program", "programState")):
            try:
                out[key] = dashboard_query(cmd)
            except (OSError, ValueError) as e:
                out[key] = f"unavailable ({e})"
        return out

    def readiness_problems(self, state, motion=True):
        """Reasons movement must NOT start. Empty list = ready."""
        p = []
        if self.fault_latched:
            p.append(f"Fault latched: {self.fault_latched}")
        if state is None:
            p.append("No fresh live robot state." + (f" ({self.rtde.error})" if self.rtde and self.rtde.error else ""))
            return p
        if state.robot_mode is not None and state.robot_mode != 7:
            p.append(f"Robot mode is {ROBOT_MODES.get(state.robot_mode, state.robot_mode)}, not RUNNING "
                     "(this program never powers on or releases brakes; do it on the teach pendant).")
        if state.safety_mode is not None and state.safety_mode not in (1, 2):
            p.append(f"Safety mode is {SAFETY_MODES.get(state.safety_mode, state.safety_mode)}; resolve it on the teach pendant.")
        if state.robot_mode is None or state.safety_mode is None:
            p.append("Robot/safety mode not reported by the interface; cannot confirm the robot is ready.")
        if state.runtime_state in (2, 3, 5):
            p.append(f"A program is already {RUNTIME_STATES[state.runtime_state]} on the controller.")
        for c in L.check_consistency(include_home=motion):
            p.append(c)
        return p

    # --- sending ---
    def _send_urscript(self, script):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(L.ROBOT_CONNECT_TIMEOUT_SECONDS)
        try:
            s.connect((L.ROBOT_IP, L.URSCRIPT_PORT))
            s.sendall(script.encode("utf-8"))
            time.sleep(0.3)
        finally:
            s.close()

    def _wait_for_completion(self, token, expect_pose, timeout, gripper_only=False):
        t0 = time.monotonic()
        seen_token = False
        while time.monotonic() - t0 < timeout:
            if self.stop_requested.is_set():
                return ExecutionResult("STOPPED", "Stop requested by the operator during execution.")
            st = self.get_state(max_age=max(L.STATE_MAX_AGE_SECONDS, 2.0))
            if st is None:
                time.sleep(0.1)
                continue
            if self.rtde.token_available and st.token_register == token:
                seen_token = True
            speed_ok = st.speed_norm() is None or st.speed_norm() < L.STOP_SPEED_EPSILON_M_S
            pose_ok = True
            if expect_pose is not None and st.pose is not None:
                pose_ok = (_dist(st.pose, expect_pose) <= L.POSE_POSITION_TOLERANCE_M
                           and rotation_angle_between(st.pose[3:], expect_pose[3:]) <= L.POSE_ORIENTATION_TOLERANCE_RAD)
            if self.rtde.token_available:
                # Token path: works for both arm motion and gripper-only programs.
                if seen_token and speed_ok and pose_ok:
                    return ExecutionResult("COMPLETED_CONFIRMED", "Program end marker seen and robot at rest"
                                           + (" at the expected pose." if expect_pose is not None else "."))
            else:
                # No RTDE token register (e.g. URCap occupies it, or older firmware).
                if expect_pose is not None and pose_ok and speed_ok and time.monotonic() - t0 > 1.0:
                    return ExecutionResult("COMPLETED_CONFIRMED",
                                           "Robot at rest at the expected pose (no end marker available).")
                # Gripper-only: no pose change and no token — use a conservative timed wait.
                if gripper_only and time.monotonic() - t0 >= L.GRIPPER_FALLBACK_WAIT_SECONDS:
                    return ExecutionResult("COMPLETED_TIMED",
                                           f"Gripper script sent; waited {L.GRIPPER_FALLBACK_WAIT_SECONDS:.1f} s "
                                           "for OnRobot URCap (no RTDE end-marker available on this controller). "
                                           "Confirm gripper moved visually.")
            time.sleep(0.05)
        return ExecutionResult("SENT_NOT_CONFIRMED", f"Script was sent but completion was NOT confirmed within {timeout:.0f} s. "
                               "Check the robot; motion state is unknown.")

    def execute(self, result, approved_sha256, wait=True):
        """Send a validated program. `result` must be a ValidationResult that passed; it is re-validated
        against fresh state and compared byte-for-byte (SHA-256) with what was approved."""
        if not result.ok:
            return ExecutionResult("REFUSED", "Validation did not pass; nothing sent.")
        if not L.get_live():
            return ExecutionResult("REFUSED", "LIVE is OFF (execution lockout). Nothing sent.")
        if hashlib.sha256("\n".join(result.body_lines).encode()).hexdigest() != approved_sha256:
            return ExecutionResult("REFUSED", "Code differs from what was approved. Nothing sent.")
        if not self._busy.acquire(blocking=False):
            return ExecutionResult("REFUSED", "Another program is still executing.")
        try:
            self.stop_requested.clear()
            state = self.get_state()
            problems = self.readiness_problems(state, motion=result.has_motion)
            if problems:
                return ExecutionResult("REFUSED", " | ".join(problems))
            fresh = validate_body(result.body_lines, state)
            if not fresh.ok or fresh.sha256 != approved_sha256:
                return ExecutionResult("REFUSED", "Re-validation against the current robot state failed: " + fresh.summary())
            if result.start_state_pose and state.pose and _dist(result.start_state_pose, state.pose) > L.STATE_CHANGE_TOLERANCE_M:
                return ExecutionResult("REFUSED", "The robot moved since this plan was validated; ask again so it is re-validated and re-approved.")
            if self.dry_run:
                self._simulate(fresh)
                return ExecutionResult("SIMULATED", "Dry-run only: nothing was sent to any robot and nothing physical happened.")
            gripper_only = fresh.has_gripper and not fresh.has_motion
            self._token_counter += 1
            token = self._token_counter
            # script = build_urscript(fresh.body_lines, token, marker=True)
            script = build_urscript(fresh.body_lines, token, marker=not gripper_only)
            try:
                self._send_urscript(script)
            except OSError as e:
                self.fault_latched = f"sending the script failed ({e}); motion state unknown"
                return ExecutionResult("FAILED", self.fault_latched)
            if gripper_only:
                return self._finish_gripper(fresh)
            if not wait:
                return ExecutionResult("SENT_NOT_CONFIRMED", "Command sent; not waiting for completion.")
            timeout = fresh.estimated_seconds * 2 + L.EXECUTION_TIMEOUT_MARGIN_SECONDS
            r = self._wait_for_completion(token, fresh.final_pose, timeout,
                                          gripper_only=fresh.has_gripper and not fresh.has_motion)
            if r.status == "SENT_NOT_CONFIRMED":
                self.fault_latched = "program completion was not confirmed; verify the robot, then restart this application"
            return r
        finally:
            self._busy.release()

    def _wait_timed(self, seconds):
        """Gripper-only completion. There is no end-of-program signal for this script form, so the program is
        reported as TIMED (duration elapsed, no fault seen), never as a confirmed grip. The operator verifies the
        gripper physically. Stop and a safety fault during the wait are honoured."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            if self.stop_requested.is_set():
                return ExecutionResult("STOPPED", "Stop requested during the gripper program.")
            st = self.get_state(max_age=max(L.STATE_MAX_AGE_SECONDS, 2.0))
            if st is not None and st.safety_mode is not None and st.safety_mode not in (1, 2):
                self.fault_latched = f"safety mode {SAFETY_MODES.get(st.safety_mode, st.safety_mode)} during gripper program"
                return ExecutionResult("FAILED", self.fault_latched)
            time.sleep(0.1)
        return ExecutionResult("COMPLETED_TIMED",
                               f"Gripper program sent and {seconds:.1f} s elapsed with no safety fault. The controller gives no "
                               "end-of-program signal for this form, so the grip is NOT confirmed by software: check the gripper.")

    def _finish_gripper(self, fresh):
        """Gripper-only completion. Status ladder (never claims more than was measured):
          COMPLETED_CONFIRMED     the URCap width read back matches the commanded width (within tolerance)
          COMPLETED_WIDTH_MISMATCH the width was read back but differs (object in the fingers, or missed target)
          COMPLETED_TIMED         no width readback available (RTDE register not configured, or no value arrived)"""
        target = last_grip_target(fresh.body_lines)
        r = self._wait_timed(fresh.estimated_seconds + L.GRIPPER_TIMED_MARGIN_SECONDS)
        if r.status != "COMPLETED_TIMED":
            return r
        if target is None or not (self.rtde and self.rtde.width_available):
            return r
        measured, t0 = None, time.monotonic()
        while time.monotonic() - t0 < GRIPPER_WIDTH_READ_TIMEOUT_S:
            st = self.get_state(max_age=max(L.STATE_MAX_AGE_SECONDS, 2.0))
            v = getattr(st, "double_register_0", None) if st is not None else None
            if v is not None and v >= 0.0:
                measured = v
                break
            time.sleep(0.1)
        if measured is None:
            return ExecutionResult("COMPLETED_TIMED", "Gripper program finished, but no width value arrived over RTDE "
                                   "(check the Log tab for an rg_get_width error). Grip NOT confirmed by software.")
        tol = getattr(L, "GRIPPER_WIDTH_TOL_MM", GRIPPER_WIDTH_TOL_MM_DEFAULT)
        if abs(measured - target) <= tol:
            return ExecutionResult("COMPLETED_CONFIRMED", f"Gripper width measured {measured:.1f} mm, target {target:.1f} mm "
                                   f"(tolerance {tol:g} mm).")
        return ExecutionResult("COMPLETED_WIDTH_MISMATCH", f"Gripper width measured {measured:.1f} mm, target {target:.1f} mm. "
                               "Probably an object is in the fingers or the move did not reach the target. Check the gripper.")

    def _simulate(self, res):
        pose_now = res.final_pose if res.final_pose is not None else res.final_pose_est
        if pose_now is not None:
            self._sim = RobotState("SIMULATED", q=res.final_q, pose=list(pose_now), tcp_speed=[0.0] * 6,
                                   qd=[0.0] * 6, robot_mode=7, safety_mode=1, runtime_state=1)

    # --- home ---
    def home_program_lines(self):
        h = list(L.HOME_POSE)
        return [f"movel(p[{', '.join(_fmt(x) for x in h)}], a={_fmt(L.HOME_TCP_ACCELERATION)}, v={_fmt(L.HOME_TCP_SPEED)})"]

    def home(self):
        """Validate (same validator as every other motion) and move to HOME_POSE; wait for confirmation."""
        state = self.get_state()
        problems = self.readiness_problems(state)
        if problems:
            return ExecutionResult("REFUSED", " | ".join(problems))
        res = validate_body(self.home_program_lines(), state)
        if not res.ok:
            return ExecutionResult("REFUSED", res.summary())
        return self.execute(res, res.sha256)

    # --- stop ---
    def stop(self):
        """Immediate software stop. Callable from any thread, any time. Bypasses LLM and approval.
        Returns (requested: bool, confirmed: bool|None, text)."""
        self.stop_requested.set()
        if self.dry_run:
            return True, True, "SIMULATED stop (dry-run): no robot is connected."
        notes, requested = [], False
        script = f"def ai_stop():\n  stopj({L.STOP_JOINT_DECELERATION})\nend\nai_stop()\n"
        try:
            self._send_urscript(script)
            requested = True
            notes.append("stopj() script sent on the URScript port")
        except OSError as e:
            notes.append(f"stopj script FAILED ({e})")
        try:
            notes.append("dashboard stop -> " + dashboard_query("stop", timeout=2.0))
            requested = True
        except (OSError, ValueError) as e:
            notes.append(f"dashboard stop FAILED ({e})")
        confirmed = self._confirm_stopped()
        if confirmed is not True:
            self.fault_latched = ("stop was requested but a stopped robot could not be confirmed"
                                  if requested else "stop could NOT be delivered through any channel")
        if not requested:
            return False, False, "STOP NOT DELIVERED (" + "; ".join(notes) + "). USE THE PHYSICAL EMERGENCY STOP."
        word = {True: "robot confirmed stopped", False: "robot still moving", None: "stopped state UNKNOWN (no fresh state)"}[confirmed]
        return True, confirmed, "; ".join(notes) + f" => {word}. New movement is blocked." if confirmed is not True \
            else "; ".join(notes) + f" => {word}."

    def _confirm_stopped(self):
        t0, good = time.monotonic(), 0
        seen = False
        while time.monotonic() - t0 < L.STOP_CONFIRM_TIMEOUT_SECONDS:
            st = self.rtde.get(max_age=0.5)
            if st is not None and (st.speed_norm() is not None or st.joint_speed_max() is not None):
                seen = True
                v_ok = st.speed_norm() is None or st.speed_norm() < L.STOP_SPEED_EPSILON_M_S
                q_ok = st.joint_speed_max() is None or st.joint_speed_max() < L.STOP_JOINT_SPEED_EPSILON_RAD_S
                good = good + 1 if (v_ok and q_ok) else 0
                if good >= 3:
                    return True
            time.sleep(0.1)
        return False if seen else None

    def close(self):
        if self.rtde:
            self.rtde.close()