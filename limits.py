"""limits.py -- the ONE place for every user-editable setting.

Edit only the block directly below. All other files import from here.
Session-only changes made with `/limit` never rewrite this file.
"""

# =====================================================================
# ================= EDIT THESE SETTINGS (EVERYTHING HERE) =============
# =====================================================================

# ---------------------------------------------------------------------
# A. Python / runtime and local LLM
# ---------------------------------------------------------------------
MIN_PYTHON = (3, 13)                 # minimum Python version (3.13+); no fixed patch requirement.
OLLAMA_URL = "http://localhost:11434" # local Ollama only. No cloud, no external API.
OLLAMA_MODEL = "gpt-oss:20b"          # Tested with this tag; set another installed Ollama model explicitly if desired.
LLM_TIMEOUT_SECONDS = 600.0           # s, per request (a 20B model on CPU can be slow)
LLM_TEMPERATURE = 0.1                 # low = more repeatable plans/code
LLM_NUM_CTX = 16384                   # tokens of context requested from Ollama
LLM_MAX_OUTPUT_TOKENS = 8192          # tokens per reply (gpt-oss "thinking" uses part of this)
LLM_THINK = "medium"                  # "low" | "medium" | "high", or None to omit the option
LLM_JSON_RETRIES = 2                  # re-asks when a reply is not valid JSON/schema (no robot action)
PLANNER_MAX_TOOL_CALLS = 3            # max read-only tool calls (e.g. get_robot_state) per task
LLM_EMPTY_RETRIES = 2                 # an EMPTY reply is retried this many times with a SHORT focused prompt
LLM_RETRY_THINK = "low"               # reasoning effort used for those short retries (None = same as LLM_THINK)
CODEGEN_MODE = "template"             # "template": step code is rendered by software from the plan step
                                      #             (most reliable; the validator still checks every line).
                                      # "llm":      the model writes each step's line from the exact templates;
                                      #             the line must equal the plan step or it is rejected.

# ---------------------------------------------------------------------
# B. Robot connection (UR5e)
# ---------------------------------------------------------------------
ROBOT_IP = "192.168.X.X"           # Publication placeholder: set your controller address before LIVE use.
URSCRIPT_PORT = 30002                 # URScript sent with a plain socket (your proven method)
DASHBOARD_PORT = 29999                # read-only status + "stop"; never power/brake/unlock
RTDE_PORT = 30004                     # live state (read only)
RTDE_FREQUENCY_HZ = 10.0              # Hz, live state update rate (max 125)
ROBOT_CONNECT_TIMEOUT_SECONDS = 3.0   # s
ROBOT_READ_TIMEOUT_SECONDS = 3.0      # s
STATE_MAX_AGE_SECONDS = 1.0           # s, live state older than this counts as unavailable

# ---------------------------------------------------------------------
# C. Home TCP pose  p[x, y, z, rx, ry, rz]
#    x,y,z in METRES; rx,ry,rz = rotation VECTOR in RADIANS (not Euler angles).
#    Taken from the real controller screen. Do not edit unless the home changes.
# ---------------------------------------------------------------------
HOME_POSE = [0.49246, -0.135581, 0.260306, 2.217481, 2.218513, -0.007947]
HOME_TCP_SPEED = 0.05                 # m/s   used by startup homing and /home
HOME_TCP_ACCELERATION = 0.1           # m/s^2 used by startup homing and /home

# ---------------------------------------------------------------------
# D. Workspace and motion limits
#    The box is the MAXIMUM of the robot's TCP workspace. It is NOT a promise that the arm can reach every point of it,
#    and the robot is not obliged to reach the maximum: which heights the arm can reach depends on the DISTANCE from the base
#    and on the DIRECTION (azimuth), because the wrist offset and the tool orientation change the effective reach. When a
#    task asks for a point the arm physically cannot reach from where it stands, the
#    software ADJUSTS the coordinates automatically (reach envelope, section K): it gets as close as it comfortably can,
#    stops short of the edge, and says so in the plan. No question is asked.
#    Turning the base by 180 degrees from HOME (TCP radius about 0.51 m) sweeps the TCP over y = +-0.511 m and ends at
#    x = -0.492 m, so X_MIN and the Y limits must reach at least that far or the validator refuses the rotation.
#    HOME_POSE (x, y, z) must lie inside the box, otherwise ALL robot movement is blocked (by design).
# ---------------------------------------------------------------------
X_MIN = -0.550000                     # m  (was 0.150000) behind the base axis: needed for a 180 degree base turn (ends at x = -0.492)
X_MAX = 0.657000                      # m  unchanged
Y_MIN = -0.550000                     # m  (was -0.320000) the base turn passes y = -0.511 when it swings through -Y
Y_MAX = 0.550000                      # m  (was 0.320000) ... and y = +0.511 through +Y
Z_MIN = 0.020180                      # m  unchanged; close to the base the envelope raises the floor (model result for a tool pointing down, not a specification)
Z_MAX = 0.650000                      # m  (was 0.500000) a tool pointing down reaches about 0.60 m close to the base (model result); the envelope lowers it farther out
MAX_TCP_SPEED = 0.05                  # m/s    previously supplied value
MAX_TCP_ACCELERATION = 0.1            # m/s^2  previously supplied value

# Absolute sanity bounds for /limit changes (typo protection, not a safety rating)
SANITY_MAX_ABS_POSITION_M = 1.2       # m
SANITY_MAX_TCP_SPEED = 1.0            # m/s
SANITY_MAX_TCP_ACCELERATION = 3.0     # m/s^2

# ---------------------------------------------------------------------
# E. Default motion values and joint-space (movej) limits
# ---------------------------------------------------------------------
DEFAULT_TCP_SPEED = 0.02              # m/s    used when a plan step gives no speed
DEFAULT_TCP_ACCELERATION = 0.05       # m/s^2  used when a plan step gives no acceleration
DEFAULT_JOINT_SPEED = 0.2             # rad/s  movej default
DEFAULT_JOINT_ACCELERATION = 0.3      # rad/s^2
MAX_JOINT_SPEED = 0.5                 # rad/s  movej hard cap
MAX_JOINT_ACCELERATION = 1.0          # rad/s^2
# Conservative joint bounds in RADIANS, order: base, shoulder, elbow, wrist1, wrist2, wrist3
JOINT_LIMITS_RAD = [(-6.0, 6.0), (-3.3, 3.3), (-3.3, 3.3),
                    (-6.0, 6.0), (-6.0, 6.0), (-6.0, 6.0)]
MAX_SLEEP_SECONDS = 10.0              # s per sleep() call
PATH_SAMPLE_STEP_M = 0.005            # m, spacing of path samples used for workspace checks

# ---------------------------------------------------------------------
# F. Gripper (OnRobot RG2/RG6 via rg_grip(width_mm, force_N))
#    Defaults below are RG2-style bounds -- edit to match your gripper.
# ---------------------------------------------------------------------
GRIPPER_ENABLED = True
GRIPPER_WIDTH_MIN_MM = 3.0            # mm
GRIPPER_WIDTH_MAX_MM = 110.0          # mm  RG2 mechanical maximum
GRIPPER_FINGERTIP_OFFSET_MM = 2.0     # mm  "Fingertip offset RG2" on the pendant (OnRobot Setup screen)
# The OnRobot URCap clamps the width to MAX - 2 * fingertip offset (110 - 4 = 106 mm). Validator uses this value,
# so a 110 mm request is refused here instead of being silently clamped on the robot.
GRIPPER_WIDTH_MAX_EFFECTIVE_MM = GRIPPER_WIDTH_MAX_MM - 2 * GRIPPER_FINGERTIP_OFFSET_MM
GRIPPER_FORCE_MIN_N = 3.0             # N
GRIPPER_FORCE_MAX_N = 40.0            # N

# ---------------------------------------------------------------------
# G. Program size, repair loop, timeouts, stop handling
# ---------------------------------------------------------------------
# Plan step actions the planner may use. Only actions implemented in robot_sender.CAPABILITIES work:
# loop/movep/distance are NOT implemented, so they are not exposed (the planner must say "unsupported").
ALLOWED_ACTIONS = ("movel", "movej", "movec", "home", "repeat", "gripper", "sleep")
MAX_REPEAT_COUNT = 20                 # repeat blocks are expanded by software into validated flat steps
MIN_ARC_RADIUS_M = 0.005              # m   movec circles smaller than this are rejected
MAX_ARC_RADIUS_M = 0.5                # m   movec circles larger than this are rejected
MAX_ARC_SWEEP_DEG = 350.0             # deg arcs of (almost) full circles are rejected
MAX_PLAN_STEPS = 15
MAX_PROGRAM_LINES = 40
MAX_REPAIR_ATTEMPTS = 3               # repairs after the first generation
APPROVAL_TIMEOUT_SECONDS = 300.0      # s to type APPROVE, otherwise nothing runs
# Gripper-only programs: rg_grip() returns control immediately (OnRobot URCap is async).
# We insert a sleep after rg_grip so the token write happens after the gripper physically moves.
LIMIT_MARGIN_M = 0.001                # m  "until limit" moves stop this far INSIDE the limit (never touch it)
EXACT_RETURN_TOL_M = 0.0001           # m  a return to a saved pose counts as exact within this distance (0.1 mm)
GRIPPER_TIMED_MARGIN_SECONDS = 2.0    # s  extra wait for gripper-only programs (no end marker, timed completion)
GRIPPER_MOVE_ESTIMATE_SECONDS = 3.0   # s  planning estimate for one rg_grip (blocking call; the URCap waits for the move)
GRIPPER_SETTLE_SLEEP_SECONDS = 3.5    # s injected after each rg_grip() before the end-marker write
# If output_int_register_0 is unavailable (URCap conflict), fall back to a timed wait instead.
GRIPPER_FALLBACK_WAIT_SECONDS = 6.0   # s to wait after script send when token is unavailable
STOP_CONFIRM_TIMEOUT_SECONDS = 5.0    # s to wait for "robot is really stopped"
STOP_JOINT_DECELERATION = 2.0         # rad/s^2 used by the stopj() stop command
STOP_SPEED_EPSILON_M_S = 0.002        # m/s   below this TCP speed counts as stopped
STOP_JOINT_SPEED_EPSILON_RAD_S = 0.005  # rad/s below this joint speed counts as stopped
EXECUTION_TIMEOUT_MARGIN_SECONDS = 15.0 # s added on top of the estimated program duration
POSE_POSITION_TOLERANCE_M = 0.002     # m   "arrived" position tolerance
POSE_ORIENTATION_TOLERANCE_RAD = 0.02 # rad "arrived" orientation tolerance
STATE_CHANGE_TOLERANCE_M = 0.003      # m   robot moved this much since validation => re-approve
REPORTS_DIR = "reports"               # failure reports (Markdown) are written here
APPROVAL_DIR = "approval"               # approved reports (Markdown) are written here

# ---------------------------------------------------------------------
# H. Behaviour switches and language convention
# ---------------------------------------------------------------------
LIVE_AT_START = True                  # default LIVE. /settings can lock execution OFF (not simulation).
# Words the planner may use. None = "not defined" -> the planner must ask the user.
# Example: "forward": "+X"  (only set this if you know your base-frame convention)
DIRECTION_CONVENTION = {
    "up": "+Z", "down": "-Z",
    "forward": "+X", "backward": "-X", "left": "+Y", "right": "-Y",
}
# --dry-run ONLY: simulated joint start (rad). Always labelled SIMULATED, never real data.
SIM_START_JOINTS = [-0.0, -1.5713, -1.5703, -1.5708, 1.5708, 3.1415]   # = the joints of the real robot at HOME_POSE (see /status)

# ---------------------------------------------------------------------
# I. Terminal UI (ui.py / main.py)
# ---------------------------------------------------------------------
UI_COLOR = True                       # ANSI colours. Switched off automatically when NO_COLOR is set or output is piped.
UI_ASCII = False                      # True = plain ASCII symbols/spinner (old consoles that show boxes instead of dots)
UI_SPINNER_FPS = 10.0                 # animation speed of the "Thinking..." status line
UI_MIN_TASK_CHARS = 7                 # a task for the AI must have at least this many characters
UI_MAX_TASK_CHARS = 1000              # ... and at most this many (keeps the 16k-token context safe)

# ---------------------------------------------------------------------
# J. /robot corner tour (tour.py)
#    Visits the 8 corners of the workspace box one after another. Each corner sits LIMIT_MARGIN_M inside the box.
#    Reachability uses the same envelope as every other program (section K).
# ---------------------------------------------------------------------
TOUR_SAMPLE_STEP_M = 0.01             # m   spacing of the reachability samples along each tour segment

# ---------------------------------------------------------------------
# K. Reach envelope + self-collision (robot_sender.py)  -- the "ADJUST" rules
#    The workspace box (section D) is NOT the whole story: the reachable heights depend on the distance from the base AND on the
#    direction (the wrist offset and the tool orientation matter), so a point inside the box may still be out of reach.
#    Every straight/arc path is therefore also solved with inverse kinematics from the LIVE joints and the LIVE tool.
#    A pose is OK only if: a joint solution exists, no joint hits its range, the elbow/wrist2 are not singular, the arm is
#    not stretched beyond REACH_COMFORT_FRACTION of its maximum reach ("not max"), and the capsule model below shows no
#    contact between the arm, the tool, the base column and the table.
#    When a plan asks for "as far as possible", "to the limit" or a corner, the software stops at the EDGE OF THIS
#    ENVELOPE (and says so in the plan as ADJUSTED: it is not the workspace limit, the robot simply finds it hard).
#    The capsule radii are generous APPROXIMATIONS (not the real housings): this is not certified collision checking.
# ---------------------------------------------------------------------
REACH_CHECK_ENABLED = True            # False = no envelope check at all (NOT recommended: a far corner can fault the robot)
REACH_SAMPLE_STEP_M = 0.02            # m   spacing of the checks along every move the validator verifies
REACH_IK_POS_TOL_M = 0.0005           # m   IK must hit the target within this distance
REACH_IK_ROT_TOL_RAD = 0.005          # rad ... and within this orientation error
REACH_IK_MAX_ITER = 60                # iterations per sample
REACH_MAX_JOINT_JUMP_RAD = 0.35       # rad max joint change between two samples (bigger = the arm would flip configuration)
REACH_MAX_ROT_STEP_RAD = 0.10         # rad max tool rotation between two samples (must be < REACH_MAX_JOINT_JUMP_RAD); long turns get more samples
REACH_SINGULARITY_MARGIN_RAD = 0.2    # rad keep the elbow and wrist 2 at least this far from singular poses
REACH_COMFORT_FRACTION = 0.90         # shoulder-to-wrist distance may be at most this fraction of the arm's maximum stretch (0.95 let far corners through at 94%)
REACH_ADJUST_BUFFER_M = 0.02          # m   an ADJUSTED target stays this far inside the envelope edge (never exactly on it)
FLOOR_Z_M = 0.0                       # m   table height in the base frame (links must stay above it)
BASE_COLUMN_RADIUS_M = 0.10           # m   keep-out cylinder around the base axis (base + shoulder housing)
BASE_COLUMN_HEIGHT_M = 0.30           # m   ... and its height
UPPER_ARM_RADIUS_M = 0.08             # m   capsule radii of the links (generous round numbers)
FOREARM_RADIUS_M = 0.07
WRIST_RADIUS_M = 0.06
TOOL_RADIUS_M = 0.08                  # m   gripper body (RG2 is about 0.15 m wide at its widest)
SELF_COLLISION_MARGIN_M = 0.02        # m   extra clearance kept between any two capsules
# Base keep-out: ALWAYS enforced by the validator, even when REACH_CHECK_ENABLED = False. The TCP path may not enter the
# cylinder around the base axis (column + tool + margin) below this height.
BASE_KEEPOUT_RADIUS_M = BASE_COLUMN_RADIUS_M + TOOL_RADIUS_M + SELF_COLLISION_MARGIN_M   # m (0.20)
BASE_KEEPOUT_HEIGHT_M = 0.60          # m   height of that cylinder above the base plane

# =====================================================================
# ============ END OF EDITABLE SETTINGS -- code below ================
# =====================================================================

import math

LIMIT_NAMES = {
    "x_min": "X_MIN", "x_max": "X_MAX", "y_min": "Y_MIN", "y_max": "Y_MAX",
    "z_min": "Z_MIN", "z_max": "Z_MAX",
    "max_speed": "MAX_TCP_SPEED", "max_accel": "MAX_TCP_ACCELERATION",
}
_UNITS = {   # limit name -> {accepted suffix: factor to SI}
    "position": {"": 1.0, "m": 1.0, "mm": 0.001},
    "speed": {"": 1.0, "m/s": 1.0, "mm/s": 0.001},
    "accel": {"": 1.0, "m/s2": 1.0, "m/s^2": 1.0, "mm/s2": 0.001, "mm/s^2": 0.001},
}


def _kind(name):
    return "speed" if name == "max_speed" else "accel" if name == "max_accel" else "position"


def _initial_limits():
    return {k: float(globals()[v]) for k, v in LIMIT_NAMES.items()}


ACTIVE = _initial_limits()            # session copy; changed only by set_limit()
RUNTIME = {"live": bool(LIVE_AT_START)}


def get_limits():
    return dict(ACTIVE)


def get_live():
    return RUNTIME["live"]


def set_live(value):
    RUNTIME["live"] = bool(value)


def reset_session_limits():
    ACTIVE.clear()
    ACTIVE.update(_initial_limits())


def fingerprint_material():
    """Everything that must invalidate a pending approval when it changes."""
    return {"limits": get_limits(), "live": get_live(), "home": list(HOME_POSE),
            "model": OLLAMA_MODEL, "gripper": GRIPPER_ENABLED}


def check_static_config():
    """Hard errors in the editable block itself (typos). Returns list of strings."""
    p = []
    if not (isinstance(HOME_POSE, (list, tuple)) and len(HOME_POSE) == 6
            and all(isinstance(v, (int, float)) and math.isfinite(v) for v in HOME_POSE)):
        p.append("HOME_POSE must be six finite numbers [x,y,z,rx,ry,rz].")
    if not isinstance(OLLAMA_MODEL, str) or not OLLAMA_MODEL.strip():
        p.append("OLLAMA_MODEL must be a non-empty exact model tag.")
    if len(JOINT_LIMITS_RAD) != 6 or any(len(t) != 2 or not t[0] < t[1] for t in JOINT_LIMITS_RAD):
        p.append("JOINT_LIMITS_RAD needs six (min, max) pairs with min < max.")
    if not GRIPPER_WIDTH_MIN_MM < GRIPPER_WIDTH_MAX_MM or not GRIPPER_FORCE_MIN_N < GRIPPER_FORCE_MAX_N:
        p.append("Gripper width/force min must be below max.")
    if CODEGEN_MODE not in ("template", "llm"):
        p.append('CODEGEN_MODE must be "template" or "llm".')
    if not (isinstance(LLM_EMPTY_RETRIES, int) and 0 <= LLM_EMPTY_RETRIES <= 5):
        p.append("LLM_EMPTY_RETRIES must be an integer 0..5.")
    if not (isinstance(MAX_REPEAT_COUNT, int) and MAX_REPEAT_COUNT >= 1):
        p.append("MAX_REPEAT_COUNT must be an integer >= 1.")
    if not 0 < MIN_ARC_RADIUS_M < MAX_ARC_RADIUS_M or not 0 < MAX_ARC_SWEEP_DEG < 360:
        p.append("Arc settings invalid (need 0 < MIN_ARC_RADIUS_M < MAX_ARC_RADIUS_M, 0 < MAX_ARC_SWEEP_DEG < 360).")
    for n, v in (("MAX_REPAIR_ATTEMPTS", MAX_REPAIR_ATTEMPTS), ("MAX_PLAN_STEPS", MAX_PLAN_STEPS),
                 ("MAX_PROGRAM_LINES", MAX_PROGRAM_LINES)):
        if not isinstance(v, int) or v < 1:
            p.append(f"{n} must be an integer >= 1.")
    if isinstance(REACH_MAX_ROT_STEP_RAD, (int, float)) and isinstance(REACH_MAX_JOINT_JUMP_RAD, (int, float)) \
            and not REACH_MAX_ROT_STEP_RAD < REACH_MAX_JOINT_JUMP_RAD:
        p.append("REACH_MAX_ROT_STEP_RAD must be smaller than REACH_MAX_JOINT_JUMP_RAD.")
    if not (isinstance(UI_MIN_TASK_CHARS, int) and isinstance(UI_MAX_TASK_CHARS, int)
            and 1 <= UI_MIN_TASK_CHARS < UI_MAX_TASK_CHARS):
        p.append("UI_MIN_TASK_CHARS / UI_MAX_TASK_CHARS must be integers with 1 <= min < max.")
    for n in ("DEFAULT_TCP_SPEED", "DEFAULT_TCP_ACCELERATION", "HOME_TCP_SPEED", "HOME_TCP_ACCELERATION",
              "UI_SPINNER_FPS", "TOUR_SAMPLE_STEP_M", "REACH_SAMPLE_STEP_M", "REACH_IK_POS_TOL_M", "REACH_IK_ROT_TOL_RAD",
              "REACH_MAX_JOINT_JUMP_RAD", "REACH_MAX_ROT_STEP_RAD", "REACH_SINGULARITY_MARGIN_RAD", "REACH_ADJUST_BUFFER_M", "BASE_COLUMN_RADIUS_M",
              "BASE_COLUMN_HEIGHT_M", "UPPER_ARM_RADIUS_M", "FOREARM_RADIUS_M", "WRIST_RADIUS_M", "TOOL_RADIUS_M",
              "DEFAULT_JOINT_SPEED", "DEFAULT_JOINT_ACCELERATION", "MAX_JOINT_SPEED",
              "MAX_JOINT_ACCELERATION", "MAX_SLEEP_SECONDS", "PATH_SAMPLE_STEP_M", "BASE_KEEPOUT_RADIUS_M", "BASE_KEEPOUT_HEIGHT_M"):
        v = globals()[n]
        if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
            p.append(f"{n} must be a positive finite number.")
    return p


def check_limit_values(lim):
    """Hard validity of a limits dict: finite, min < max, positive speeds, sane range."""
    p = []
    for k, v in lim.items():
        if not isinstance(v, float) or not math.isfinite(v):
            p.append(f"{k} is not a finite number.")
    if p:
        return p
    for ax in "xyz":
        if not lim[f"{ax}_min"] < lim[f"{ax}_max"]:
            p.append(f"{ax}_min ({lim[ax + '_min']}) must be strictly less than {ax}_max ({lim[ax + '_max']}).")
        for s in ("min", "max"):
            if abs(lim[f"{ax}_{s}"]) > SANITY_MAX_ABS_POSITION_M:
                p.append(f"{ax}_{s} is beyond +/-{SANITY_MAX_ABS_POSITION_M} m (unit typo?).")
    if not 0 < lim["max_speed"] <= SANITY_MAX_TCP_SPEED:
        p.append(f"max_speed must be in (0, {SANITY_MAX_TCP_SPEED}] m/s.")
    if not 0 < lim["max_accel"] <= SANITY_MAX_TCP_ACCELERATION:
        p.append(f"max_accel must be in (0, {SANITY_MAX_TCP_ACCELERATION}] m/s^2.")
    return p


def check_consistency(lim=None, include_home=True):
    """Everything that must hold before ANY movement is allowed.
    Includes the HOME_POSE vs workspace check. Returns list of problems."""
    lim = lim or get_limits()
    p = check_static_config() + check_limit_values(lim)
    if p:
        return p
    if include_home:
        for name, val in (("x", HOME_POSE[0]), ("y", HOME_POSE[1]), ("z", HOME_POSE[2])):
            lo, hi = lim[f"{name}_min"], lim[f"{name}_max"]
            if not lo <= val <= hi:
                p.append(f"CONFLICT: HOME_POSE {name} = {val:.6f} m is outside [{name}_min {lo:.6f}, "
                         f"{name}_max {hi:.6f}]. Fix the limits (or HOME_POSE) in the top block of limits.py.")
    for n, v, cap, unit in (("HOME_TCP_SPEED", HOME_TCP_SPEED, lim["max_speed"], "m/s"),
                            ("HOME_TCP_ACCELERATION", HOME_TCP_ACCELERATION, lim["max_accel"], "m/s^2"),
                            ("DEFAULT_TCP_SPEED", DEFAULT_TCP_SPEED, lim["max_speed"], "m/s"),
                            ("DEFAULT_TCP_ACCELERATION", DEFAULT_TCP_ACCELERATION, lim["max_accel"], "m/s^2")):
        if v > cap + 1e-12:
            p.append(f"{n} = {v} {unit} exceeds the active maximum {cap} {unit}.")
    return p


def parse_limit_value(name, text):
    """'0.30', '300mm', '0.05 m/s' -> float in SI units. Raises ValueError with a clear message."""
    kind = _kind(name)
    t = text.strip().lower().replace(" ", "")
    for suffix in sorted(_UNITS[kind], key=len, reverse=True):
        if suffix and t.endswith(suffix):
            num, factor = t[: -len(suffix)], _UNITS[kind][suffix]
            break
    else:
        num, factor = t, 1.0
    try:
        v = float(num) * factor
    except ValueError:
        accepted = ", ".join(s for s in _UNITS[kind] if s) or "none"
        raise ValueError(f"'{text}' is not a number (accepted unit suffixes for {name}: {accepted}).")
    if not math.isfinite(v):
        raise ValueError("value must be finite (no NaN/inf).")
    return v


def set_limit(name, text):
    """Session-only change of one limit. Returns (ok, message)."""
    name = name.lower()
    if name not in LIMIT_NAMES:
        return False, f"Unknown limit '{name}'. Known: {', '.join(LIMIT_NAMES)}"
    try:
        v = parse_limit_value(name, text)
    except ValueError as e:
        return False, str(e)
    trial = get_limits()
    trial[name] = v
    errs = check_limit_values(trial)
    if errs:
        return False, "Rejected, nothing changed: " + " ".join(errs)
    ACTIVE[name] = v
    msg = f"{name} = {v:g} (session only; edit {LIMIT_NAMES[name]} at the top of limits.py to make it permanent)."
    left = check_consistency()
    if left:
        msg += " Movement is still BLOCKED: " + " | ".join(left)
    else:
        msg += " Configuration is consistent."
    return True, msg
