"""main.py -- terminal UI for the UR5e AI controller (see ui.py for the keyboard/animation machinery).

Run:  python main.py            (LIVE, the normal mode)
      python main.py --dry-run  (explicit simulation only; never chosen automatically)
"""
import datetime
import json
import os
import sys
import time

import ai_agent as A
import limits as L
import robot_sender as R
import tour as T
import ui as UI
from ui import paint

STOP_WORDS = {"/stop", "stop", "s"}

# command -> one-line help (also feeds Tab completion)
COMMANDS = {
    "/help": "this help",
    "/status": "REAL robot state and active limits",
    "/ai": "describe a task in plain language (plain text without /ai works too)",
    "/robot": "visit the 8 workspace corners at the fastest allowed speed, then back to the start (no AI)",
    "/home": "validated move to HOME_POSE",
    "/limit": "show limits, or change one for this session:  /limit z_max 400mm",
    "/settings": "show settings;  /settings live on|off = execution lockout",
    "/log": "show the process log of the last AI task / tour",
    "/clear": "forget the current task conversation and any pending plan",
    "/cls": "clear the screen",
    "/stop": "immediate software stop (never goes through the AI)",
    "/exit": "quit (does not stop or move the robot)",
}
ALIASES = {"/corners": "/robot", "/tour": "/robot", "/clearscreen": "/cls"}
COMPLETIONS = list(COMMANDS) + list(ALIASES)
BARE_COMMANDS = {c[1:] for c in COMMANDS} | {"quit"}

WARN_REACH_OFF = "REACH CHECK OFF: arm reach and self-collision are NOT checked (the base keep-out cylinder still is)."
_LAST = {"pipe": None}          # process log of the most recent task, for /log
_CON = None                      # the Console (global so the final cleanup can restore the terminal)


def out(text=""):
    UI.say(UI.colorize(text))


def note(text, style="gray"):
    UI.say(paint(text, style))


# ---------------------------------------------------------------------------------------------
# Static texts
# ---------------------------------------------------------------------------------------------
def show_help():
    UI.say(UI.rule("commands"))
    rows = [("/ai <task>", COMMANDS["/ai"] + f"; {L.UI_MIN_TASK_CHARS}-{L.UI_MAX_TASK_CHARS} characters"),
            ("/robot [speed] [stay] [fixed]", "corner tour: 8 points one after another at the max allowed speed (or the speed you give, "
                                      "e.g. /robot 0.04 or /robot 40mm/s); 'stay' = do not return to the start; 'fixed' = keep the tool "
                                      "orientation (default: the tool turns with the arm). No AI."),
            ("/home", COMMANDS["/home"]), ("/status", COMMANDS["/status"]),
            ("/limit [name value]", COMMANDS["/limit"]), ("/settings [live on|off]", COMMANDS["/settings"]),
            ("/log", COMMANDS["/log"]), ("/clear", COMMANDS["/clear"]), ("/cls", COMMANDS["/cls"]),
            ("/stop | stop | s", COMMANDS["/stop"]), ("/exit", COMMANDS["/exit"])]
    for name, desc in rows:
        UI.say(f"  {paint(name.ljust(24), 'cyan', 'bold')} {desc}")
    UI.say()
    UI.say(f"  {paint('Keys', 'bold')}")
    UI.say(f"  {paint('while AI works / robot moves'.ljust(30), 'gray')} {paint('Enter', 'cyan')} show/hide the process details"
           f"   {paint('s', 'red', 'bold')} STOP at once")
    hist = "Up/Down" if UI.ASCII else "↑ ↓"
    UI.say(f"  {paint('at the prompt'.ljust(30), 'gray')} {paint(hist, 'cyan')} history"
           f"   {paint('Tab', 'cyan')} complete   {paint('Ctrl+L', 'cyan')} clear screen   {paint('Esc', 'cyan')} clear the line")
    UI.say(f"  {paint('plan approval'.ljust(30), 'gray')} {paint(UI.SYM['arrows'], 'cyan')} choose APPROVE / CANCEL,"
           f" {paint('Enter', 'cyan')} confirm, {paint('Esc', 'cyan')} cancel")
    UI.say()
    UI.say(paint("Examples:  move 10 mm up  |  return to the exact position where you started", "gray"))
    UI.say(paint("           move 40 mm +X, 30 mm +Y, draw a 25 mm radius semicircle in the XZ plane, return to the start pose", "gray"))
    UI.say(paint("           repeat moving 10 mm +X five times  |  rotate the tool 30 degrees about Z, keep the position", "gray"))
    UI.say(paint("           /ai close the gripper to 30 mm with 20 N, wait 2 s, then open it fully", "gray"))
    UI.say(paint("Primitives: movel, movej, movec (arcs), polygon, circle, home, repeat (N times), gripper, sleep.  "
                 "NOT implemented: movep, distance, IK goals.", "gray"))
    UI.say(paint("Plans always show the code; nothing runs until you choose APPROVE.", "yellow"))


def show_header(robot, dry):
    mode = "DRY-RUN (simulated)" if dry else ("LIVE" if L.get_live() else "LIVE connection, execution LOCKED")
    style = "magenta" if dry else ("green" if L.get_live() else "yellow")
    sep = paint(f"  {UI.SYM['mid']}  ", "gray")
    UI.say(paint("UR5e AI controller", "bold", "cyan") + sep + paint(mode, style, "bold") + sep
           + paint(f"model {L.OLLAMA_MODEL}", "gray") + sep + paint(f"robot {L.ROBOT_IP}", "gray"))
    UI.say(paint("/help for commands  " + UI.SYM["mid"] + "  Tab completes  " + UI.SYM["mid"]
                 + "  s = STOP while something runs", "gray"))


def show_limits():
    lim = L.get_limits()
    out("Active limits (session values; permanent edits go in the top block of limits.py):")
    for k in L.LIMIT_NAMES:
        unit = "m/s" if k == "max_speed" else "m/s^2" if k == "max_accel" else "m"
        out(f"  {k:10s} = {lim[k]:.6f} {unit}")
    out(f"  HOME_POSE  = {L.HOME_POSE}  (m, rad rotation vector)")
    out(f"  movej: v<= {L.MAX_JOINT_SPEED} rad/s, a<= {L.MAX_JOINT_ACCELERATION} rad/s^2; sleep<= {L.MAX_SLEEP_SECONDS} s; "
        f"max {L.MAX_PLAN_STEPS} steps / {L.MAX_PROGRAM_LINES} lines")
    out(f"  gripper: {'enabled' if L.GRIPPER_ENABLED else 'DISABLED'}, width {L.GRIPPER_WIDTH_MIN_MM}-{L.GRIPPER_WIDTH_MAX_EFFECTIVE_MM:g} mm, "
        f"force {L.GRIPPER_FORCE_MIN_N}-{L.GRIPPER_FORCE_MAX_N} N")
    out(f"  reach envelope: {'ON' if L.REACH_CHECK_ENABLED else 'OFF (NOT recommended)'}; arm stretch <= "
        f"{L.REACH_COMFORT_FRACTION * 100:.0f}% of its maximum, ADJUSTED targets stop {L.REACH_ADJUST_BUFFER_M * 100:.0f} cm "
        f"before the edge of the arm's reach (section K of limits.py)")
    problems = L.check_consistency()
    if problems:
        UI.say("  CONFIGURATION CHECK: " + paint("PROBLEMS - movement blocked:", "red", "bold"))
        for p in problems:
            UI.say(paint("   - " + p, "red"))
    else:
        UI.say("  CONFIGURATION CHECK: " + paint("consistent", "green"))


def show_status(robot):
    live = L.get_live()
    mode = paint("DRY-RUN (SIMULATED)", "magenta", "bold") if robot.dry_run else paint("LIVE connection", "green", "bold")
    UI.say(f"Mode: {mode} | LIVE execution: " + (paint("ON", "green", "bold") if live else paint("OFF (lockout)", "yellow", "bold"))
           + f" | model: {L.OLLAMA_MODEL}")
    if not L.REACH_CHECK_ENABLED:
        UI.say(paint(WARN_REACH_OFF, "red", "bold"))
    st = robot.get_state()
    out("Robot state:\n  " + (st.to_text() if st else "UNAVAILABLE (no fresh state"
                             + (f": {robot.rtde.error}" if robot.rtde and robot.rtde.error else "") + ")"))
    tool = R.tool_from_state(st)
    if tool is not None:
        why = R.envelope_problem(st.q, tool)
        out(f"  arm stretch: {R.stretch_fraction(st.q) * 100:.0f}% of maximum reach (comfort limit "
            f"{L.REACH_COMFORT_FRACTION * 100:.0f}%); tool length {tool.length * 100:.1f} cm; "
            + ("inside the reach envelope." if why is None else "OUTSIDE the envelope: " + why))
    if not robot.dry_run:
        out("  dashboard: " + json.dumps(robot.dashboard_status()))
        out(f"  end-of-program marker register: {'available' if robot.rtde.token_available else 'unavailable'}")
    if robot.fault_latched:
        UI.say(paint(f"  FAULT LATCHED (movement blocked): {robot.fault_latched}", "red", "bold"))
    problems = robot.readiness_problems(st)
    UI.say("  movement readiness: " + (paint("READY", "green", "bold") if not problems else paint("BLOCKED", "red", "bold")))
    for p in problems:
        UI.say(paint("   - " + p, "red"))
    show_limits()


def show_candidate(c):
    """Final candidate summary (the pipeline log showed every attempt)."""
    v = c.validation
    out(A.banner("FINAL CANDIDATE", "validated, waiting for your approval"))
    if not L.REACH_CHECK_ENABLED:
        UI.say(paint(WARN_REACH_OFF, "red", "bold"))
    out(A.banner("PLAN"))
    out(A.render_plan(c.plan, c.steps))
    _adj = A.render_adjustments(c.steps)
    if _adj:
        out("")
        out(_adj)
    out("  Live state when validated:\n    " + c.state_text.replace("\n", "\n    "))
    out(A.banner("STEP CODE"))
    out(A.render_codes(c.steps, c.step_codes))
    out(A.banner("FULL PROGRAM", "exactly what is sent inside def ai_task() ... end"))
    out(A.render_program(c.program_lines))
    out(A.banner("AI REVIEW"))
    out(A.render_review(c.review))
    out(A.banner("VALIDATOR", "deterministic, authoritative"))
    out(A.render_validator(v))
    if c.repaired:
        UI.say(paint("\n  Repaired locally during this task: plan step(s) " + ", ".join(".".join(map(str, p)) for p in c.repaired), "yellow"))
    UI.say(UI.rule("", "cyan"))


# ---------------------------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------------------------
def check_python():
    v = sys.version_info
    minimum = L.MIN_PYTHON
    if v.major != 3 or (v.major, v.minor) < tuple(minimum):
        raise SystemExit(f"ERROR: Python {minimum[0]}.{minimum[1]} or newer (Python 3) is required, "
                         f"but you are running {v.major}.{v.minor}.{v.micro}.")


def make_stop(robot):
    def _stop():
        UI.say(paint("\n!!! STOP requested - sending stop now ...", "red", "bold"))
        req, conf, text = robot.stop()
        UI.say(paint(("STOP: " if req else "STOP FAILED: ") + text, "red" if not req else "yellow", "bold"))
    return _stop


def make_prompt(robot):
    """A coloured status dot + mode tag: green = ready, yellow = execution locked, red = blocked, magenta = dry-run."""
    if robot.dry_run:
        tag, col = "DRY", "magenta"
    elif robot.fault_latched or robot.readiness_problems(robot.get_state()):
        tag, col = "BLOCKED", "red"
    elif not L.get_live():
        tag, col = "LOCK", "yellow"
    else:
        tag, col = "LIVE", "green"
    return f"{paint(UI.SYM['dot'], col)} {paint('ur5e', 'bold')} {paint(tag, col)} {paint(UI.SYM['prompt'], 'cyan')} "


def task_text_problem(text):
    n = len(text.strip())
    if n < L.UI_MIN_TASK_CHARS:
        return (f"Task too short: {n}/{L.UI_MIN_TASK_CHARS} characters minimum. Say what the robot should do, "
                "e.g. 'move 10 mm up'. (Commands start with /, see /help.)")
    if n > L.UI_MAX_TASK_CHARS:
        return f"Task too long: {n} characters (maximum {L.UI_MAX_TASK_CHARS}). Shorten it or split it into two tasks."
    return None


def get_approval_filepath(prefix="approval"):
    os.makedirs(L.APPROVAL_DIR, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(L.APPROVAL_DIR, f"{prefix}_{timestamp}.md")


_STATUS_STYLE = {"COMPLETED_CONFIRMED": "green", "SIMULATED": "cyan", "COMPLETED_TIMED": "yellow",
                 "COMPLETED_WIDTH_MISMATCH": "yellow", "SENT_NOT_CONFIRMED": "yellow", "STOPPED": "yellow",
                 "REFUSED": "red", "FAILED": "red"}


def show_result(res):
    UI.say(paint("RESULT: ", "bold") + paint(str(res), _STATUS_STYLE.get(getattr(res, "status", ""), "white"), "bold"))


def ask_approval(con):
    """Two buttons switched with Left/Right. The focus starts on CANCEL, so APPROVE always takes a deliberate key press.
    Without a real terminal (pipes) the old rule applies: type APPROVE exactly."""
    timeout = L.APPROVAL_TIMEOUT_SECONDS
    UI.say()
    if con.interactive:
        UI.say(paint(f"Read the plan, the code and the validator result above, then choose "
                     f"({UI.SYM['arrows']} switch, Enter confirm, Esc cancel):", "bold"))
        idx = con.select([("APPROVE", "bg_green"), ("CANCEL", "bg_red")], default=1, timeout=timeout)
        return idx == 0
    ans = con.read_line(f"Type APPROVE (exactly) within {timeout:.0f} s to run this program, anything else cancels: ", timeout)
    return (ans or "").strip() == "APPROVE"


# ---------------------------------------------------------------------------------------------
# Approval + execution (shared by AI tasks and built-in commands)
# ---------------------------------------------------------------------------------------------
def present_and_execute(robot, con, cand):
    show_candidate(cand)
    problems = robot.readiness_problems(robot.get_state(), motion=cand.validation.has_motion)
    if not L.get_live():
        note("LIVE execution is OFF (/settings). Nothing will be sent.", "yellow")
        return
    if problems:
        UI.say(paint("Execution is BLOCKED right now:\n  - " + "\n  - ".join(problems), "red", "bold"))
        return
    if not ask_approval(con) or robot.stop_requested.is_set():
        note("Not approved. Nothing was sent.", "yellow")
        return
    if L.fingerprint_material() != cand.fingerprint_data["cfg"]:
        UI.say(paint("Settings/limits/LIVE changed since validation - approval invalidated. Ask again.", "red"))
        return
    robot.stop_requested.clear()
    out(A.banner("PRE-EXECUTION CHECK"))
    out("  refreshing live robot state and re-validating the FULL approved program ...")
    fresh_state = robot.get_state()
    fresh = R.validate_body(cand.validation.body_lines, fresh_state)
    if not fresh.ok or fresh.sha256 != cand.fingerprint_data["sha"]:
        out(A.render_validator(fresh) if not fresh.ok else "  code hash differs from what you approved")
        UI.say(paint("  NOT executed. Ask again so the task is re-planned and re-approved.", "red", "bold"))
        return
    UI.say(paint("  PASS (state fresh, program unchanged)", "green", "bold"))

    fname = get_approval_filepath(prefix="approved_code")
    with open(fname, "w", encoding="utf-8") as f:
        f.write(f"# Task: {cand.task}\n\n")
        f.write("## Plan:\n```json\n" + json.dumps(cand.plan, indent=2) + "\n```\n\n")
        f.write("## Code per step:\n```python\n" + A.render_codes(cand.steps, cand.step_codes) + "\n```\n\n")
        f.write("## Validated instruction lines:\n```urscript\n" + "\n".join(cand.program_lines) + "\n```\n\n")
        f.write("## Exact URScript sent to the robot (token shown as TOKEN):\n```urscript\n"
                + R.script_preview(cand.validation.body_lines, cand.validation.has_motion,
                                   cand.validation.has_gripper) + "```\n")
    note(f"  approved code saved to: {fname}")
    out(A.banner("EXECUTING", "press s at any time to STOP"))
    pipe = UI.Pipeline(con, details=False)
    with pipe.running("Executing"):
        res = robot.execute(cand.validation, cand.fingerprint_data["sha"])
    show_result(res)


def fail_candidate(cand, pipe):
    path = A.write_failure_report(cand)
    if not pipe.verbose:
        pipe.dump("what happened")
    out(A.banner("FAILED"))
    UI.say(paint(f"  {cand.fail_reason}", "red") + f"\n  NOTHING will run. Report written: {path}")


# ---------------------------------------------------------------------------------------------
# Tasks
# ---------------------------------------------------------------------------------------------
def run_task(robot, agent, con, text):
    robot.stop_requested.clear()
    agent.clear()                                   # every task starts with a fresh, small context
    pipe = UI.Pipeline(con)
    _LAST["pipe"] = pipe
    t0 = time.monotonic()
    try:
        out(A.banner("TASK"))
        UI.say("  " + text)
        with pipe.running("Understanding the task"):
            kind, obj = agent.submit(text, log=pipe)
        while kind == "clarify":
            UI.say()
            UI.say(paint("AI question: ", "yellow", "bold") + obj)
            ans = con.read_line(paint("  your answer (empty = cancel) ", "yellow") + UI.SYM["prompt"] + " ",
                                L.APPROVAL_TIMEOUT_SECONDS)
            if not ans or ans.strip().lower() == "/clear":
                agent.clear()
                note("Cancelled.", "yellow")
                return
            with pipe.running("Thinking"):
                kind, obj = agent.submit(ans, log=pipe)
        if kind == "unsupported":
            out(A.banner("UNSUPPORTED"))
            UI.say(paint(f"  {obj}", "yellow") + "\n  Nothing will run. Rephrase the task using the supported primitives (/help).")
            return
        with pipe.running("Planning"):
            cand = agent.build_candidate(text, obj, progress=pipe)
    except A.Aborted:
        note("Task aborted because STOP was pressed. Nothing was sent.", "yellow")
        agent.clear()
        return
    except A.LLMError as e:
        path = A.write_diagnostic_report(text, str(e), agent)
        if not pipe.verbose:
            pipe.dump("what happened")
        UI.say(paint(f"\nAI error: {e}", "red", "bold") + f"\nNothing was sent to the robot. Diagnostic report: {path}")
        return
    if not cand.ok:
        fail_candidate(cand, pipe)
        return
    UI.say(paint(f"\n{UI.SYM['ok']} AI pipeline finished in {time.monotonic() - t0:.0f} s", "green")
           + paint("   (/log shows the process log)" if not pipe.verbose else "", "gray"))
    present_and_execute(robot, con, cand)


def cmd_robot(robot, agent, con, rest):
    """/robot [speed] [stay]: corner tour built by software (tour.py), checked by the same validator, then approved."""
    speed, stay, follow = None, False, True
    for tok in rest.split():
        if tok.lower() in ("stay", "nohome"):
            stay = True
        elif tok.lower() == "fixed":
            follow = False
        else:
            try:
                speed = L.parse_limit_value("max_speed", tok)
            except ValueError as e:
                UI.say(paint(f"Usage: /robot [speed, e.g. 0.05 or 50mm/s] [stay] [fixed]   ({e})", "yellow"))
                return
    robot.stop_requested.clear()
    state = robot.get_state()
    problems = robot.readiness_problems(state)
    if problems:
        UI.say(paint("Cannot start the tour - movement is BLOCKED:\n  - " + "\n  - ".join(problems), "red", "bold"))
        return
    task = "/robot corner tour" + (f" at {speed:g} m/s" if speed else " at the maximum allowed speed") + (", stay at the end" if stay else "") + ("" if follow else ", tool orientation fixed")
    agent.begin_task(task)                            # saves the 'start' pose the tour returns to
    pipe = UI.Pipeline(con)
    _LAST["pipe"] = pipe
    try:
        out(A.banner("TASK"))
        UI.say("  " + task)
        with pipe.running("Checking reachability"):
            plan, notes = T.build_tour_plan(state, speed, stay, should_stop=robot.stop_requested.is_set, log=pipe, follow_yaw=follow)
        with pipe.running("Validating"):
            cand = agent.build_candidate_fixed(task, plan, "Built-in /robot command: no AI involved, the validator "
                                               "checked every segment.", progress=pipe)
    except T.TourError as e:
        UI.say(paint(f"Tour not possible: {e}", "red", "bold") + "\nNothing was sent.")
        return
    except T.Cancelled:
        note("Tour check stopped because STOP was pressed. Nothing was sent.", "yellow")
        return
    UI.say()
    for i, n in enumerate(notes):
        UI.say(paint(("  " if i else "") + n, *(("white", "bold") if i == 0 else ("yellow",))))
    if not cand.ok:
        fail_candidate(cand, pipe)
        return
    present_and_execute(robot, con, cand)


def do_home(robot, con):
    robot.stop_requested.clear()
    note("Homing: validating path to HOME_POSE ...")
    pipe = UI.Pipeline(con, details=False)
    with pipe.running("Homing"):
        res = robot.home()
    show_result(res)


def startup(robot, dry, con):
    check_python()
    bad = L.check_static_config()
    if bad:
        raise SystemExit("Configuration errors in limits.py:\n  - " + "\n  - ".join(bad))
    if not L.REACH_CHECK_ENABLED:
        UI.say(UI.rule("WARNING", "red"))
        UI.say(paint("  " + WARN_REACH_OFF, "red", "bold"))
        UI.say(paint("  Plans can leave the arm's comfortable reach and the robot may stop with a fault. Set "
                     "REACH_CHECK_ENABLED = True in limits.py to switch the check back on.", "red"))
        UI.say(UI.rule("", "red"))
    try:
        UI.say(paint(UI.SYM["ok"] + " ", "green") + A.check_ollama())
    except A.LLMError as e:
        raise SystemExit(f"ERROR: {e}")
    try:
        UI.say(paint(UI.SYM["ok"] + " ", "green") + robot.connect())
    except ConnectionError as e:
        UI.say(paint(f"ROBOT NOT CONNECTED: {e}", "red", "bold") + "\nMovement is blocked. (No automatic fallback to simulation.)")
        return
    t0 = time.monotonic()
    while robot.get_state() is None and time.monotonic() - t0 < 5:
        time.sleep(0.1)
    robot.capture_session_start()           # remembered BEFORE homing: pose_ref "session_start"
    for c in R.check_capabilities():
        UI.say(paint("CONFIG PROBLEM: " + c, "red"))
    show_status(robot)
    problems = robot.readiness_problems(robot.get_state())
    if problems:
        UI.say(paint("\nStartup homing SKIPPED - movement blocked:\n  - " + "\n  - ".join(problems), "yellow", "bold"))
    else:
        do_home(robot, con)


# ---------------------------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------------------------
def handle_line(robot, agent, con, line):
    cmd, _, rest = line.partition(" ")
    low = ALIASES.get(cmd.lower(), cmd.lower())
    if low == "/help":
        show_help()
    elif low == "/status":
        show_status(robot)
    elif low == "/settings":
        a = rest.lower().split()
        if len(a) == 2 and a[0] == "live" and a[1] in ("on", "off"):
            L.set_live(a[1] == "on")
            UI.say(f"LIVE execution is now " + paint(a[1].upper(), "green" if a[1] == "on" else "yellow", "bold")
                   + " (pending approvals are invalid; /stop still works).")
        UI.say(f"LIVE execution: {'ON' if L.get_live() else 'OFF'} | model {L.OLLAMA_MODEL} | robot {L.ROBOT_IP}:{L.URSCRIPT_PORT}"
               f" | gripper {'on' if L.GRIPPER_ENABLED else 'off'}\nChange: /settings live on|off")
    elif low == "/limit":
        parts = rest.split(None, 1)
        if not parts:
            show_limits()
        elif len(parts) == 2:
            ok, msg = L.set_limit(parts[0], parts[1])
            UI.say(paint(msg, "green" if ok and "BLOCKED" not in msg else "yellow" if ok else "red"))
        else:
            UI.say(paint("Usage: /limit <name> <value>   e.g. /limit x_min 0.30", "yellow"))
    elif low == "/home":
        do_home(robot, con)
    elif low == "/robot":
        cmd_robot(robot, agent, con, rest)
    elif low == "/clear":
        agent.clear()
        note("Task context cleared (settings, limits, connection untouched).", "green")
    elif low == "/cls":
        UI.clear_screen()
        show_header(robot, robot.dry_run)
    elif low == "/log":
        pipe = _LAST["pipe"]
        if pipe is None or not pipe.lines:
            note("No process log yet: run a task first.", "yellow")
        else:
            pipe.dump("process log of the last task")
    elif low == "/ai":
        if not rest.strip():
            UI.say(paint("Usage: /ai <task>", "yellow"))
        else:
            submit_task(robot, agent, con, rest.strip())
    elif line.startswith("/"):
        UI.say(paint("Unknown command. Type /help.", "yellow"))
    else:
        submit_task(robot, agent, con, line)


def submit_task(robot, agent, con, text):
    if text.strip().lower() in BARE_COMMANDS:
        UI.say(paint(f"'{text.strip()}' looks like a command. Did you mean /{text.strip().lower()}?", "yellow"))
        return
    problem = task_text_problem(text)
    if problem:
        UI.say(paint(problem, "yellow"))
        return
    run_task(robot, agent, con, text)


def main():
    global _CON
    dry = "--dry-run" in sys.argv[1:]
    robot = R.Robot(dry_run=dry)
    agent = A.Agent(robot)
    con = _CON = UI.Console(on_stop=make_stop(robot), stop_words=STOP_WORDS, completions=COMPLETIONS)
    con.start()
    show_header(robot, dry)
    startup(robot, dry, con)
    UI.say(paint("\nType /help for commands. Stop: s / stop / /stop", "gray"))
    while True:
        UI.say()
        line = con.read_line(make_prompt(robot))
        if line is None or line.strip() == "/exit":
            st = robot.get_state()
            note("Closing. The robot was NOT commanded to stop or move." + ("" if st else " Its current state is unknown."), "yellow")
            robot.close()
            return
        line = line.strip()
        if not line:
            continue
        try:
            handle_line(robot, agent, con, line)
        except Exception as e:          # never crash the loop; never act on an error
            UI.say(paint(f"Unexpected error: {type(e).__name__}: {e}. Nothing was sent.", "red", "bold"))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        UI.say("\nInterrupted. The robot was not commanded.")
    finally:
        if _CON is not None:
            _CON.restore()
