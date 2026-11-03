"""ai_agent.py -- local Ollama calls, planning, step code, independent review, LOCAL repair loop, reports.

Model output is DATA: it is parsed and validated, never executed. Pipeline per task:
  understand -> live state -> plan (JSON) -> resolve numbers (software) -> code per step -> FULL program
  -> independent review -> deterministic validation -> repair ONLY the failing step -> re-assemble FULL program
  -> re-review -> re-validate (max MAX_REPAIR_ATTEMPTS)
"""
import copy
import datetime
import json
import math
import os
import time
import urllib.error
import urllib.request

import limits as L
import prompts as P
import robot_sender as R


class LLMError(Exception):
    pass


class EmptyReply(LLMError):
    """The model returned no text. NOT a robot error; retried with a short focused prompt."""


class Aborted(Exception):
    """Operator pressed stop while the AI pipeline was running."""


# ---------------------------------------------------------------------------------------------
# Terminal rendering (one place, so every stage looks the same)
# ---------------------------------------------------------------------------------------------
def banner(tag, extra=""):
    head = f"==== [{tag}]" + (f" {extra}" if extra else "") + " "
    return "\n" + head + "=" * max(4, 72 - len(head))


def render_plan(plan, steps):
    out = [f"  Understanding: {plan.get('interpretation', '')}",
           f"  Primitives:    {', '.join(str(x) for x in plan.get('operations', []) or []) or '-'}"]
    for a in plan.get("assumptions", []) or []:
        out.append(f"  Assumption:    {a}")
    for i, s in enumerate(plan.get("steps", []) or [], 1):
        if not isinstance(s, dict):
            out.append(f"  {i}. (invalid step)")
            continue
        if s.get("action") == "repeat":
            p = s.get("params") if isinstance(s.get("params"), dict) else {}
            out.append(f"  {i}. {s.get('name', '')}  [repeat x{p.get('count')}]  {s.get('description', '')}")
            for j, b in enumerate(p.get("body") or [], 1):
                if isinstance(b, dict):
                    out.append(f"       {i}.{j} {b.get('name', '')}  [{b.get('action')}]  {b.get('description', '')}")
        else:
            out.append(f"  {i}. {s.get('name', '')}  [{s.get('action')}]  {s.get('description', '')}")
    out.append(f"  -> {len(steps)} executable step(s) after software resolution")
    return "\n".join(out)


def render_adjustments(steps):
    """Plain-language block listing every target the software moved because of the arm's reach. Empty string if none."""
    rows = [st for st in steps if st.get("adjusted") and st.get("note")]
    if not rows:
        return ""
    out = ["ADJUSTED: the arm cannot comfortably reach everything that was asked, so these steps go as far as it can.",
           "          This is NOT the workspace limit. The robot finds it hard to reach, so it ADJUSTS and never goes to the maximum."]
    for st in rows:
        out.append(f"  Step {st['id']}: {st['name']}\n    {st['note']}")
    return "\n".join(out)


def render_codes(steps, codes, only=None):
    out = []
    for st in steps:
        if only is not None and st["id"] not in only:
            continue
        note = f"\n    NOTE: {st['note']}" if st.get("note") else ""
        out.append(f"  Step {st['id']}: {st['name']}\n    {codes.get(st['id'], '(no code)')}{note}")
    return "\n".join(out) or "  (none)"


def render_program(lines):
    return "\n".join(f"  {i:2d}  {ln}" for i, ln in enumerate(lines, 1)) or "  (empty)"


def render_validator(res):
    if res.ok:
        return "  PASS\n" + "\n".join("   " + n for n in res.numeric_notes) + \
               f"\n   estimated duration ~{res.estimated_seconds:.1f} s, SHA-256 {res.sha256[:16]}..."
    return "  FAIL\n" + "\n\n".join("  " + f.detailed().replace("\n", "\n  ") for f in res.findings)


def render_review(rv):
    if not rv:
        return "  (not run: the validator failed first)"
    out = [f"  {'APPROVED' if rv['approved'] else 'NOT APPROVED'} - {rv['summary']}"]
    out += [f"   [{f['severity']}] {f['message']}" + (f" (step {f['step']})" if f.get("step") else "") for f in rv["findings"]]
    return "\n".join(out)


# ---------------------------------------------------------------------------------------------
# Ollama (exact model tag only)
# ---------------------------------------------------------------------------------------------
def _http(path, payload=None, timeout=30.0):
    url = L.OLLAMA_URL.rstrip("/") + path
    data = None if payload is None else json.dumps(payload).encode()
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"},
                                 method="POST" if data else "GET")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8"))
    except (urllib.error.URLError, OSError, ValueError) as e:
        raise LLMError(f"Ollama request to {url} failed: {e}")


def check_ollama():
    """Reachable AND exact tag installed. Never pulls or substitutes a model."""
    tags = _http("/api/tags", timeout=L.ROBOT_CONNECT_TIMEOUT_SECONDS + 5)
    names = {m.get("name") for m in tags.get("models", [])} | {m.get("model") for m in tags.get("models", [])}
    if L.OLLAMA_MODEL not in names:
        raise LLMError(f"Model '{L.OLLAMA_MODEL}' is not installed in Ollama (found: {sorted(n for n in names if n)}). "
                       f"Install it yourself with: ollama pull {L.OLLAMA_MODEL}   (nothing is pulled or substituted automatically)")
    return f"Ollama OK, exact model '{L.OLLAMA_MODEL}' is installed."


def chat(system, messages, think="default"):
    payload = {"model": L.OLLAMA_MODEL, "stream": False,
               "messages": [{"role": "system", "content": system}] + messages,
               "options": {"temperature": L.LLM_TEMPERATURE, "num_ctx": L.LLM_NUM_CTX,
                           "num_predict": L.LLM_MAX_OUTPUT_TOKENS}}
    level = L.LLM_THINK if think == "default" else think
    if level:
        payload["think"] = level
    out = _http("/api/chat", payload, timeout=L.LLM_TIMEOUT_SECONDS)
    if out.get("model") not in (None, L.OLLAMA_MODEL):
        raise LLMError(f"Ollama answered with model '{out.get('model')}', not '{L.OLLAMA_MODEL}'. Rejected.")
    msg = out.get("message") or {}
    content = msg.get("content")
    if not isinstance(content, str) or not content.strip():
        hint = " (the model produced only hidden reasoning; its token budget may have been used up)" if msg.get("thinking") else ""
        raise EmptyReply("Empty reply from the model" + hint + ".")
    return content


def extract_json(text):
    """First JSON object in the text (tolerates ``` fences / stray prose). Raises ValueError."""
    dec = json.JSONDecoder()
    for i, ch in enumerate(text):
        if ch == "{":
            try:
                obj, _ = dec.raw_decode(text[i:])
            except ValueError:
                continue
            if isinstance(obj, dict):
                return obj
    raise ValueError("no JSON object found")


def ask_json(system, messages, check, stop_event=None, short=None, log=print, label="model"):
    """Call the model until `check(obj)` returns None (valid).
    - invalid JSON/schema -> re-ask with the problem named (max LLM_JSON_RETRIES), conversation kept;
    - EMPTY reply -> NOT fatal: retry with a short focused prompt (`short()` -> (system, messages)) and low
      reasoning effort, max LLM_EMPTY_RETRIES; then a clean EmptyReply error (nothing is ever executed)."""
    cur_system, cur_messages, think = system, messages, "default"
    json_fail, empty, last = 0, 0, "unknown"
    while True:
        if stop_event is not None and stop_event.is_set():
            raise Aborted()
        t0 = time.monotonic()
        log(f"  [LLM] {label} ...")
        try:
            reply = chat(cur_system, cur_messages, think)
        except EmptyReply as e:
            empty += 1
            log(f"  [LLM] {label}: EMPTY reply after {time.monotonic() - t0:.1f} s. Nothing was sent to the robot.")
            if empty > L.LLM_EMPTY_RETRIES:
                raise EmptyReply(f"{label}: the model returned an empty reply {empty} times "
                                 f"(original + {L.LLM_EMPTY_RETRIES} short retries). Nothing was sent to the robot.")
            log(f"  [LLM] {label}: short focused retry {empty}/{L.LLM_EMPTY_RETRIES} (smaller prompt, low reasoning effort)")
            if short is not None:
                cur_system, cur_messages = short()
            think = L.LLM_RETRY_THINK if L.LLM_RETRY_THINK else "default"
            continue
        log(f"  [LLM] {label}: reply in {time.monotonic() - t0:.1f} s")
        try:
            obj = extract_json(reply)
            problem = check(obj)
        except ValueError as e:
            obj, problem = None, f"not valid JSON ({e})"
        if problem is None:
            messages.append({"role": "assistant", "content": reply})   # the real conversation records the good reply
            return obj
        cur_messages.append({"role": "assistant", "content": reply})
        json_fail += 1
        last = problem
        if json_fail > L.LLM_JSON_RETRIES:
            raise LLMError(f"{label}: no valid reply after {json_fail} tries ({last}). Nothing was sent to the robot.")
        log(f"  [LLM] {label}: reply rejected ({problem}); asking again")
        cur_messages.append({"role": "user", "content": f"Your reply was rejected: {problem}. "
                             "Reply again with ONE valid JSON object in the required format."})


# ---------------------------------------------------------------------------------------------
# Schema checks (strict; anything else = rejection)
# ---------------------------------------------------------------------------------------------
def _check_planner(o):
    a = o.get("action")
    if a == "tool":
        return None if o.get("tool") in ("get_robot_state", "get_limits") else "unknown tool (only get_robot_state, get_limits)"
    if a == "clarify":
        return None if isinstance(o.get("question"), str) and o["question"].strip() else "clarify needs a question string"
    if a == "unsupported":
        return None if isinstance(o.get("reason"), str) and o["reason"].strip() else "unsupported needs a reason string"
    if a == "plan":
        if not isinstance(o.get("interpretation"), str):
            return "plan needs 'interpretation' (string)"
        if not isinstance(o.get("operations"), list) or not all(isinstance(x, str) for x in o["operations"]):
            return "plan needs 'operations' (list of primitive names)"
        if not isinstance(o.get("assumptions"), list) or not all(isinstance(x, str) for x in o["assumptions"]):
            return "plan needs 'assumptions' (list of strings)"
        if not isinstance(o.get("steps"), list):
            return "plan needs 'steps' (list)"
        sc = o.get("self_check")
        keys = ("covers_all_requested_actions", "end_matches_request", "no_extra_steps")
        if not isinstance(sc, dict) or not all(isinstance(sc.get(k), bool) for k in keys):
            return "plan needs 'self_check' with boolean covers_all_requested_actions, end_matches_request, no_extra_steps"
        if not all(sc[k] for k in keys):
            return f"your own self_check found a problem ({sc}); fix the plan so that every check is true"
        return None
    return "'action' must be tool, clarify, unsupported or plan"


def _check_step_repair(o):
    s = o.get("step")
    if not isinstance(s, dict) or not isinstance(s.get("action"), str) or not isinstance(s.get("params"), dict):
        return 'needs {"step": {"name","action","description","params"}}'
    return None


def _check_codegen(o):
    if not isinstance(o.get("code"), str) or not isinstance(o.get("explanation"), str):
        return "needs string fields 'code' and 'explanation'"
    return None


def _check_review(o):
    if not isinstance(o.get("approved"), bool) or not isinstance(o.get("summary"), str):
        return "needs boolean 'approved' and string 'summary'"
    f = o.get("findings")
    if not isinstance(f, list):
        return "'findings' must be a list"
    for x in f:
        if not (isinstance(x, dict) and x.get("severity") in ("error", "warning", "info") and isinstance(x.get("message"), str)):
            return "each finding needs severity (error|warning|info) and message"
        if "kind" in x and not isinstance(x["kind"], str):
            return "finding 'kind' must be a string"
    return None


# ---------------------------------------------------------------------------------------------
# Candidate = everything shown to the user before approval
# ---------------------------------------------------------------------------------------------
class Candidate:
    def __init__(self, task):
        self.task = task
        self.plan = None
        self.steps = []
        self.step_codes, self.explanations = {}, {}
        self.validation = None
        self.review = None
        self.attempts = []          # for the failure report
        self.ok = False
        self.state_text = ""
        self.fingerprint_data = None
        self.refs = {}
        self.fail_reason = ""
        self.repaired = []          # plan-step paths that were repaired locally

    @property
    def program_lines(self):
        return self.validation.body_lines if self.validation else []


def _norm_step(step):
    """Identity of a step for 'is this the same answer again?' (name/description ignored)."""
    return json.dumps({"a": step.get("action"), "p": step.get("params")}, sort_keys=True, default=str)


def _get_step(plan, path):
    s = plan["steps"][path[0] - 1]
    return s if len(path) == 1 else s["params"]["body"][path[1] - 1]


def _set_step(plan, path, new):
    if len(path) == 1:
        plan["steps"][path[0] - 1] = new
    else:
        plan["steps"][path[0] - 1]["params"]["body"][path[1] - 1] = new


def _path_text(path):
    return ".".join(str(x) for x in path)


class Agent:
    def __init__(self, robot):
        self.robot = robot
        self.history = []           # planner conversation, ONE task only
        self.task = None
        self.qa = []                # (question, answer) pairs given during this task
        self.pending_question = None
        self.refs = {}
        self.stop_event = robot.stop_requested

    def clear(self):
        self.history, self.task, self.qa, self.pending_question, self.refs = [], None, [], None, {}

    def task_context(self):
        """The task as ONE text: the request plus every answer the user gave. Planner, step repair, reviewer
        and reports all use this, so an answer like 'yes, 6 cm is fine' is never lost."""
        base = self.task or ""
        if not self.qa:
            return base
        lines = [base, "", "Clarifications given by the user (binding; they override earlier assumptions):"]
        lines += [f"- Question: {q}\n  User answer: {a}" for q, a in self.qa]
        return "\n".join(lines)

    # --- tools (read only, registered) ---
    def _tool(self, name):
        if name == "get_limits":
            return P.limits_text()
        return P.state_summary(self.robot.get_state(), self.refs)

    # --- planning ---
    def begin_task(self, text):
        """Start a NEW task: fresh context, capture the saved poses from the live robot."""
        self.clear()
        st = self.robot.get_state()
        self.task = text
        if st is not None and st.pose is not None:
            self.refs["start"] = list(st.pose)
        if self.robot.session_start_pose:
            self.refs["session_start"] = list(self.robot.session_start_pose)

    def submit(self, user_text, log=print):
        """One planner exchange. Returns ('clarify', q) | ('unsupported', why) | ('plan', plan_dict)."""
        state = self.robot.get_state()
        if self.task is None:
            self.begin_task(user_text)
            msg = P.planner_first_message(user_text, state, self.refs)
        else:
            if self.pending_question is not None:
                self.qa.append((self.pending_question, user_text))
                self.pending_question = None
            msg = (f"User answer to your last question: {user_text}\n\n"
                   f"Whole task with all answers so far:\n{self.task_context()}\n\n{P.state_summary(state, self.refs)}")
        self.history.append({"role": "user", "content": msg})
        kind, obj = self._planner_loop(log)
        if kind == "plan":
            # Pre-check BEFORE building anything: a move that does not fit the workspace is a question for the user.
            problems = R.fit_problems(obj, self.robot.get_state(), self.refs)
            if problems:
                first = problems[0]
                head = first.split(". Ask the user")[0]
                if "nothing to move" in first:
                    q = head + ". Choose another direction, or cancel?"
                elif "saved pose" in head:
                    q = head + ". Rotate the base back (base_rotate through the same side), choose another target, or cancel?"
                elif "cannot comfortably move" in head:
                    q = head + ". Choose a nearer or lower point, or cancel?"
                elif "base column" in head:
                    q = head + ". Choose a target that does not pass next to the base, or cancel?"
                elif "comfortable reach" in first:
                    q = (head + ". Shapes, arcs, returns to a saved pose and joint moves cannot be adjusted automatically. "
                         "Choose a smaller, nearer or lower target, or cancel?")
                else:
                    q = head + ". Shorten the move to what fits (stop at the edge), or cancel?"
                self.pending_question = q
                return "clarify", q
        return kind, obj

    def _planner_short(self):
        return (P.planner_system_prompt(compact=True),
                [{"role": "user", "content": P.planner_short_message(self.task_context(), [], self.robot.get_state(), self.refs)}])

    def _planner_loop(self, log=print):
        for _ in range(L.PLANNER_MAX_TOOL_CALLS + 1):
            obj = ask_json(P.planner_system_prompt(), self.history, _check_planner, self.stop_event,
                           short=self._planner_short, log=log, label="planner")
            if obj["action"] == "tool":
                self.history.append({"role": "user", "content": f"TOOL RESULT ({obj['tool']}):\n{self._tool(obj['tool'])}"})
                continue
            if obj["action"] == "clarify":
                self.pending_question = obj["question"].strip()
                return "clarify", obj["question"].strip()
            if obj["action"] == "unsupported":
                return "unsupported", obj["reason"].strip()
            return "plan", obj
        raise LLMError("The planner used its tool-call budget without producing a plan.")

    # --- step code ---
    def _gen_code(self, step, total, repair=None, same=False, log=print):
        """CODEGEN_MODE 'template': software renders the line. 'llm': the model writes it from exact templates."""
        if L.CODEGEN_MODE == "template":
            return R.expected_line(step), "rendered from the plan step by the exact template"
        msgs = [{"role": "user", "content": P.codegen_user_prompt(step, total, repair, same)}]
        sysmsg = P.codegen_system_prompt()
        o = ask_json(sysmsg, msgs, _check_codegen, self.stop_event, short=lambda: (sysmsg, list(msgs[:1])),
                     log=log, label=f"code for step {step['id']}")
        return o["code"].strip(), o["explanation"].strip()

    # --- local repair of ONE plan step (small prompt) ---
    def _repair_plan_step(self, plan, path, finding_texts, failed, state, log):
        raw = _get_step(plan, path)
        failed.setdefault(path, [])
        if _norm_step(raw) not in failed[path]:
            failed[path].append(_norm_step(raw))
        top = plan["steps"]
        nb = [f"{i}. {s.get('name')} [{s.get('action')}]" for i, s in enumerate(top, 1)
              if isinstance(s, dict) and abs(i - path[0]) == 1]
        for attempt in range(1 + 2):
            msgs = [{"role": "user", "content": P.step_repair_user(
                self.task_context(), _path_text(path), raw, "\n\n".join(finding_texts),
                [json.loads(x) for x in failed[path]], "; ".join(nb) or "-", state, self.refs, same_answer=attempt > 0)}]
            sysmsg = P.step_repair_system()
            o = ask_json(sysmsg, msgs, _check_step_repair, self.stop_event, short=lambda: (sysmsg, list(msgs[:1])),
                         log=log, label=f"repair of plan step {_path_text(path)}")
            new = o["step"]
            if _norm_step(new) in failed[path]:
                log(f"  [REPAIR] the model returned the SAME failed step again; telling it to change the implementation "
                    f"({attempt + 1}/3)")
                continue
            new.setdefault("name", raw.get("name", ""))
            new.setdefault("description", raw.get("description", ""))
            _set_step(plan, path, new)
            return True
        return False

    # --- independent review of the FULL assembled program ---
    def _review(self, cand, state, repaired, log):
        res = cand.validation
        user = P.reviewer_user_prompt(
            cand.task, {"interpretation": cand.plan.get("interpretation"), "assumptions": cand.plan.get("assumptions"),
                        "operations": cand.plan.get("operations"), "plan_as_written": cand.plan.get("steps"),
                        "resolved_steps": cand.steps},
            {str(k): v for k, v in cand.step_codes.items()}, "\n".join(res.body_lines), res.summary(),
            P.state_summary(state, cand.refs), L.get_limits(), cand.refs, repaired, facts=_facts(cand))
        msgs = [{"role": "user", "content": user}]
        compact = [{"role": "user", "content": P.reviewer_user_prompt(
            cand.task, {"interpretation": cand.plan.get("interpretation"), "plan_as_written": cand.plan.get("steps")},
            {}, "\n".join(res.body_lines), res.summary(),
            "", {}, compact=True)}]
        return ask_json(P.REVIEWER_SYSTEM_PROMPT, msgs, _check_review, self.stop_event,
                        short=lambda: (P.REVIEWER_SYSTEM_PROMPT, compact), log=log, label="independent review")

    # --- full pipeline with bounded LOCAL repair ---
    def build_candidate(self, task, plan, progress=print):
        cand = Candidate(self.task_context())
        cand.refs = dict(self.refs)
        raw_plan = copy.deepcopy(plan)
        codes, sigs, failed_plan, failed_code = {}, {}, {}, {}
        code_errors = {}                         # step id -> detailed text (llm mode only)
        repaired_paths = []
        try:
            for attempt in range(L.MAX_REPAIR_ATTEMPTS + 1):
                if self.stop_event.is_set():
                    raise Aborted()
                label = "initial generation" if attempt == 0 else f"repair attempt {attempt}/{L.MAX_REPAIR_ATTEMPTS}"
                progress(banner("PLAN", label))
                state = self.robot.get_state()
                steps, plan_findings = R.resolve_plan(raw_plan, state, refs=self.refs)
                cand.plan, cand.steps = raw_plan, steps
                progress(render_plan(raw_plan, steps))
                _adj = render_adjustments(steps)
                if _adj:
                    progress(_adj)
                # --- code per step: only (re)generate steps whose numbers changed or that were rejected
                progress(banner("STEP CODE"))
                regenerated = []
                for st in steps:
                    sig = json.dumps([st["action"], st["params"]], sort_keys=True)
                    if st["id"] in codes and sigs.get(st["id"]) == sig and st["id"] not in code_errors:
                        continue
                    repair, same = None, False
                    if st["id"] in code_errors:
                        repair = {"previous_code": codes.get(st["id"], ""), "detail": code_errors[st["id"]]}
                    code, expl = self._gen_code(st, len(steps), repair, False, progress)
                    if repair and code == failed_code.get(st["id"]):
                        progress(f"  [REPAIR] step {st['id']}: same failed code returned again; re-asking with an explicit warning")
                        code, expl = self._gen_code(st, len(steps), repair, True, progress)
                    codes[st["id"]], sigs[st["id"]] = code, sig
                    cand.explanations[st["id"]] = expl
                    regenerated.append(st["id"])
                code_errors = {}
                cand.step_codes = {st["id"]: codes.get(st["id"], "") for st in steps}
                progress(render_codes(steps, cand.step_codes) + ("" if attempt == 0 else
                         f"\n  (regenerated: {regenerated or 'none - all step code unchanged'})"))
                # --- FULL program, FULL validation
                res, _meta = R.validate_candidate(steps, cand.step_codes, state, plan_findings)
                cand.validation = res
                cand.state_text = P.state_summary(state, self.refs)
                progress(banner("FULL PROGRAM"))
                progress(render_program(res.body_lines))
                progress(banner("VALIDATOR" if attempt == 0 else "REVALIDATION", "full program"))
                progress(render_validator(res))
                findings = list(res.findings)
                review = None
                if res.ok:
                    progress(banner("AI REVIEW", "fresh context, full program"))
                    review = self._review(cand, state, [_path_text(p) for p in repaired_paths], progress)
                    cand.review = review
                    progress(render_review(review))
                    errs = _blocking_reviewer_errors(review, _facts(cand), progress)
                    # approved=false only counts if a real (non-overruled) error stands behind it
                    if not errs and not review["approved"] and any(f["severity"] == "error" for f in review["findings"]):
                        progress("  [REVIEW] reviewer said not approved, but every error it raised was overruled by the "
                                 "deterministic facts: accepted")
                    if not errs:
                        cand.ok = True
                        cand.attempts.append({"label": label, "plan": copy.deepcopy(raw_plan), "steps": copy.deepcopy(steps),
                                              "codes": dict(cand.step_codes), "findings": [], "review": review})
                        cand.repaired = repaired_paths
                        cand.fingerprint_data = {"plan": cand.plan, "steps": cand.steps, "codes": cand.step_codes,
                                                 "sha": res.sha256, "cfg": L.fingerprint_material(),
                                                 "pose": state.pose if state else None}
                        return cand
                    # reviewer objections become findings: step-bound -> local repair, otherwise whole-plan
                    byid = {s["id"]: s for s in steps}
                    for f in errs or [{"message": f"Reviewer did not approve: {review['summary']}", "step": None}]:
                        st = byid.get(f.get("step"))
                        findings.append(R.Finding("Reviewer: " + f["message"], step=st["id"] if st else None,
                                                  kind="plan" if st else "global",
                                                  step_name=st["name"] if st else None, src=st["src"] if st else None,
                                                  instruction=cand.step_codes.get(st["id"]) if st else None))
                cand.attempts.append({"label": label, "plan": copy.deepcopy(raw_plan), "steps": copy.deepcopy(steps),
                                      "codes": dict(cand.step_codes), "findings": [f.detailed() for f in findings],
                                      "review": review})
                # ---------------- decide what to repair ----------------
                fatal = [f for f in findings if f.kind == "fatal" or f.param in ("fit", "reach")]
                if fatal:
                    cand.fail_reason = ("Not repairable by the AI (configuration, robot state, or the arm cannot reach it): "
                                        + " | ".join(f.message for f in fatal))
                    return self._finish_fail(cand)
                if attempt == L.MAX_REPAIR_ATTEMPTS:
                    break
                glob = [f for f in findings if f.kind == "global" or (f.kind == "plan" and not f.src)]
                code_f = [f for f in findings if f.kind == "code"]
                plan_f = [f for f in findings if f.kind == "plan" and f.src]
                if code_f and L.CODEGEN_MODE == "template":
                    cand.fail_reason = ("INTERNAL: rendered code does not match its plan step: "
                                        + " | ".join(f.message for f in code_f))
                    return self._finish_fail(cand)
                if code_f:                                              # repair ONLY those steps' code
                    ids = sorted({f.step for f in code_f if f.step})
                    progress(banner("REPAIR", "code of step(s) " + ", ".join(map(str, ids)) + " only"))
                    for f in code_f:
                        progress("  " + f.detailed().replace("\n", "\n  "))
                    for i in ids:
                        failed_code[i] = codes.get(i)
                        code_errors[i] = "\n\n".join(f.detailed() for f in code_f if f.step == i)
                    continue
                if plan_f:                                              # repair ONLY the root-cause plan step(s)
                    # Path/position errors cascade: a bad pose in step 2 makes steps 3, 4 ... fail too. Repair only the
                    # FIRST geometric failure; later ones are re-checked after the fix. Errors that depend only on a
                    # step's own numbers (speed, accel, gripper, sleep, joint bounds) and resolver errors are independent.
                    geometric = [f for f in plan_f if f.step and not _independent(f)]
                    first = min((f.step for f in geometric), default=None)
                    selected = [f for f in plan_f if not f.step or _independent(f) or f.step == first]
                    later = sorted({f.step for f in geometric if f.step != first})
                    by_src = {}
                    for f in selected:
                        by_src.setdefault(f.src, []).append(f)
                    progress(banner("REPAIR", "plan step(s) " + ", ".join(_path_text(p) for p in by_src) + " only"))
                    if later:
                        progress(f"  Errors in later step(s) {later} follow from the first failure (the pose chain); "
                                 "they are NOT touched now and are re-checked after the fix.")
                    for src, fl in by_src.items():
                        for f in fl:
                            progress("  " + f.detailed().replace("\n", "\n  "))
                        old = copy.deepcopy(_get_step(raw_plan, src))
                        ok = self._repair_plan_step(raw_plan, src, [f.detailed() for f in fl], failed_plan, state, progress)
                        if not ok:
                            cand.fail_reason = (f"The model could not produce a DIFFERENT valid version of plan step "
                                                f"{_path_text(src)} after several tries (it kept returning the same failed step).")
                            return self._finish_fail(cand)
                        repaired_paths.append(src)
                        progress(f"  step {_path_text(src)} BEFORE: {json.dumps(old.get('params'))}")
                        progress(f"  step {_path_text(src)} AFTER:  {json.dumps(_get_step(raw_plan, src).get('params'))}")
                        progress("  all other plan steps were left untouched; the FULL program is rebuilt and re-validated next")
                    continue
                if glob:                                                # whole-plan problem: minimal re-plan
                    progress(banner("REPAIR", "whole plan (no single step is to blame)"))
                    text = "\n\n".join(f.detailed() for f in glob)
                    progress("  " + text.replace("\n", "\n  "))
                    before = _norm_plan(raw_plan)
                    self.history.append({"role": "user", "content": P.repair_plan_prompt(text, raw_plan)})
                    kind, obj = self._planner_loop(progress)
                    if kind != "plan":
                        cand.fail_reason = f"During repair the planner answered '{kind}' instead of a corrected plan."
                        return self._finish_fail(cand)
                    if _norm_plan(obj) == before:
                        cand.fail_reason = "The planner returned the same rejected plan again."
                        return self._finish_fail(cand)
                    raw_plan = obj
                    continue
            cand.ok = False
            cand.fail_reason = cand.fail_reason or f"Still failing after the initial generation + {L.MAX_REPAIR_ATTEMPTS} repairs."
            return cand
        except Aborted:
            raise
        except LLMError as e:
            cand.ok = False
            cand.fail_reason = f"AI error: {e}"
            return cand

    def build_candidate_fixed(self, task, plan, note, progress=print):
        """Candidate for BUILT-IN commands (e.g. /robot): the plan is made by software, not by the model.
        resolve -> template code -> FULL deterministic validation. No LLM call, no repair loop: if the validator
        rejects it, that is final. The AI review is skipped on purpose and the review box says so."""
        cand = Candidate(task)
        cand.refs = dict(self.refs)
        state = self.robot.get_state()
        progress(banner("PLAN", "built-in command, no AI"))
        steps, plan_findings = R.resolve_plan(plan, state, refs=self.refs)
        cand.plan, cand.steps = plan, steps
        progress(render_plan(plan, steps))
        codes = {}
        for st in steps:
            try:
                codes[st["id"]] = R.expected_line(st)
            except (KeyError, TypeError):
                codes[st["id"]] = ""
        cand.step_codes = codes
        progress(banner("STEP CODE"))
        progress(render_codes(steps, codes))
        res, _meta = R.validate_candidate(steps, codes, state, plan_findings)
        cand.validation = res
        cand.state_text = P.state_summary(state, self.refs)
        progress(banner("VALIDATOR", "full program"))
        progress(render_validator(res))
        cand.attempts.append({"label": "built-in command", "plan": copy.deepcopy(plan), "steps": copy.deepcopy(steps),
                              "codes": dict(codes), "findings": [f.detailed() for f in res.findings], "review": None})
        if not res.ok:
            cand.fail_reason = "The validator rejected the built-in command: " + " | ".join(f.message for f in res.findings)
            return cand
        cand.review = {"approved": True, "summary": note, "findings": []}
        cand.ok = True
        cand.fingerprint_data = {"plan": cand.plan, "steps": cand.steps, "codes": cand.step_codes, "sha": res.sha256,
                                 "cfg": L.fingerprint_material(), "pose": state.pose if state else None}
        return cand

    def _finish_fail(self, cand):
        cand.ok = False
        return cand


_INDEPENDENT_PARAMS = {"v", "a", "width_mm", "force_n", "seconds"}


def _independent(f):
    """True if the finding depends only on the step's own numbers, not on where earlier steps left the robot."""
    p = f.param or ""
    return p in _INDEPENDENT_PARAMS or (len(p) > 1 and p[0] == "q" and p[1:].isdigit())


def _facts(cand):
    """Numbers the deterministic side measured, handed to the reviewer so it never has to do arithmetic."""
    f = {}
    res = cand.validation
    if res is not None and res.final_pose is not None and cand.refs.get("start"):
        disp = [(res.final_pose[i] - cand.refs["start"][i]) * 1000.0 for i in range(3)]
        f["end_minus_start_mm"] = [round(x, 2) for x in disp]
        f["end_vs_start_distance_mm"] = round(math.sqrt(sum(x * x for x in disp)), 4)
        f["end_vs_start_exact_within_tolerance"] = math.sqrt(sum(x * x for x in disp)) / 1000.0 <= L.EXACT_RETURN_TOL_M
    notes = [st.get("name", "") + ": " + st["note"] for st in cand.steps if st.get("note")]
    if notes:
        f["moves_shortened_or_limited"] = notes
    f["workspace_margin_note"] = (f"until_limit targets sit LIMIT_MARGIN_M = {L.LIMIT_MARGIN_M * 1000:.0f} mm inside the limit, "
                                 "by design; that is correct, not an error.")
    return f


def _blocking_reviewer_errors(review, facts, log):
    """Reviewer 'errors' that only complain about tiny precision are overruled by the deterministic facts.
    Anything else the reviewer marks as an error still blocks and goes to repair."""
    blocking = []
    for f in review["findings"]:
        if f["severity"] != "error":
            continue
        if f.get("kind") == "start_return" and facts.get("end_vs_start_exact_within_tolerance"):
            log(f"  [REVIEW] overruled by the validator: {f['message'][:140]}  "
                f"(measured end-vs-start {facts['end_vs_start_distance_mm']} mm, tolerance {L.EXACT_RETURN_TOL_M * 1000} mm)")
            continue
        blocking.append(f)
    return blocking


def _norm_plan(plan):
    return json.dumps(plan.get("steps"), sort_keys=True, default=str)


# ---------------------------------------------------------------------------------------------
# Reports (Markdown) -- failures and diagnostics. Nothing was sent to the robot in any of these cases.
# ---------------------------------------------------------------------------------------------
def _report_path(prefix):
    os.makedirs(L.REPORTS_DIR, exist_ok=True)
    return os.path.join(L.REPORTS_DIR, f"{prefix}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.md")


def write_failure_report(cand):
    path = _report_path("failure")
    out = [f"# Failure report", "", "## User task", "", cand.task, "", f"## Why it failed", "", cand.fail_reason or "(see attempts)", "",
           f"Saved start pose: {cand.refs.get('start')}", ""]
    for i, a in enumerate(cand.attempts):
        out += [f"## Attempt {i + 1}: {a['label']}", "", "### Plan", "```json", json.dumps(a["plan"], indent=2), "```",
                "### Resolved steps", "```json", json.dumps(a["steps"], indent=2), "```", "### Code per step", "```"]
        out += [f"step {k}: {v}" for k, v in sorted(a["codes"].items())]
        out += ["```", "### Validator / reviewer findings", "```"]
        out += a["findings"] or ["(none)"]
        out += ["```", ""]
    out += ["## Outcome", "", f"No valid program. NOTHING was sent to the robot."]
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(out) + "\n")
    return path


def write_diagnostic_report(task, error_text, agent):
    """For AI failures before a candidate exists (e.g. empty planner replies)."""
    path = _report_path("diagnostic")
    tail = agent.history[-6:] if agent else []
    out = ["# AI diagnostic report", "", "## User task", "", task or "(unknown)", "", "## Error", "", error_text, "",
           "## Last planner messages (truncated)", ""]
    for m in tail:
        out += [f"**{m['role']}**", "```", (m["content"] or "")[:2500], "```", ""]
    out += ["Nothing was sent to the robot."]
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(out) + "\n")
    return path