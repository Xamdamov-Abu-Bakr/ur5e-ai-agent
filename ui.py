"""ui.py -- terminal UI for the UR5e controller. Standard library only (msvcrt on Windows, termios on Unix).

What it gives main.py
  * paint / colorize        ANSI colours (auto-off with NO_COLOR, piped output, or UI_COLOR = False)
  * Console                 ONE reader thread owns the keyboard. Four modes:
        busy    AI is working / robot is moving. Keys:  s = STOP at once,  Enter (or d) = show/hide process details
        line    a normal prompt with a small line editor (arrows, history, Tab completion, Ctrl+L)
        select  button row, switched with Left/Right, confirmed with Enter, Esc cancels
  * Pipeline                animated status line ("Thinking... planner  12s") + a buffered process log that the
                            operator can expand with Enter at any time (and read later with /log)
  * clear_screen / rule / say   small helpers (say = thread-safe print)

If stdin/stdout is not a real terminal (pipes, IDE consoles without VT support) everything falls back to plain
line input: no animation, the full log is printed, and approval is typed ("APPROVE").
"""
import atexit
import contextlib
import os
import queue
import re
import shutil
import sys
import threading
import time
import _thread

import limits as L

IS_WIN = os.name == "nt"
if IS_WIN:
    import msvcrt
else:
    import codecs
    import select
    import termios
    import tty

OUT_LOCK = threading.RLock()
_ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


def _enable_vt():
    """Windows 10+: switch the console to ANSI escape processing. Other systems: always fine."""
    if not IS_WIN:
        return True
    try:
        import ctypes
        k32 = ctypes.windll.kernel32
        handle = k32.GetStdHandle(-11)
        mode = ctypes.c_uint32()
        if not k32.GetConsoleMode(handle, ctypes.byref(mode)):
            return False
        return bool(k32.SetConsoleMode(handle, mode.value | 0x0004))   # ENABLE_VIRTUAL_TERMINAL_PROCESSING
    except Exception:
        return False


_IS_TTY = sys.stdout.isatty() and sys.stdin.isatty()
VT = _IS_TTY and _enable_vt()
USE_COLOR = bool(L.UI_COLOR) and sys.stdout.isatty() and VT and "NO_COLOR" not in os.environ
ASCII = bool(L.UI_ASCII)

SYM = {"dot": "*" if ASCII else "●", "prompt": ">" if ASCII else "❯", "ok": "[OK]" if ASCII else "✔",
       "fail": "[X]" if ASCII else "✘", "left": ">" if ASCII else "▶", "right": "<" if ASCII else "◀",
       "dash": "-" if ASCII else "─", "mid": "|" if ASCII else "·", "ellipsis": "..." if ASCII else "…",
       "arrows": "Left/Right" if ASCII else "← →"}
FRAMES = "|/-\\" if ASCII else "⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏"

_STYLES = {"bold": "1", "dim": "2", "red": "31", "green": "32", "yellow": "33", "blue": "34", "magenta": "35",
           "cyan": "36", "gray": "90", "white": "97", "hi_blue": "94", "bg_green": "42;30", "bg_red": "41;97"}


def paint(text, *styles):
    codes = [_STYLES[s] for s in styles if s in _STYLES]      # unknown / empty style names are ignored
    if not USE_COLOR or not codes:
        return text
    return "\x1b[" + ";".join(codes) + "m" + text + "\x1b[0m"


def strip_ansi(text):
    return _ANSI_RE.sub("", text)


def term_width():
    return max(30, shutil.get_terminal_size((80, 24)).columns)


# ---------------------------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------------------------
def say(text="", end="\n"):
    """Thread-safe print. On a terminal the current line (spinner / prompt remnant) is cleared first."""
    with OUT_LOCK:
        if VT:
            sys.stdout.write("\r\x1b[K")
        sys.stdout.write(text + end)
        sys.stdout.flush()


def rule(title="", style="cyan"):
    w = min(term_width() - 1, 78)
    head = f"{SYM['dash'] * 2} {title} " if title else ""
    return paint(head + SYM["dash"] * max(2, w - len(head)), style)


def clear_screen():
    with OUT_LOCK:
        if not _IS_TTY:
            return
        if IS_WIN and not VT:
            os.system("cls")
        else:
            sys.stdout.write("\x1b[2J\x1b[3J\x1b[H")
            sys.stdout.flush()


_RE_CODE = re.compile(r"^\s+\d+\s+(movel|movej|movec|rg_grip|sleep)\(")


def colorize(text):
    """Colour a block of pipeline/validator output line by line (no-op without colour support)."""
    if not USE_COLOR:
        return text
    out = []
    for ln in text.split("\n"):
        s = ln.strip()
        if s.startswith("===="):
            out.append(paint(ln, "cyan", "bold"))
        elif s.startswith("[LLM]"):
            out.append(paint(ln, "gray"))
        elif s.startswith("[REPAIR]") or s.startswith("Errors in later") or s.startswith("WARNING") or s.startswith("ADJUSTED"):
            out.append(paint(ln, "yellow"))
        elif s.startswith("[REVIEW]"):
            out.append(paint(ln, "magenta"))
        elif s.startswith("[error]") or s.startswith("FAIL") or s.startswith("NOT APPROVED") or s.startswith("ERROR") \
                or "NOTHING will run" in s or s.startswith("!!!"):
            out.append(paint(ln, "red", "bold") if s.startswith(("FAIL", "NOT APPROVED", "!!!")) else paint(ln, "red"))
        elif s.startswith("[warning]"):
            out.append(paint(ln, "yellow"))
        elif s.startswith("[info]"):
            out.append(paint(ln, "gray"))
        elif s.startswith("PASS") or (s.startswith("APPROVED") and not s.startswith("NOT")):
            out.append(paint(ln, "green", "bold"))
        elif _RE_CODE.match(ln) or s.startswith(("movel(", "movej(", "movec(", "rg_grip(", "sleep(")):
            out.append(paint(ln, "hi_blue"))
        else:
            out.append(ln)
    return "\n".join(out)


# ---------------------------------------------------------------------------------------------
# Keyboard (one key at a time)
# ---------------------------------------------------------------------------------------------
class _WinKeys:
    _EXT = {"H": "UP", "P": "DOWN", "K": "LEFT", "M": "RIGHT", "G": "HOME", "O": "END", "S": "DELETE"}

    def get(self):
        ch = msvcrt.getwch()
        if ch in ("\x00", "\xe0"):
            return self._EXT.get(msvcrt.getwch(), "")
        return {"\r": "ENTER", "\n": "ENTER", "\x08": "BACKSPACE", "\x03": "CTRL_C", "\x04": "CTRL_D",
                "\x1a": "CTRL_D", "\x1b": "ESC", "\t": "TAB", "\x0c": "CTRL_L"}.get(ch, ch)


class _UnixKeys:
    _CSI = {b"A": "UP", b"B": "DOWN", b"C": "RIGHT", b"D": "LEFT", b"H": "HOME", b"F": "END", b"3~": "DELETE",
            b"1~": "HOME", b"7~": "HOME", b"4~": "END", b"8~": "END"}

    def __init__(self, fd):
        self.fd = fd
        self.dec = codecs.getincrementaldecoder("utf-8")("replace")

    def _byte(self, timeout=None):
        if timeout is not None and not select.select([self.fd], [], [], timeout)[0]:
            return None
        b = os.read(self.fd, 1)
        if not b:
            raise EOFError
        return b

    def get(self):
        while True:
            b = self._byte()
            if b == b"\x1b":
                return self._escape()
            if b in (b"\r", b"\n"):
                return "ENTER"
            if b in (b"\x7f", b"\x08"):
                return "BACKSPACE"
            named = {b"\x04": "CTRL_D", b"\t": "TAB", b"\x0c": "CTRL_L"}.get(b)
            if named:
                return named
            ch = self.dec.decode(b)          # "" while a multi-byte character is still incomplete
            if ch:
                return ch

    def _escape(self):
        nxt = self._byte(0.05)
        if nxt is None:
            return "ESC"
        if nxt in (b"[", b"O"):
            seq = b""
            while True:
                c = self._byte(0.05)
                if c is None:
                    return ""
                seq += c
                if c.isalpha() or c == b"~":
                    return self._CSI.get(seq, "")
        return ""                            # Alt+key etc.: ignored


# ---------------------------------------------------------------------------------------------
# Console: the single owner of the keyboard
# ---------------------------------------------------------------------------------------------
class Console:
    def __init__(self, on_stop, stop_words=(), completions=()):
        self.on_stop = on_stop
        self.stop_words = {w.lower() for w in stop_words}
        self.completions = sorted(completions)
        self.interactive = _IS_TTY and VT
        self.mode = "busy"                   # busy | line | select
        self.lines, self.keys = queue.Queue(), queue.Queue()
        self.history = []
        self.on_toggle = None                # set by Pipeline while it runs
        self._prompt, self._buf, self._cur, self._row = "", [], 0, 0
        self._hidx, self._draft = None, ""
        self._old_term, self._fd = None, None
        self._src = None

    # --- lifecycle ---
    def start(self):
        if self.interactive:
            if IS_WIN:
                self._src = _WinKeys()
            else:
                self._fd = sys.stdin.fileno()
                self._old_term = termios.tcgetattr(self._fd)
                tty.setcbreak(self._fd)      # no echo, no line buffering; Ctrl+C still raises KeyboardInterrupt
                self._src = _UnixKeys(self._fd)
                atexit.register(self.restore)
            target = self._loop_keys
        else:
            target = self._loop_plain
        threading.Thread(target=target, daemon=True, name="keyboard").start()

    def restore(self):
        if self._old_term is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
            finally:
                self._old_term = None

    # --- reader threads ---
    def _loop_plain(self):
        while True:
            try:
                line = input()
            except EOFError:
                self.lines.put(None)
                return
            if line.strip().lower() in self.stop_words:
                self.on_stop()
                continue
            self.lines.put(line)

    def _loop_keys(self):
        while True:
            try:
                key = self._src.get()
            except (EOFError, OSError):
                self.lines.put(None)
                return
            if not key:
                continue
            try:
                self._handle(key)
            except Exception as e:           # the keyboard thread must never die
                say(paint(f"[ui] input error: {type(e).__name__}: {e}", "red"))

    def _stop(self):
        self.on_stop()
        if self.mode == "line":
            self._render()

    def _handle(self, key):
        if key == "CTRL_C":
            _thread.interrupt_main()         # Windows only; on Unix Ctrl+C arrives as SIGINT
            return
        mode = self.mode
        if mode == "busy":
            if key in ("s", "S"):
                self._stop()
            elif key in ("ENTER", " ", "d", "D", "v", "V", "TAB"):
                cb = self.on_toggle
                if cb:
                    cb()
            elif key == "CTRL_L":
                clear_screen()
            return
        if mode == "select":
            if key in ("s", "S"):
                self._stop()
            else:
                self.keys.put(key)
            return
        self._edit(key)

    # --- line editor ---
    def _render(self):
        with OUT_LOCK:
            w = term_width()
            pv = len(strip_ansi(self._prompt))
            text = "".join(self._buf)
            total = pv + len(text)
            out = (f"\x1b[{self._row}A" if self._row else "") + "\r" + self._prompt + text + "\x1b[J"
            if total and total % w == 0:
                out += " \r"                 # cursor would sit in the "pending wrap" state: force it to the next row
            end_row = total // w
            cur_row, cur_col = divmod(pv + self._cur, w)
            if end_row - cur_row:
                out += f"\x1b[{end_row - cur_row}A"
            out += "\r" + (f"\x1b[{cur_col}C" if cur_col else "")
            self._row = cur_row
            sys.stdout.write(out)
            sys.stdout.flush()

    def _set(self, text):
        self._buf, self._cur = list(text), len(text)
        self._render()

    def _edit(self, key):
        buf = self._buf
        if key == "ENTER":
            text = "".join(buf)
            self._cur = len(buf)
            self._render()
            with OUT_LOCK:
                sys.stdout.write("\n")
                sys.stdout.flush()
            self._buf, self._cur, self._row, self._hidx = [], 0, 0, None
            if text.strip() and (not self.history or self.history[-1] != text):
                self.history.append(text)
            if text.strip().lower() in self.stop_words:
                self.on_stop()
                self._render()               # same prompt again, the main thread is still waiting
                return
            self.lines.put(text)
        elif key == "BACKSPACE":
            if self._cur > 0:
                del buf[self._cur - 1]
                self._cur -= 1
                self._render()
        elif key == "DELETE":
            if self._cur < len(buf):
                del buf[self._cur]
                self._render()
        elif key == "LEFT":
            if self._cur > 0:
                self._cur -= 1
                self._render()
        elif key == "RIGHT":
            if self._cur < len(buf):
                self._cur += 1
                self._render()
        elif key == "HOME":
            self._cur = 0
            self._render()
        elif key == "END":
            self._cur = len(buf)
            self._render()
        elif key == "UP":
            if self.history:
                if self._hidx is None:
                    self._draft, self._hidx = "".join(buf), len(self.history)
                if self._hidx > 0:
                    self._hidx -= 1
                    self._set(self.history[self._hidx])
        elif key == "DOWN":
            if self._hidx is not None:
                self._hidx += 1
                if self._hidx >= len(self.history):
                    self._hidx = None
                    self._set(self._draft)
                else:
                    self._set(self.history[self._hidx])
        elif key == "ESC":
            self._set("")
        elif key == "CTRL_L":
            clear_screen()
            self._row = 0
            self._render()
        elif key == "CTRL_D":
            if not buf:
                self.lines.put(None)
            elif self._cur < len(buf):
                del buf[self._cur]
                self._render()
        elif key == "TAB":
            self._complete()
        elif len(key) == 1 and key.isprintable():
            buf.insert(self._cur, key)
            self._cur += 1
            self._render()

    def _complete(self):
        text = "".join(self._buf)
        if not text.startswith("/") or " " in text:
            return
        hits = [c for c in self.completions if c.startswith(text.lower())]
        if len(hits) == 1:
            self._set(hits[0] + " ")
        elif len(hits) > 1:
            pref = os.path.commonprefix(hits)
            if len(pref) > len(text):
                self._set(pref)
            else:
                say(paint("  " + "   ".join(hits), "gray"))
                self._row = 0
                self._render()

    # --- API for the main thread ---
    @staticmethod
    def _drain(q):
        while True:
            try:
                q.get_nowait()
            except queue.Empty:
                return

    def read_line(self, prompt="", timeout=None):
        """One line of input. Returns the text, '' on timeout, None on end of input (Ctrl+D / closed stdin)."""
        if not self.interactive:
            sys.stdout.write(strip_ansi(prompt))
            sys.stdout.flush()
            try:
                return self.lines.get(timeout=timeout)
            except queue.Empty:
                return ""
        with OUT_LOCK:
            self._drain(self.lines)
            self._prompt, self._buf, self._cur, self._row, self._hidx = prompt, [], 0, 0, None
            self.mode = "line"
            self._render()
        try:
            return self.lines.get(timeout=timeout)
        except queue.Empty:
            say("")
            return ""
        finally:
            self.mode = "busy"

    def select(self, options, default=0, timeout=None):
        """Button row. options = [(label, style), ...]. Left/Right (or Tab) move, Enter confirms, Esc cancels.
        Returns the chosen index, or None on Esc / timeout. Interactive terminals only."""
        idx, n = default, len(options)
        deadline = None if timeout is None else time.monotonic() + timeout
        self._drain(self.keys)
        self.mode = "select"
        try:
            while True:
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    say("")
                    return None
                self._draw_buttons(options, idx, remaining)
                try:
                    key = self.keys.get(timeout=0.25)
                except queue.Empty:
                    continue
                if key == "LEFT":
                    idx = max(0, idx - 1)
                elif key == "RIGHT":
                    idx = min(n - 1, idx + 1)
                elif key == "TAB":
                    idx = (idx + 1) % n
                elif key == "ENTER":
                    self._draw_buttons(options, idx, None)
                    with OUT_LOCK:
                        sys.stdout.write("\n")
                        sys.stdout.flush()
                    return idx
                elif key == "ESC":
                    say("")
                    return None
        finally:
            self.mode = "busy"

    def _draw_buttons(self, options, idx, remaining):
        parts = []
        for i, (label, style) in enumerate(options):
            if i == idx:
                parts.append(paint(f" {SYM['left']} {label} {SYM['right']} ", style, "bold"))
            else:
                parts.append(paint(f"   {label}   ", "gray"))
        line = "   " + "   ".join(parts)
        if remaining is not None:
            m, s = divmod(int(remaining), 60)
            line += paint(f"    {m}:{s:02d} left", "gray")
        with OUT_LOCK:
            sys.stdout.write("\r" + line + "\x1b[K")
            sys.stdout.flush()


# ---------------------------------------------------------------------------------------------
# Pipeline: animated status line + buffered, expandable process log
# ---------------------------------------------------------------------------------------------
_TAG_STATUS = {"PLAN": "Planning", "STEP CODE": "Writing code", "FULL PROGRAM": "Assembling program",
               "VALIDATOR": "Validating", "REVALIDATION": "Validating", "AI REVIEW": "Reviewing",
               "REPAIR": "Repairing", "TASK": "Starting"}


class Pipeline:
    """Callable log sink: pass it wherever the agent expects `log=print`. Every message is stored; the status line
    shows what the pipeline is doing right now. Enter (while it runs) shows/hides the full log."""

    def __init__(self, console, details=True):
        self.con, self.details = console, details
        self.lines = []
        self.verbose = not console.interactive          # without a terminal there is no animation: print everything
        self.status, self.detail = "Working", ""
        self._t0 = time.monotonic()
        self._thread, self._halt = None, threading.Event()
        self.elapsed = 0.0

    # --- log sink ---
    def __call__(self, *args, **kwargs):
        text = " ".join(str(a) for a in args)
        with OUT_LOCK:
            self.lines.append(text)
            self._parse(text)
            if self.verbose:
                say(colorize(text))

    def _parse(self, text):
        t = text.strip()
        m = re.match(r"^\[LLM\]\s+(.+?)\s+\.\.\.$", t)
        if m:
            self.status, self.detail = "Thinking", m.group(1)
            return
        if t.startswith("[LLM]"):
            if "reply in" in t:
                self.status = "Reading reply"
            elif "EMPTY" in t or "rejected" in t or "retry" in t:
                self.status = "Retrying"
            return
        m = re.match(r"^=+\s*\[([^\]]+)\]\s*(.*)$", t)
        if m:
            self.status = _TAG_STATUS.get(m.group(1), m.group(1).title())
            self.detail = re.sub(r"=+$", "", m.group(2)).strip()

    def toggle(self):
        if not self.details:
            return
        with OUT_LOCK:
            self.verbose = not self.verbose
            if self.verbose:
                say(rule("process log (Enter hides it)", "gray"))
                for t in self.lines:
                    say(colorize(t))
            else:
                say(paint("   details hidden - Enter shows them again", "gray"))

    def dump(self, title="process log"):
        with OUT_LOCK:
            say(rule(title, "gray"))
            for t in self.lines:
                say(colorize(t))
            say(rule("", "gray"))

    # --- animation ---
    def _draw(self, i):
        el = time.monotonic() - self._t0
        base = f"{FRAMES[i % len(FRAMES)]} {self.status}{SYM['ellipsis']}"
        det = f" {self.detail}" if self.detail else ""
        tail = f"  {el:.0f}s"
        hint = ""
        if self.details:
            hint += f"Enter: {'hide' if self.verbose else 'show'} details   "
        hint += "s: STOP"
        room = term_width() - 2
        if len(base + det + tail) + 3 + len(hint) > room:
            hint = "s: STOP" if len(base + det + tail) + 3 + 7 <= room else ""
        if len(base + det + tail) + (3 + len(hint) if hint else 0) > room:
            det = det[: max(0, room - len(base + tail) - (3 + len(hint) if hint else 0) - 1)]
        line = paint(base, "cyan", "bold") + paint(det, "gray") + paint(tail, "gray")
        if hint:
            line += "   " + paint(hint, "gray")
        with OUT_LOCK:
            sys.stdout.write("\r" + line + "\x1b[K")
            sys.stdout.flush()

    def _spin(self):
        i = 0
        period = 1.0 / max(1.0, float(L.UI_SPINNER_FPS))
        while not self._halt.wait(period):
            self._draw(i)
            i += 1

    @contextlib.contextmanager
    def running(self, status="Working", detail=""):
        self.status, self.detail, self._t0 = status, detail, time.monotonic()
        self.con.on_toggle = self.toggle
        self.con.mode = "busy"
        if self.con.interactive:
            self._halt.clear()
            self._thread = threading.Thread(target=self._spin, daemon=True, name="spinner")
            self._thread.start()
        try:
            yield self
        finally:
            self._halt.set()
            if self._thread:
                self._thread.join(timeout=1.0)
                self._thread = None
            self.con.on_toggle = None
            self.elapsed = time.monotonic() - self._t0
            with OUT_LOCK:
                if VT:
                    sys.stdout.write("\r\x1b[K")
                    sys.stdout.flush()
