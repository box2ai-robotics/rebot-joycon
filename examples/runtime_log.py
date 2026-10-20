"""Process-wide console and SDK diagnostics; no hardware access."""
import atexit
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import sys
import threading
import time

_session = None
_start_lock = threading.Lock()


class _Tee:
    def __init__(self, stream, session, name):
        self.stream, self.session, self.name = stream, session, name

    def write(self, text):
        result = self.stream.write(text)
        if text:
            self.session.emit("console", stream=self.name, text=text)
        return result

    def flush(self):
        self.stream.flush()

    def __getattr__(self, name):
        return getattr(self.stream, name)


class RuntimeLog:
    def __init__(self, root):
        self.directory = Path(root) / (
            datetime.now().strftime("%Y%m%d_%H%M%S_%f") + f"_{os.getpid()}"
        )
        self.directory.mkdir(parents=True)
        self.events = (self.directory / "events.jsonl").open("x", encoding="utf-8")
        self.console = (self.directory / "console.log").open("x", encoding="utf-8")
        self.queue = queue.SimpleQueue()
        self.lock = threading.Lock()
        self.closed = False
        self.error = None
        self.original_stdout, self.original_stderr = sys.stdout, sys.stderr
        self.worker = threading.Thread(target=self._write, name="rebot-log", daemon=True)
        self.worker.start()

    def emit(self, event, **fields):
        # Callers supply snapshots, never mutable controller arrays.
        with self.lock:
            if not self.closed:
                self.queue.put(dict(time=datetime.now().astimezone().isoformat(),
                                    monotonic_s=time.monotonic(), event=event, **fields))

    def _write(self):
        try:
            while True:
                row = self.queue.get()
                if row is None:
                    break
                self.events.write(json.dumps(row, ensure_ascii=False) + "\n")
                self.events.flush()
                if row["event"] == "console":
                    self.console.write(row["text"])
                    self.console.flush()
        except Exception as exc:
            self.error = repr(exc)
            self.original_stderr.write(f"\n运行日志写入失败：{exc}\n")
            # Stop accepting data rather than accumulating an unbounded backlog.
            with self.lock:
                self.closed = True
            while not self.queue.empty():
                self.queue.get()
        finally:
            self.events.close()
            self.console.close()

    def close(self):
        with self.lock:
            if not self.closed:
                self.queue.put(dict(time=datetime.now().astimezone().isoformat(),
                                    monotonic_s=time.monotonic(), event="session_end"))
                self.closed = True
                self.queue.put(None)
        self.worker.join()
        if isinstance(sys.stdout, _Tee) and sys.stdout.session is self:
            sys.stdout = self.original_stdout
        if isinstance(sys.stderr, _Tee) and sys.stderr.session is self:
            sys.stderr = self.original_stderr


def start_runtime_log():
    global _session
    with _start_lock:
        if _session is None:
            root = os.environ.get("REBOT_LOG_DIR",
                                  str(Path(__file__).resolve().parents[1] / "logs"))
            session = RuntimeLog(root)
            _session = session
            sys.stdout = _Tee(sys.stdout, session, "stdout")
            sys.stderr = _Tee(sys.stderr, session, "stderr")
            atexit.register(session.close)
            session.emit("session_start", entry=sys.argv[0], cwd=os.getcwd(),
                         python=sys.executable, pid=os.getpid())
            print(f"运行日志目录：{session.directory}", flush=True)
        return _session


def record_event(event, **fields):
    if _session is not None:
        _session.emit(event, **fields)
