"""
Logs operational PROBLEMS only -- not routine decision-cycle output
(that's gui_export.py's job, into config/events.json / actions.json).
This should stay quiet on a normal day and only write when core itself
is misbehaving:

  - couldn't reach the ingestion API
  - startup / shutdown issues (crashed, didn't shut down cleanly)
  - an unhandled exception inside a decision cycle
  - corrupt/unreadable data
  - anything else where core isn't doing its job correctly

Plain text, one line per problem, append-only -- deliberately not JSON;
this is meant to be tailed/grepped by a human during an incident, not
parsed by a GUI.
"""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path


class Logger:
    def __init__(self, path: str = "logs/system.log"):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def startup(self):
        self._write("STARTUP", "core started")

    def shutdown(self, clean: bool = True):
        self._write("SHUTDOWN", "core stopped cleanly" if clean else "core stopped UNCLEANLY")

    def api_unreachable(self, detail: str):
        self._write("API_UNREACHABLE", detail)

    def cycle_error(self, exc: Exception):
        self._write("CYCLE_ERROR", f"{type(exc).__name__}: {exc}")

    def data_error(self, detail: str):
        self._write("DATA_ERROR", detail)

    def _write(self, kind: str, message: str):
        ts = datetime.now(timezone.utc).isoformat()
        with open(self.path, "a", encoding="utf-8") as f:
            f.write(f"{ts} [{kind}] {message}\n")
