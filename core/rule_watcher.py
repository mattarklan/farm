"""
Watches config/rules.yaml for changes and reloads RuleEngine/GoalEngine
in place, the instant it changes -- no polling loop.

This IS "the facility for governance to interface with core" mentioned
in chat: governance's whole job is writing a validated rule into this
file; core's only job here is noticing and reloading. Nothing about
core needs to know governance exists beyond that -- it just watches a
file, same as it would if you'd hand-edited rules.yaml yourself.

Uses the `watchdog` package (OS-level file system events -- inotify on
Linux, ReadDirectoryChangesW on Windows, FSEvents on macOS) if
installed, which is genuinely idle at 0% CPU between changes. Falls
back to a slow poll (every FALLBACK_POLL_SECONDS) with a one-time
warning logged if `watchdog` isn't installed, rather than hard-failing
-- `pip install watchdog` to get the real thing.

Thread-safety note: RuleEngine.reload()/GoalEngine.reload() just
reassign self.rules to a new list -- that's a single atomic pointer
swap under the GIL, so calling reload() from the watcher's background
thread while the main loop is mid-cycle is safe without extra locking.
The main loop will simply pick up the new rules on its NEXT cycle,
never mid-cycle.
"""
from __future__ import annotations

import threading
import time
from pathlib import Path

FALLBACK_POLL_SECONDS = 5

try:
    from watchdog.observers import Observer
    from watchdog.events import FileSystemEventHandler
    _HAVE_WATCHDOG = True
except ImportError:
    _HAVE_WATCHDOG = False


class RuleWatcher:
    def __init__(self, path: str, on_change, logger=None):
        """
        path: the rules.yaml path to watch.
        on_change: callback, called with no args whenever the file changes.
        logger: optional core.logger.Logger, for reporting fallback mode / errors.
        """
        self.path = Path(path)
        self.on_change = on_change
        self.logger = logger
        self._observer = None
        self._stop_flag = threading.Event()
        self._poll_thread = None

    def start(self):
        if _HAVE_WATCHDOG:
            self._start_watchdog()
        else:
            if self.logger:
                self.logger.data_error(
                    "watchdog package not installed -- falling back to a "
                    f"{FALLBACK_POLL_SECONDS}s poll for rules.yaml changes. "
                    "`pip install watchdog` for instant, zero-idle-CPU reload."
                )
            self._start_fallback_poll()

    def stop(self):
        if self._observer is not None:
            self._observer.stop()
            self._observer.join(timeout=2)
        self._stop_flag.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2)

    def _start_watchdog(self):
        watcher = self

        class _Handler(FileSystemEventHandler):
            def on_modified(self, event):
                if not event.is_directory and Path(event.src_path).resolve() == watcher.path.resolve():
                    watcher.on_change()

        self._observer = Observer()
        self._observer.schedule(_Handler(), str(self.path.parent), recursive=False)
        self._observer.daemon = True
        self._observer.start()

    def _start_fallback_poll(self):
        def loop():
            last_mtime = self._mtime()
            while not self._stop_flag.is_set():
                time.sleep(FALLBACK_POLL_SECONDS)
                current = self._mtime()
                if current is not None and current != last_mtime:
                    last_mtime = current
                    self.on_change()

        self._poll_thread = threading.Thread(target=loop, daemon=True)
        self._poll_thread.start()

    def _mtime(self):
        try:
            return self.path.stat().st_mtime
        except OSError:
            return None
