"""
fois/scheduler.py

Runs the FOIS engine on a configurable interval.

This is separate from the operator's pipeline scheduler.
The investor runs on a much slower cadence — daily or weekly is plenty.
The operator runs every 60 seconds reacting to sensor events.

Usage:
    python -m fois.scheduler

Or import and call directly:
    from fois.scheduler import FoisScheduler
    scheduler = FoisScheduler(db_path="farm.db", interval_hours=24)
    scheduler.run()
"""

import logging
import time
from datetime import datetime, timezone

from fois.engine import FoisEngine

logger = logging.getLogger(__name__)


class FoisScheduler:

    def __init__(self, db_path: str = "farm.db", interval_hours: float = 24,
                 top_n: int = 3, run_once: bool = False):
        self.db_path = db_path
        self.interval_seconds = interval_hours * 3600
        self.top_n = top_n
        self.run_once = run_once

    def run(self):
        logger.info(
            f"FOIS scheduler started — interval: {self.interval_seconds/3600:.1f}h, "
            f"top_n: {self.top_n}"
        )

        while True:
            self._run_cycle()

            if self.run_once:
                logger.info("FOIS scheduler: run_once=True, exiting")
                break

            next_run = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
            logger.info(
                f"FOIS scheduler: next run in {self.interval_seconds/3600:.1f}h "
                f"(sleeping from {next_run})"
            )
            time.sleep(self.interval_seconds)

    def _run_cycle(self):
        started = datetime.now(timezone.utc)
        logger.info(f"FOIS: cycle starting at {started.isoformat()}")

        try:
            engine = FoisEngine(db_path=self.db_path, top_n=self.top_n)
            opportunities = engine.run()

            elapsed = (datetime.now(timezone.utc) - started).total_seconds()
            logger.info(
                f"FOIS: cycle complete in {elapsed:.1f}s — "
                f"{len(opportunities)} opportunity/ies written to pending/"
            )

        except Exception as e:
            logger.error(f"FOIS: cycle failed: {e}", exc_info=True)
            # Don't crash the scheduler — log and wait for next cycle


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    import argparse
    parser = argparse.ArgumentParser(description="FOIS Scheduler")
    parser.add_argument("--db", default="farm.db", help="Path to farm SQLite DB")
    parser.add_argument("--interval-hours", type=float, default=24.0,
                        help="Hours between FOIS runs (default: 24)")
    parser.add_argument("--top-n", type=int, default=3,
                        help="Max proposals to generate per run (default: 3)")
    parser.add_argument("--once", action="store_true",
                        help="Run once and exit (useful for testing)")
    args = parser.parse_args()

    scheduler = FoisScheduler(
        db_path=args.db,
        interval_hours=args.interval_hours,
        top_n=args.top_n,
        run_once=args.once,
    )
    scheduler.run()
