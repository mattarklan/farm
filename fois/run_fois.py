"""Convenience entry point.

Usage:
    python run_fois.py fetch      # run all fetchers once, then exit
    python run_fois.py serve      # start the API server (uvicorn)

Lives at farm/fois/run_fois.py. Can be run directly from any working
directory - it adds farm/ (its own parent's parent) to sys.path so
`import fois.x` resolves regardless of cwd.

In production you'd likely run the fetch as a daily cron/systemd timer,
and the API as a long-running service (matching how ingestor is deployed).
"""

import logging
import sys
from pathlib import Path

FARM_ROOT = Path(__file__).resolve().parent.parent
if str(FARM_ROOT) not in sys.path:
    sys.path.insert(0, str(FARM_ROOT))


def main():
    logging.basicConfig(level=logging.INFO)

    if len(sys.argv) < 2 or sys.argv[1] not in ("fetch", "serve"):
        print(__doc__)
        sys.exit(1)

    command = sys.argv[1]

    if command == "fetch":
        from fois.scheduler import run_all
        results = run_all()
        for domain, count in results.items():
            print(f"{domain}: {count} readings inserted")

    elif command == "serve":
        import uvicorn
        uvicorn.run("fois.api:app", host="0.0.0.0", port=8010, reload=False,
                     app_dir=str(FARM_ROOT))


if __name__ == "__main__":
    main()
