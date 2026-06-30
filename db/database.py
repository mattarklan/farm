import sqlite3
from contextlib import contextmanager

DB_PATH = "farm.db"


def init_db():
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()

    # Performance + concurrency improvement (important for ingestion)
    cur.execute("PRAGMA journal_mode=WAL;")
    cur.execute("PRAGMA synchronous=NORMAL;")
    cur.execute("PRAGMA temp_store=MEMORY;")

    conn.commit()
    conn.close()


@contextmanager
def get_conn():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    try:
        yield conn
        conn.commit()
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()


def execute(query: str, params=()):
    with get_conn() as conn:
        cur = conn.cursor()
        cur.execute(query, params)


def fetchall(query: str, params=()):
    with get_conn() as conn:
        cur = conn.cursor()
        cur.execute(query, params)
        return cur.fetchall()


def fetchone(query: str, params=()):
    with get_conn() as conn:
        cur = conn.cursor()
        cur.execute(query, params)
        return cur.fetchone()