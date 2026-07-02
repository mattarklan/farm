import os
from farm.data.database import execute, fetchone

MIGRATIONS_DIR = os.path.join(os.path.dirname(__file__), "migrations")

def ensure_version_table():
    execute("""
        CREATE TABLE IF NOT EXISTS schema_version (
            version INTEGER NOT NULL
        )
    """)
    row = fetchone("SELECT version FROM schema_version LIMIT 1")
    if row is None:
        execute("INSERT INTO schema_version (version) VALUES (0)")

def get_version():
    row = fetchone("SELECT version FROM schema_version LIMIT 1")
    return row["version"] if row else 0

def set_version(v: int):
    execute("UPDATE schema_version SET version = ?", (v,))

def apply_migration(sql_file: str):
    path = os.path.join(MIGRATIONS_DIR, sql_file)
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, "r") as f:
        sql = f.read()
    statements = [s.strip() for s in sql.split(";") if s.strip()]
    for stmt in statements:
        execute(stmt)

def migrate():
    ensure_version_table()
    current = get_version()
    migrations = [
        (1, "001_init.sql"),
        (2, "002_add_events.sql"),
        (3, "003_add_summaries.sql"),
    ]
    for version, file in migrations:
        if current < version:
            print(f"Applying migration {file}")
            apply_migration(file)
            set_version(version)
    print(f"DB up to date (v{get_version()})")

if __name__ == "__main__":
    migrate()