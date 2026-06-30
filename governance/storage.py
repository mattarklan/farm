from pathlib import Path
import tempfile
import yaml


def _load_yaml(path: Path) -> dict:
    if not path.exists():
        return {}
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def _save_yaml(path: Path, data: dict) -> None:
    """Write to a temp file in the same directory, then atomically replace."""
    dir_ = path.parent
    with tempfile.NamedTemporaryFile(
        "w", dir=dir_, delete=False, suffix=".tmp"
    ) as tmp:
        yaml.safe_dump(data, tmp, sort_keys=False)
        tmp_path = Path(tmp.name)
    tmp_path.replace(path)  # atomic on POSIX; best-effort on Windows


def load_rules(path: Path) -> dict:
    return _load_yaml(path)


def save_rules(path: Path, rules: dict) -> None:
    _save_yaml(path, rules)
