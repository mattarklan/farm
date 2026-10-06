"""Loads and parses config/sources.yaml."""

import re
import yaml
from pathlib import Path
from datetime import timedelta
from importlib import import_module

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "fois" / "sources.yaml"

_DURATION_RE = re.compile(r"^(\d+)([hd])$")


def parse_duration(text: str) -> timedelta:
    """Parse simple durations like '48h' or '2d' into a timedelta."""
    match = _DURATION_RE.match(text.strip())
    if not match:
        raise ValueError(f"Unrecognised duration format: {text!r} (expected e.g. '48h', '2d')")
    amount, unit = match.groups()
    amount = int(amount)
    if unit == "h":
        return timedelta(hours=amount)
    return timedelta(days=amount)


class Config:
    def __init__(self, path: Path = DEFAULT_CONFIG_PATH):
        self.path = Path(path)
        with open(self.path) as f:
            self._raw = yaml.safe_load(f)

    @property
    def regions(self) -> dict:
        return self._raw.get("regions", {})

    @property
    def domains(self) -> dict:
        return self._raw.get("domains", {})

    def domain_names(self) -> list[str]:
        return list(self.domains.keys())

    def staleness_for(self, domain: str) -> timedelta:
        raw = self.domains[domain].get("staleness_warn_after", "48h")
        return parse_duration(raw)

    def sources_for(self, domain: str) -> list[dict]:
        sources = self.domains[domain].get("sources", [])
        return sorted(sources, key=lambda s: s.get("priority", 999))

    def mode_for(self, domain: str) -> str:
        """'fallback' (default): try sources in priority order, stop at first
        success per region - for when sources are redundant (e.g. weather).
        'all': try every source, keep every successful reading - for when
        sources represent genuinely different values (e.g. price per
        supplier), not alternatives for the same fact."""
        return self.domains[domain].get("mode", "fallback")

    def load_fetcher_class(self, source_cfg: dict):
        """Import and return the Fetcher class referenced by a source config entry."""
        module = import_module(source_cfg["module"])
        return getattr(module, source_cfg["class"])
