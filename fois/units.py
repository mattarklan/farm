"""Unit parsing and normalisation for price comparison.

Suppliers list prices per bag in whatever unit they feel like (25L bag,
40L bag, 50kg bag...) which makes "cheapest" meaningless to compare
directly - a $7.70 25L bag isn't cheaper than an $11.00 40L bag, it's
just smaller. This module converts a (value, unit) pair into a common
basis so they actually can be compared.

Two normalised bases, deliberately kept separate:

- price_per_m3: exact. Litres -> cubic metres is fixed unit math, no
  assumptions involved. Always computable for any volume-based unit.
- price_per_tonne: an ESTIMATE for volume-based products, because it
  requires a bulk density (kg per litre) to convert volume to weight,
  and bulk density varies by moisture, compaction, and the specific
  product. Only computed when a density is configured (see
  config/fois/links.yaml's `densities` section), and always flagged as
  an estimate on the result - never presented as if it were exact.
  For products already priced by weight (kg/tonne), it IS exact.
"""

import re
from dataclasses import dataclass
from typing import Optional

_VOLUME_TO_LITRES = {"ml": 0.001, "l": 1.0}
_MASS_TO_KG = {"g": 0.001, "kg": 1.0, "t": 1000.0, "tonne": 1000.0, "tonnes": 1000.0}

# e.g. "25L_bag", "40L", "20kg_bag", "kg", "1.5t"
_UNIT_RE = re.compile(
    r'^(\d+(?:\.\d+)?)\s*(ml|l|kg|g|tonnes|tonne|t)(?:_?\w*)?$',
    re.IGNORECASE,
)


@dataclass
class ParsedUnit:
    kind: str  # "volume" or "mass"
    litres: Optional[float] = None
    kg: Optional[float] = None


def parse_unit(unit_str: str) -> Optional[ParsedUnit]:
    """Parse a unit string like '25L_bag' into a quantity fois can do
    maths with. Returns None if the format isn't recognised (e.g. 'bale',
    'packet' - anything without a parseable quantity+unit)."""
    if not unit_str:
        return None
    match = _UNIT_RE.match(unit_str.strip())
    if not match:
        return None
    quantity = float(match.group(1))
    suffix = match.group(2).lower()

    if suffix in _VOLUME_TO_LITRES:
        return ParsedUnit(kind="volume", litres=quantity * _VOLUME_TO_LITRES[suffix])
    if suffix in _MASS_TO_KG:
        return ParsedUnit(kind="mass", kg=quantity * _MASS_TO_KG[suffix])
    return None


def price_per_m3(value: float, unit_str: str) -> Optional[float]:
    """Exact - no density assumption needed, just litres -> m3."""
    parsed = parse_unit(unit_str)
    if parsed is None or parsed.kind != "volume" or not parsed.litres:
        return None
    cubic_metres = parsed.litres / 1000.0
    return value / cubic_metres if cubic_metres > 0 else None


def price_per_tonne(value: float, unit_str: str, density_kg_per_litre: Optional[float] = None) -> Optional[float]:
    """Exact for mass-priced units. For volume-priced units, requires a
    density and is an ESTIMATE - caller is responsible for flagging that
    (see api.py, which sets price_per_tonne_is_estimate on the response)."""
    parsed = parse_unit(unit_str)
    if parsed is None:
        return None

    if parsed.kind == "mass" and parsed.kg:
        tonnes = parsed.kg / 1000.0
        return value / tonnes if tonnes > 0 else None

    if parsed.kind == "volume" and parsed.litres and density_kg_per_litre:
        kg = parsed.litres * density_kg_per_litre
        tonnes = kg / 1000.0
        return value / tonnes if tonnes > 0 else None

    return None
