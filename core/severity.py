"""
Shared severity scale used on both Events and Actions.

1 = CRITICAL - immediate risk to equipment, crops, or safety; needs eyes now
2 = URGENT   - needs attention soon, not necessarily this second
3 = REVIEW   - worth a human glance when convenient, nothing broken yet
4 = INFO     - FYI / audit trail, no action expected

Plain IntEnum so it's just an int wherever it's stored (YAML, JSON,
dataclasses) but still comparable and self-documenting in code.
"""
from enum import IntEnum


class Severity(IntEnum):
    CRITICAL = 1
    URGENT = 2
    REVIEW = 3
    INFO = 4
