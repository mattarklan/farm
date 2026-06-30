from dataclasses import dataclass
from typing import Any, Optional


@dataclass(slots=True)
class Rule:
    rule_id: str
    version: int
    data: dict[str, Any]
    source_proposal: str


@dataclass(slots=True)
class Decision:
    status: str   # APPROVED | REJECTED | MODIFIED
    reason: str
    rule: Optional[Rule] = None