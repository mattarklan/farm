#engine.py

from core.state import FarmState
from core.events import EventEngine
from core.rules import RuleEngine
from core.executor import Executor
from core.constraints import ConstraintEngine
from core.logger import Logger
import yaml

class FarmEngine:
    def __init__(self, rules_path: str = "config/rules.yaml"):
        self.state = FarmState()
        self.event_engine = EventEngine()
        with open(rules_path, "r", encoding="utf-8") as f:
            rules_config = yaml.safe_load(f)
        self.rule_engine = RuleEngine(rules_config)
        self.constraint_engine = ConstraintEngine()
        self.executor = Executor()
        self.logger = Logger()

    def step(self):
        # 1. EVENTS
        events = self.event_engine.process(self.state)
        # 2. RULES → ACTIONS
        actions = self.rule_engine.process(events, self.state)
        # 3. CONSTRAINT FILTER
        safe_actions = self.constraint_engine.process(actions, self.state)
        # 4. EXECUTE
        self.executor.process(safe_actions)
        # 5. LOG
        self.logger.log(self.state, events, safe_actions)