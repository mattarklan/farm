from core.loader import RegistryLoader
from core.state_builder import StateBuilder
from core.event_engine import EventEngine
from core.rule_engine import RuleEngine
from core.constraint_engine import ConstraintEngine
from core.executor import Executor
from core.pipeline import Pipeline
from core.derived import DerivedEngine
from core.window_store import RollingWindowStore
from core.decision_memory import DecisionMemory
import yaml

# -------------------------
# LOAD CONFIG
# -------------------------
loader = RegistryLoader()
registry = loader.load("config/registry/water.yaml")

with open("config/events.yaml") as f:
    event_rules = yaml.safe_load(f)

with open("config/constraints.yaml") as f:
    constraint_config = yaml.safe_load(f)

# -------------------------
# STATE
# -------------------------
state = StateBuilder().build(registry)
state["sensors"]["tank_1_level"].value = 15
state["sensors"]["tank_2_level"].value = 80

# -------------------------
# COMPONENTS
# -------------------------
window_store = RollingWindowStore()
event_engine = EventEngine(registry, event_rules)
rule_engine = RuleEngine(event_rules)
constraint_engine = ConstraintEngine(constraint_config)
executor = Executor()
derived_engine = DerivedEngine(window_store)
decision_memory = DecisionMemory()

# -------------------------
# PIPELINE
# -------------------------
pipeline = Pipeline(
    event_engine,
    rule_engine,
    constraint_engine,
    executor,
    derived_engine,
    window_store,
    decision_memory
)

result = pipeline.run(state)

# -------------------------
# OUTPUT
# -------------------------
print("\nEVENTS:\n", result["events"])
print("\nACTIONS:\n", result["actions"])
print("\nALLOWED:\n", result["allowed"])
print("\nBLOCKED:\n", result["blocked"])
print("\nDELAYED:\n", result["delayed"])
print("\nRESULTS:\n", result["results"])