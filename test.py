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
from core.sensor_store import SensorStore
from datetime import datetime, timezone
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
# DATABASE + SEED
# -------------------------
db = SensorStore("farm.db")
db.record("tank_1_level", 15.0, datetime.now(timezone.utc))
db.record("tank_2_level", 80.0, datetime.now(timezone.utc))

# -------------------------
# STATE
# -------------------------
state = StateBuilder().build(db, registry)

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
print("\nEVENTS:")
for ev in result["events"]:
    # Inserts a newline right before the data dictionary starts
    formatted_event = repr(ev).replace(", timestamp=", ",\n timestamp=")
    print(f" [{formatted_event}]")
print("\nACTIONS:\n", result["actions"])
print("\nALLOWED:\n", result["allowed"])
print("\nBLOCKED:\n", result["blocked"])
print("\nDELAYED:\n", result["delayed"])
print("\nRESULTS:\n", result["results"])