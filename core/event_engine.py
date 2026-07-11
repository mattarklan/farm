#event_engine.py
import uuid
from core.models import Event


class EventEngine:

    def __init__(self, registry, event_rules):
        self.registry = registry
        self.rules = event_rules["rules"]


    def process(self, state, derived):

        events = []

        sensors = state["sensors"]

        for sensor_id, sensor_state in sensors.items():

            if sensor_state.confidence <= 0:
                # No legitimate signal. ConfidenceMonitor raises its own
                # event for this separately -- don't also let a garbage
                # reading fire a threshold event (e.g. a dead sensor
                # reporting 0 shouldn't look like "soil moisture is 0").
                continue

            value = sensor_state.value
            asset = self.registry.get_asset_from_sensor(sensor_id)

            if asset is None:
                continue

            sensor_derived = derived.get(sensor_id, {})

            for rule in self.rules:

                # --------------------------
                # 1. asset match
                # --------------------------
                if rule["asset_type"] != asset.type:
                    continue

                # --------------------------
                # 2. sensor match
                # --------------------------
                if not sensor_id.endswith(rule["sensor_suffix"]):
                    continue

                # --------------------------
                # 3. raw condition
                # --------------------------
                if not self._eval_raw(rule.get("condition", {}), value):
                    continue

                # --------------------------
                # 4. derived condition (optional)
                # --------------------------
                if "derived" in rule:

                    if not self._eval_derived(rule["derived"], sensor_derived):
                        continue

                # --------------------------
                # 5. confidence scoring
                # --------------------------
                confidence = self._compute_confidence(value, sensor_derived)

                events.append(
                    Event(
                        id=str(uuid.uuid4()),
                        event_type=rule["event_id"],
                        asset_id=asset.id,
                        sensor_id=sensor_id,
                        severity=rule["severity"],
                        confidence=confidence,
                        timestamp=state["timestamp"],
                        data={
                            "value": value,
                            "derived": sensor_derived,
                            "rule_id": rule["id"]
                        }
                    )
                )

        return events


    # --------------------------
    # RAW VALUE EVAL
    # --------------------------
    def _eval_raw(self, condition, value):

        if not condition:
            return True

        op = condition.get("op")
        target = condition.get("value")

        if op == "lt":
            return value < target
        if op == "lte":
            return value <= target
        if op == "gt":
            return value > target
        if op == "gte":
            return value >= target
        if op == "eq":
            return value == target

        return False


    # --------------------------
    # DERIVED EVAL
    # --------------------------
    def _eval_derived(self, derived_rule, derived_values):

        for key, cond in derived_rule.items():

            actual = derived_values.get(key)

            if actual is None:
                return False

            op = cond.get("op")
            target = cond.get("value")

            if op == "lt" and not actual < target:
                return False
            if op == "gt" and not actual > target:
                return False
            if op == "eq" and not actual == target:
                return False

        return True


    # --------------------------
    # SIMPLE CONFIDENCE MODEL
    # --------------------------
    def _compute_confidence(self, value, derived):

        confidence = 0.7  # baseline

        slope = derived.get("slope", 0)
        volatility = derived.get("volatility", 0)

        # stronger signal if trend agrees
        if slope < -0.1:
            confidence += 0.2

        # reduce confidence if noisy
        if volatility > 20:
            confidence -= 0.2

        # clamp
        return max(0.0, min(1.0, confidence))
