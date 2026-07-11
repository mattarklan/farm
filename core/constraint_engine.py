from datetime import datetime, timedelta


class ConstraintEngine:

    def __init__(self, config, action_types_config=None):
        self.constraints = config.get("constraints", [])
        self.action_types = (action_types_config or {}).get("action_types", {})


    def evaluate(self, actions, state, derived):

        allowed = []
        blocked = []
        delayed = []

        for action in actions:

            decision = self._evaluate_action(action, state, derived)

            if decision["status"] == "allow":
                allowed.append(action)

            elif decision["status"] == "block":
                blocked.append({
                    "action": action,
                    "reason": decision["reason"],
                    "signal": decision.get("signal")
                })

            elif decision["status"] == "delay":
                action.timestamp = datetime.utcnow() + timedelta(
                    seconds=decision.get("delay_seconds", 0)
                )
                delayed.append(action)

        return allowed, blocked, delayed


    def _evaluate_action(self, action, state, derived):

        spec = self.action_types.get(action.action_type) or self.action_types.get(action.action_type.upper())
        if spec and spec.get("always_allow"):
            # Reducing risk (turning something off, shedding load,
            # logging) should never be something a constraint can
            # prevent -- see action_types.yaml's always_allow note.
            return {"status": "allow"}

        for rule in self.constraints:

            if rule["action_type"] != action.action_type:
                continue

            result = self._evaluate_rule(rule, action, state, derived)

            if result["status"] != "allow":
                return result

        return {"status": "allow"}


    def _evaluate_rule(self, rule, action, state, derived):

        condition = rule.get("condition", {})

        mode = rule.get("mode", "block")

        # -----------------------------
        # RAW STATE CHECKS
        # -----------------------------
        raw_violation = self._check_raw(condition, state, action)

        # -----------------------------
        # DERIVED CHECKS
        # -----------------------------
        derived_violation = self._check_derived(condition, derived)

        # -----------------------------
        # RISK AGGREGATION
        # -----------------------------
        risk_score = self._compute_risk(condition, state, derived)

        # -----------------------------
        # DECISION LOGIC
        # -----------------------------

        if risk_score >= 0.8:
            return {
                "status": "block",
                "reason": "High systemic risk detected",
                "signal": risk_score
            }

        if derived_violation and mode == "block":
            return {
                "status": "block",
                "reason": rule.get("reason", "Derived constraint violated"),
                "signal": "derived"
            }

        if raw_violation and mode == "block":
            return {
                "status": "block",
                "reason": rule.get("reason", "Raw constraint violated"),
                "signal": "raw"
            }

        if risk_score >= 0.6 and mode == "delay":
            return {
                "status": "delay",
                "delay_seconds": rule.get("delay_seconds", 30),
                "signal": risk_score
            }

        return {"status": "allow"}


    # -----------------------------
    # RAW STATE EVALUATION
    # -----------------------------
    def _check_raw(self, condition, state, action):
        """
        Was: state.get("water", {})/state.get("energy", {}) -- the real
        state dict has no such nested keys (it's flat, state["sensors"][id]),
        so this always returned False regardless of real readings.

        Now: {sensor, op, value} directly against state["sensors"],
        same shape as goal_engine's `until` sensor/constraint conditions.
        "{target}" in the sensor name is substituted with the action's
        own target, so one constraint can apply to every asset a given
        action_type might run against instead of one hardcoded sensor_id.
        """
        if "sensor" not in condition:
            return False  # not a raw-sensor condition (e.g. a derived-only or risk-only rule)

        sensor_id = condition["sensor"]
        if "{target}" in sensor_id:
            if not action.target:
                return False
            sensor_id = sensor_id.replace("{target}", action.target)

        sensor_state = state["sensors"].get(sensor_id)
        if sensor_state is None:
            return False

        return self._compare(sensor_state.value, condition.get("op", "lt"), condition["value"])

    @staticmethod
    def _compare(left, op, right):
        if op == "lt": return left < right
        if op == "lte": return left <= right
        if op == "gt": return left > right
        if op == "gte": return left >= right
        if op == "eq": return left == right
        if op == "neq": return left != right
        return False


    # -----------------------------
    # DERIVED EVALUATION
    # -----------------------------
    def _check_derived(self, condition, derived):

        for sensor_id, rules in condition.get("derived", {}).items():

            sensor_data = derived.get(sensor_id, {})

            for key, rule in rules.items():

                value = sensor_data.get(key)

                if value is None:
                    continue

                op = rule.get("op")
                target = rule.get("value")

                if op == "lt" and not value < target:
                    return False

                if op == "gt" and not value > target:
                    return False

                if op == "eq" and not value == target:
                    return False

        return True


    # -----------------------------
    # RISK MODEL (simple but extensible)
    # -----------------------------
    def _compute_risk(self, condition, state, derived):

        risk = 0.0

        # battery stress
        # Was: state.get("energy", {}).get("battery_soc_percent", 100) --
        # same bug _check_raw used to have. state is flat
        # (state["sensors"][sensor_id]), there's no "energy" key, so this
        # always silently fell back to 100 and could never fire.
        # Battery SOC readings are a loose guess (low confidence) right
        # now, so this only counts a "_soc" sensor if it's actually
        # trusted (confidence > 0) -- same "don't judge off an untrusted
        # reading" rule every other engine follows. Takes the worst
        # (lowest) trusted SOC across all battery sensors rather than
        # assuming there's exactly one.
        battery_socs = [
            s.value for sid, s in state["sensors"].items()
            if sid.endswith("_soc") and s.confidence > 0
        ]
        if battery_socs and min(battery_socs) < 20:
            risk += 0.4

        # rapid decline risk
        for sensor_id, d in derived.items():
            slope = d.get("slope", 0)
            volatility = d.get("volatility", 0)

            if slope < -0.3:
                risk += 0.3

            if volatility > 25:
                risk += 0.2

        return min(1.0, risk)
