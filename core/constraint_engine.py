from datetime import datetime, timedelta


class ConstraintEngine:

    def __init__(self, config):
        self.constraints = config.get("constraints", [])


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
        raw_violation = self._check_raw(condition, state)

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
    def _check_raw(self, condition, state):

        water = state.get("water", {})
        energy = state.get("energy", {})

        if "battery_soc_min" in condition:
            if energy.get("battery_soc_percent", 100) < condition["battery_soc_min"]:
                return True

        if "water_pressure_min" in condition:
            if water.get("pressure_kpa", 999) < condition["water_pressure_min"]:
                return True

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
        battery = state.get("energy", {}).get("battery_soc_percent", 100)
        if battery < 20:
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
