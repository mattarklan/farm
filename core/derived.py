from datetime import datetime


class DerivedEngine:

    def __init__(self, window_store):
        self.window_store = window_store


    def compute(self, sensor_ids, now=None):

        now = now or datetime.utcnow()

        derived = {}

        for sensor_id in sensor_ids:

            history = self.window_store.get(sensor_id)

            if len(history) < 2:
                continue

            # unpack values
            values = [v for (_, v) in history]

            # --------------------------
            # RATE OF CHANGE
            # --------------------------
            delta = values[-1] - values[0]

            time_delta = (history[-1][0] - history[0][0]).total_seconds()

            if time_delta > 0:
                slope = delta / time_delta
            else:
                slope = 0

            # --------------------------
            # MOVING AVERAGE
            # --------------------------
            avg = sum(values) / len(values)

            # --------------------------
            # SIMPLE VOLATILITY
            # --------------------------
            volatility = max(values) - min(values)

            derived[sensor_id] = {
                "slope": slope,
                "avg": avg,
                "volatility": volatility,
                "last": values[-1],
            }

        return derived