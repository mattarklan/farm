import time
import random
import json
import sys
import paho.mqtt.client as mqtt

from farm.config.loader import load_registry

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TICK_SECONDS = 10

# Pick a scenario at startup: python simulator.py rain
ACTIVE_SCENARIO = sys.argv[1] if len(sys.argv) > 1 else None

# MCU can flag "no reading / open circuit" but can't judge sensor accuracy,
# so confidence is now a hard 1/0 rather than a graded score.
CONFIDENCE_DROPOUT_PROB = 0.02

_state = {}  # (asset_id, metric) -> current value


def _seed(metric, home_gps=None):
    if metric == "latitude":
        return home_gps[0] if home_gps else -31.950300
    if metric == "longitude":
        return home_gps[1] if home_gps else 115.860250
    if metric == "hours":
        return random.uniform(100, 800)  # engine-hour meter starting point
    if "solar_voltage" in metric:
        return random.uniform(20.0, 35.0)
    if "ac_current" in metric:
        return random.uniform(0.0, 10.0)
    if "humidity" in metric:
        return random.uniform(30.0, 70.0)
    if "wind" in metric:
        return random.uniform(0.0, 20.0)
    if "moisture" in metric:
        return random.uniform(0.1, 0.6)
    if "level" in metric:
        return random.uniform(30.0, 90.0)
    if "soc" in metric:
        return random.uniform(40, 80)
    if "voltage" in metric:
        return random.uniform(12.0, 13.5)
    if "temperature" in metric:
        return random.uniform(15, 30)
    return 0.0


def apply_scenario(asset_id, metric, val):
    """Post-process override for named test scenarios. Add new scenarios
    here as new elif branches — keep each one to a couple of lines."""
    s = ACTIVE_SCENARIO
    if s == "battery_drain" and asset_id == "battery_1":
        if "voltage" in metric:
            val = val - random.uniform(0.02, 0.05)  # no floor — drive it low
        elif "soc" in metric:
            val = val - random.uniform(0.3, 0.8)
    elif s == "tank1_low" and asset_id == "tank_1" and "level" in metric:
        val = max(5.0, val - random.uniform(0.3, 0.8))
    elif s == "rain":
        if asset_id == "weather_1" and "rain" in metric:
            val = random.uniform(5.0, 15.0)
        elif "moisture" in metric:
            val = min(0.6, val + random.uniform(0.005, 0.02))
    elif s == "sensor_dropout":
        pass  # handled in fake_confidence(), not here
    elif s == "tractor_overheat" and asset_id == "tractor_100":
        if "voltage" in metric:
            val = random.uniform(13.8, 14.0)  # regulator stuck high
        elif "temperature" in metric:
            val = 200.0  # way past the normal 60 ceiling
    elif s == "bore_dry_run" and asset_id == "bore_1":
        if "current" in metric:
            val = random.uniform(4.0, 5.0)  # motor still drawing current
        elif "flow" in metric:
            val = 0.0  # no output despite drawing current — dry run / blockage
    elif s == "drought":
        if asset_id == "zone_a" and "moisture" in metric:
            val = random.uniform(0.1, 0.15)
        elif asset_id == "tank_1" and "level" in metric:
            val = val - random.uniform(0.3, 0.8)  # gradual drain, no floor
        elif asset_id == "tank_1" and "flow" in metric:
            val = 5.0  # stuck/out-of-range reading
    return val


def step_metric(asset_id, metric, home_gps=None):
    key = (asset_id, metric)
    val = _state.get(key, _seed(metric, home_gps))

    if metric in ("latitude", "longitude"):
        home = (home_gps[0] if metric == "latitude" else home_gps[1]) if home_gps else val
        val = val + random.uniform(-0.00002, 0.00002)  # non-RTK drift
        val = max(home - 0.0001, min(home + 0.0001, val))  # bounded wobble, ~11m box
    elif metric == "hours":
        val = val + (TICK_SECONDS / 3600.0)  # sensor-group uptime, always accrues
    elif "rain" in metric:
        val = random.uniform(0.5, 8.0) if random.random() > 0.9 else 0.0
    elif "wind" in metric:
        val = max(0.0, min(40.0, val + random.uniform(-1.0, 1.0)))
    elif "humidity" in metric:
        val = max(0.0, min(100.0, val + random.uniform(-1.5, 1.0)))
    elif asset_id.startswith("bore") and "current" in metric:
        val = random.uniform(3.5, 5.5) if random.random() > 0.05 else 0.0
    elif "flow" in metric:
        val = max(0.0, random.uniform(0.0, 4.0))
    elif "fuel" in metric:
        val = 50.0  # constant for now
    elif "ac_current" in metric:
        val = max(0.0, min(20.0, val + random.uniform(-0.5, 0.5)))
    elif "current" in metric:
        val = random.uniform(-5.0, 5.0)
    elif "solar_voltage" in metric:
        val = max(0.0, min(40.0, val + random.uniform(-1.0, 1.0)))
    elif "moisture" in metric:
        val = max(0.1, min(0.6, val + random.uniform(-0.01, 0.01)))
    elif "level" in metric:
        val = max(30.0, min(90.0, val + random.uniform(-0.5, 0.5)))
    elif "soc" in metric:
        val = max(0.0, min(100.0, val + random.uniform(-1.5, 1.0)))
    elif "voltage" in metric:
        val = max(10.5, min(14.0, val + random.uniform(-0.05, 0.02)))
    elif "open" in metric:
        val = 1.0 if random.random() > 0.9 else 0.0  # mostly closed
    elif "temperature" in metric:
        val = max(-5.0, min(60.0, val + random.uniform(-0.5, 0.5)))
    else:
        val += random.uniform(-0.1, 0.1)

    val = apply_scenario(asset_id, metric, val)
    _state[key] = val
    return round(val, 6) if metric in ("latitude", "longitude") else round(val, 3)


def fake_confidence(asset_id=None, metric=None):
    # MCU-level "did I get a reading at all" flag, not a sensor-accuracy score.
    if ACTIVE_SCENARIO == "sensor_dropout":
        return 0
    return 1 if random.random() > CONFIDENCE_DROPOUT_PROB else 0


def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()

    registry = load_registry()
    print(f"Simulator running with {len(registry)} assets...")
    if ACTIVE_SCENARIO:
        print(f"Scenario active: {ACTIVE_SCENARIO}")

    while True:
        now = int(time.time())
        for asset_id, asset in registry.items():
            metrics_payload = {}
            confidence_payload = {}
            home_gps = asset.get("metadata", {}).get("gps")
            for metric in asset["metrics"]:
                value = step_metric(asset_id, metric, home_gps)
                if value is None:
                    continue
                metrics_payload[metric] = value
                confidence_payload[metric] = fake_confidence(asset_id, metric)

            if not metrics_payload:
                continue

            payload = {
                "asset_id": asset_id,
                "type": asset["type"],
                "ts": now,
                "metrics": metrics_payload,
                "confidence": confidence_payload,
            }
            topic = f"farm/{asset['type']}/{asset_id}"
            client.publish(topic, json.dumps(payload))

        time.sleep(TICK_SECONDS)


if __name__ == "__main__":
    main()
