import time
import random
import json
import paho.mqtt.client as mqtt

from farm.config.loader import load_registry

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TICK_SECONDS = 10

SKIP_METRICS = {"last_irrigated"}  # not a live telemetry value

_state = {}  # (asset_id, metric) -> current value


def _seed(metric):
    if any(k in metric for k in ("level", "soc", "moisture", "fuel")):
        return random.uniform(40, 80)
    if "voltage" in metric:
        return random.uniform(12.0, 13.5)
    if "pressure" in metric:
        return random.uniform(2.0, 5.0)
    if "temperature" in metric:
        return random.uniform(15, 30)
    return 0.0


def step_metric(asset_id, metric):
    if metric in SKIP_METRICS:
        return None
    key = (asset_id, metric)
    val = _state.get(key, _seed(metric))

    if "status" in metric:
        val = 1.0 if random.random() > 0.05 else 0.0
    elif "flow" in metric:
        val = max(0.0, random.uniform(0.0, 4.0))
    elif "current" in metric:
        val = random.uniform(-5.0, 5.0)
    elif any(k in metric for k in ("level", "soc", "moisture", "fuel")):
        val = max(0.0, min(100.0, val + random.uniform(-1.5, 1.0)))
    elif "voltage" in metric:
        val = max(10.5, min(14.0, val + random.uniform(-0.05, 0.02)))
    elif "pressure" in metric:
        val = max(0.0, min(8.0, val + random.uniform(-0.2, 0.2)))
    elif "temperature" in metric:
        val = max(-5.0, min(60.0, val + random.uniform(-0.5, 0.5)))
    else:
        val += random.uniform(-0.1, 0.1)

    _state[key] = val
    return round(val, 3)


def fake_confidence():
    # placeholder — see confidence discussion; real scoring needs cross-sensor comparison
    return round(random.uniform(0.85, 1.0), 3)


def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()

    registry = load_registry()
    print(f"Simulator running with {len(registry)} assets...")

    while True:
        now = int(time.time())
        for asset_id, asset in registry.items():
            metrics_payload = {}
            confidence_payload = {}
            for metric in asset["metrics"]:
                value = step_metric(asset_id, metric)
                if value is None:
                    continue
                metrics_payload[metric] = value
                confidence_payload[metric] = fake_confidence()

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