import json
import time
import paho.mqtt.client as mqtt

from farm.db.repository import insert_raw, upsert_latest


MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TOPIC = "farm/#"


# ----------------------------
# Normalisation rules
# ----------------------------
def normalize_metric(key: str) -> str:
    # enforce canonical naming early
    mapping = {
        "moisture": "soil.moisture",
        "level": "water.level",
        "distance_km": "vehicle.distance",
        "running": "bore.running",
    }
    return mapping.get(key, key)


def compute_quality(value, confidence):
    if confidence < 0.5:
        return "BAD"
    if confidence < 0.8:
        return "DEGRADED"
    return "GOOD"


def compute_confidence(payload, metric, value):
    # simple placeholder model (replace later with real sensor health logic)
    base = 1.0

    if value is None:
        return 0.0

    # crude heuristics for simulator realism
    if metric.startswith("soil") and value < 5:
        base -= 0.2

    if metric.startswith("water") and value < 0:
        base = 0.0

    return max(0.0, min(1.0, base))


# ----------------------------
# MQTT handler
# ----------------------------
def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())

        asset_id = payload["id"]
        ts = payload["ts"]

        for k, v in payload.items():
            if k in ("id", "type", "ts"):
                continue

            metric = normalize_metric(k)
            value = float(v)

            confidence = compute_confidence(payload, metric, value)
            quality = compute_quality(value, confidence)

            insert_raw(
                asset_id=asset_id,
                metric=metric,
                value=value,
                confidence=confidence,
                quality=quality,
                ts=ts
            )

            upsert_latest(
                asset_id=asset_id,
                metric=metric,
                value=value,
                confidence=confidence,
                quality=quality,
                ts=ts
            )

    except Exception as e:
        print("ingestion error:", e)


# ----------------------------
# MQTT setup
# ----------------------------
def main():
    client = mqtt.Client()
    client.on_message = on_message

    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe(TOPIC)

    print("MQTT ingestion running...")

    client.loop_forever()


if __name__ == "__main__":
    main()