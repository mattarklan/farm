import json
import paho.mqtt.client as mqtt

from farm.data.repository import insert_raw, upsert_latest, upsert_actuator_ack

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TOPIC = "farm/#"

DEFAULT_CONFIDENCE = 1.0  # placeholder until real confidence scoring exists

# Ack fields aren't ordinary sensor telemetry — see migrations/005 for why
# they're kept out of raw_telemetry/latest_values entirely.
ACK_METRICS = {"last_ack_ts", "ack_status"}

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        asset_id = payload["asset_id"]
        ts = payload["ts"]
        metrics = payload.get("metrics", {})
        confidences = payload.get("confidence", {})

        for metric, value in metrics.items():
            confidence = confidences.get(metric, DEFAULT_CONFIDENCE)
            if metric in ACK_METRICS:
                upsert_actuator_ack(asset_id, metric, value, confidence, ts)
            else:
                insert_raw(asset_id, metric, value, confidence, ts)
                upsert_latest(asset_id, metric, value, confidence, ts)

    except Exception as e:
        print("Error:", e)

def on_connect(client, userdata, flags, rc, properties=None):
    print(f"Connected with result code {rc}")
    client.subscribe(TOPIC)

def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    print("Ingestion service running...")
    client.loop_forever()

if __name__ == "__main__":
    main()