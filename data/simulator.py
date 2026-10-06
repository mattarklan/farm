import time
import random
import json
import sys
import os
import yaml
import paho.mqtt.client as mqtt

from farm.config.loader import load_registry

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TICK_SECONDS = 10

BASE_DIR = os.path.dirname(__file__)
SIM_CONFIG_PATH = os.path.join(BASE_DIR, "sim_config.yaml")

# Pick a scenario at startup: python simulator.py sensor_dropout
ACTIVE_SCENARIO = sys.argv[1] if len(sys.argv) > 1 else None

CONFIDENCE_DROPOUT_PROB = 0.02

SIM_START_TS = time.time()

with open(SIM_CONFIG_PATH) as f:
    CFG = yaml.safe_load(f)

ACTIONS_PATH = os.path.join(BASE_DIR, CFG["paths"]["actions_yaml"])
ACTION_TYPES_PATH = os.path.join(BASE_DIR, CFG["paths"]["action_types_yaml"])
FAULTS_PATH = os.path.join(BASE_DIR, CFG["paths"]["faults_yaml"])
MCU_MAP_PATH = os.path.join(BASE_DIR, CFG["paths"]["mcu_map_yaml"])
NOT_APPLIED_STATUSES = set(CFG["action_log"]["not_applied_statuses"])

try:
    with open(MCU_MAP_PATH) as f:
        _mcu_groups = (yaml.safe_load(f) or {}).get("mcu_groups", {}) or {}
except FileNotFoundError:
    _mcu_groups = {}

_asset_to_mcu = {}
for mcu_id, members in _mcu_groups.items():
    for aid in members:
        _asset_to_mcu[aid] = mcu_id


def mcu_id_for(asset_id):
    return _asset_to_mcu.get(asset_id, f"mcu_{asset_id}")

try:
    with open(ACTION_TYPES_PATH) as f:
        ACTION_TYPES = (yaml.safe_load(f) or {}).get("action_types", {})
except FileNotFoundError:
    print(f"WARNING: action_types.yaml not found at {ACTION_TYPES_PATH} — no actuators will respond")
    ACTION_TYPES = {}

_state = {}          # (asset_id, metric) -> current value, for uncoupled metrics
_tank_litres = {}     # tank_id -> current litres
_channel_state = {}   # (target, channel) -> current commanded value
_channel_last_ts = {} # (target, channel) -> ISO timestamp of the applied action that set it


def _lookup_action_type(name):
    if not name:
        return None
    return ACTION_TYPES.get(name) or ACTION_TYPES.get(name.lower()) or ACTION_TYPES.get(name.upper())


def refresh_actuator_state():
    """Re-read actions.yaml, update _channel_state for anything newer than
    what we've already applied, and return the list of (target, channel)
    keys that just flipped bool False->True this refresh (for momentary
    draw accounting)."""
    try:
        with open(ACTIONS_PATH) as f:
            actions = yaml.safe_load(f) or []
    except FileNotFoundError:
        return []
    if isinstance(actions, dict):
        actions = actions.get("actions", [])

    newly_on = []
    for act in actions:
        target = act.get("target")
        atype = _lookup_action_type(act.get("action_type"))
        if not target or not atype or atype.get("value_type") == "none":
            continue
        if act.get("status") in NOT_APPLIED_STATUSES:
            continue
        channel = atype.get("channel")
        ts = act.get("timestamp") or ""
        key = (target, channel)
        if ts <= _channel_last_ts.get(key, ""):
            continue  # not newer than what we already know — ISO8601 sorts lexically

        if atype["value_type"] == "bool":
            new_val = atype["value"]
        elif atype["value_type"] == "int":
            new_val = (act.get("parameters") or {}).get(atype.get("param"))
        else:
            continue

        was_on = bool(_channel_state.get(key)) if atype["value_type"] == "bool" else False
        _channel_state[key] = new_val
        _channel_last_ts[key] = ts
        if atype["value_type"] == "bool" and new_val and not was_on:
            newly_on.append(key)

    return newly_on


def valve_open(target):
    return bool(_channel_state.get((target, "valve_state"), False))


def pump_on(target):
    return bool(_channel_state.get((target, "pump_state"), False))


def commanded_flow_lpm(target, default_lps):
    v = _channel_state.get((target, "flow_rate_lpm"))
    return v if v is not None else default_lps * 60.0


def load_faults():
    try:
        with open(FAULTS_PATH) as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return []
    return data.get("faults", []) or []


def _find_fault(faults, target, ftype, metric=None):
    for f in faults:
        if f.get("target") != target or f.get("type") != ftype:
            continue
        if metric is not None and f.get("metric") != metric:
            continue
        return f
    return None


def mcu_is_down(faults, mcu):
    for f in faults:
        if f.get("type") == "mcu_down" and f.get("mcu") == mcu:
            return True
    return False


def physical_valve_open(target, faults):
    """What the valve is ACTUALLY doing, vs what was commanded. Diverges
    under actuator_stuck, or freezes entirely if its MCU is down."""
    mcu = mcu_id_for(target)
    frozen_key = (target, "_mcu_down_valve_state")
    if mcu_is_down(faults, mcu):
        if frozen_key not in _state:
            _state[frozen_key] = valve_open(target)  # freeze at last-known state
        return _state[frozen_key]
    _state.pop(frozen_key, None)  # MCU back up — resume following commands

    stuck = _find_fault(faults, target, "actuator_stuck")
    if stuck:
        return str(stuck.get("stuck", "off")).lower() == "on"
    return valve_open(target)


def physical_pump_on(target, faults):
    mcu = mcu_id_for(target)
    frozen_key = (target, "_mcu_down_pump_state")
    if mcu_is_down(faults, mcu):
        if frozen_key not in _state:
            _state[frozen_key] = pump_on(target)
        return _state[frozen_key]
    _state.pop(frozen_key, None)

    stuck = _find_fault(faults, target, "actuator_stuck")
    if stuck:
        return str(stuck.get("stuck", "off")).lower() == "on"
    return pump_on(target)


def has_no_effect(faults, target):
    """Actuator is genuinely in the commanded physical state, but produces
    zero result — dry well, blocked pipe, etc."""
    return _find_fault(faults, target, "no_effect") is not None


def compute_ack(target, faults, now):
    """last_ack_ts / ack_status for one actuator target, or None if no ack
    should be published at all this tick (comms dead).

    Every actuator-bearing fault type can carry an optional `ack:` field:
      - omitted / "ok"   -> acks normally (MCU thinks everything's fine)
      - "none"           -> no ack ever published (comms/MCU dead, this IS
                             core's "command never received" case)
      - any other string -> that fixed ack_status every tick

    Without an explicit `ack:`, no_effect defaults to reporting "ok" for
    `detection_delay_s` seconds (default from config) before switching to
    a reason code — modeling realistic detection lag, not instant fault
    awareness.
    """
    mcu = mcu_id_for(target)
    if mcu_is_down(faults, mcu):
        return None

    stuck = _find_fault(faults, target, "actuator_stuck")
    if stuck:
        ack = stuck.get("ack", "ok")
        return None if ack == "none" else {"last_ack_ts": now, "ack_status": ack}

    no_eff = _find_fault(faults, target, "no_effect")
    since_key = (target, "_no_effect_since")
    if no_eff:
        ack_override = no_eff.get("ack")
        if ack_override == "none":
            return None
        if ack_override:
            return {"last_ack_ts": now, "ack_status": ack_override}
        if since_key not in _state:
            _state[since_key] = now
        delay = no_eff.get("detection_delay_s", CFG["mcu"]["no_effect_detection_delay_s"])
        default_reason = "dry" if target.startswith("bore") else "blocked"
        status = "ok" if (now - _state[since_key]) < delay else no_eff.get("reason", default_reason)
        return {"last_ack_ts": now, "ack_status": status}

    _state.pop(since_key, None)

    no_ack = _find_fault(faults, target, "no_ack")
    if no_ack:
        return None

    delay_fault = _find_fault(faults, target, "ack_delay")
    if delay_fault:
        delay_key = (target, "_ack_delay_since")
        if delay_key not in _state:
            _state[delay_key] = now
        if now - _state[delay_key] < delay_fault.get("delay_s", 5):
            return None
    else:
        _state.pop((target, "_ack_delay_since"), None)

    return {"last_ack_ts": now, "ack_status": "ok"}


def apply_sensor_stuck(value, asset_id, metric, faults):
    f = _find_fault(faults, asset_id, "sensor_stuck", metric=metric)
    if not f:
        return value
    if "value" in f:
        return f["value"]
    frozen_key = (asset_id, metric, "_frozen_value")
    if frozen_key not in _state:
        _state[frozen_key] = value  # freeze at whatever it was when first triggered
    return _state[frozen_key]


def is_raining(elapsed):
    return (elapsed % CFG["rain"]["interval_s"]) < CFG["rain"]["duration_s"]


def build_bore_dependents(registry):
    """bore_id -> [tank_ids that depend on it], from the registry's own
    `dependencies` field. A bore's rated flow is split evenly across
    however many tanks currently list it as a dependency."""
    m = {}
    for aid, a in registry.items():
        if a.get("type") == "tank":
            for dep in a.get("dependencies", []) or []:
                if dep.startswith("bore"):
                    m.setdefault(dep, []).append(aid)
    return m


def step_tank(tank_id, dt_s, bore_dependents, elapsed, faults):
    tcfg = CFG["tank"]
    bcfg = CFG["bore"]
    capacity = tcfg["capacity_l"]
    litres = _tank_litres.get(tank_id, capacity * tcfg["start_pct"] / 100.0)
    pct = litres / capacity * 100.0

    taper = max(0.0, min(1.0, (pct - tcfg["flow_zero_pct"]) /
                          (tcfg["flow_taper_start_pct"] - tcfg["flow_zero_pct"])))
    rated_lps = commanded_flow_lpm(tank_id, tcfg["valve_flow_lps"]) / 60.0
    valve_physically_open = physical_valve_open(tank_id, faults)
    rated_out_lps = rated_lps * taper if valve_physically_open else 0.0

    mismatch = _find_fault(faults, tank_id, "flow_mismatch")
    mode = mismatch.get("mode", "flow_without_level") if mismatch else None

    # real litres always follow the true physics — no_effect and
    # flow_without_level both mean nothing actually leaves the tank
    real_out_lps = 0.0 if (has_no_effect(faults, tank_id) or mode == "flow_without_level") else rated_out_lps

    in_lps = 0.0
    for bore_id, dependents in bore_dependents.items():
        if tank_id in dependents and physical_pump_on(bore_id, faults) and not has_no_effect(faults, bore_id):
            in_lps += bcfg["pump_lps"] / len(dependents)

    rain_lps = 0.0
    if is_raining(elapsed):
        rain_lps = (capacity * CFG["rain"]["tank_gain_pct_per_event"] / 100.0) / CFG["rain"]["duration_s"]

    litres += (in_lps + rain_lps - real_out_lps) * dt_s
    litres = max(0.0, min(capacity, litres))
    _tank_litres[tank_id] = litres

    # the SENSOR reading can lie independently of the real litres above
    if mode == "flow_without_level":
        reported_flow = rated_lps if valve_physically_open else 0.0  # looks normal, isn't
    elif mode == "level_without_flow":
        reported_flow = 0.0  # meter reports nothing despite the real drain above
    else:
        reported_flow = real_out_lps

    return litres / capacity * 100.0, reported_flow


def step_zone(zone_id, dt_s, faults):
    zcfg = CFG["zone"]
    key = (zone_id, "soil_moisture")
    moisture = _state.get(key, zcfg["start_moisture"])
    watering = physical_valve_open(zone_id, faults) and not has_no_effect(faults, zone_id)
    moisture += (zcfg["watering_rate_per_s"] if watering else -zcfg["drying_rate_per_s"]) * dt_s
    moisture = max(0.0, min(1.0, moisture))
    _state[key] = moisture
    return moisture, watering


def step_battery(dt_s, elapsed, bore_on_map, momentary_draw_wh):
    bcfg = CFG["battery"]
    borecfg = CFG["bore"]
    key = ("battery_1", "_energy_wh")
    energy = _state.get(key, bcfg["capacity_wh"] * bcfg["start_pct"] / 100.0)

    cycle = bcfg["solar_cycle_hours"]
    solar_active = (int(elapsed // 3600) // cycle) % 2 == 0
    solar_w = bcfg["solar_charge_wh_per_active_hour"] if solar_active else 0.0

    num_bores_on = sum(1 for v in bore_on_map.values() if v)
    bore_load_w = borecfg["power_w"] * num_bores_on

    net_w = solar_w - bore_load_w
    energy += net_w * (dt_s / 3600.0) - momentary_draw_wh
    energy = max(0.0, min(bcfg["capacity_wh"], energy))
    _state[key] = energy

    soc_pct = energy / bcfg["capacity_wh"] * 100.0
    voltage = bcfg["voltage_empty_v"] + (soc_pct / 100.0) * (bcfg["voltage_full_v"] - bcfg["voltage_empty_v"])
    voltage -= borecfg["voltage_sag_v"] * num_bores_on
    voltage = max(bcfg["voltage_empty_v"] - 4, voltage)
    current = net_w / voltage if voltage else 0.0
    solar_v = bcfg["solar_voltage_active_v"] if solar_active else bcfg["solar_voltage_inactive_v"]

    return {"soc": soc_pct, "voltage": voltage, "current": current,
            "solar_voltage": solar_v, "num_bores_on": num_bores_on}


def _seed(metric, home_gps=None):
    if metric == "latitude":
        return home_gps[0] if home_gps else -31.950300
    if metric == "longitude":
        return home_gps[1] if home_gps else 115.860250
    if metric == "hours":
        return random.uniform(100, 800)
    if "humidity" in metric:
        return random.uniform(30.0, 70.0)
    if "wind_direction" in metric:
        return random.uniform(0.0, 360.0)
    if "voltage" in metric:
        return random.uniform(12.0, 13.5)  # vehicle battery_voltage only — battery_1 is handled separately
    if "wind" in metric:
        return random.uniform(0.0, 20.0)
    if metric == "temperature":
        return random.uniform(CFG["weather"]["temperature_min_c"], CFG["weather"]["temperature_max_c"])
    return 0.0


def apply_scenario(asset_id, metric, val):
    """Only scenarios targeting metrics NOT driven by the physics engine
    still work as simple output overrides. battery_drain / tank1_low /
    drought / rain are retired here — they'd fight the real tank/battery
    state every tick instead of actually draining it. Rebuild those by
    mutating _tank_litres / the battery _energy_wh state directly if
    still wanted."""
    s = ACTIVE_SCENARIO
    if s == "sensor_dropout":
        pass  # handled in fake_confidence()
    elif s == "tractor_overheat" and asset_id == "tractor_100":
        if "voltage" in metric:
            val = random.uniform(13.8, 14.0)
        elif "temperature" in metric:
            val = 200.0
    elif s == "bore_dry_run" and asset_id == "bore_1":
        if metric == "current":
            val = random.uniform(4.0, 5.0)
        elif metric == "flow":
            val = 0.0
    return val


def step_metric(asset_id, metric, home_gps=None):
    """Handles metrics NOT covered by the physics precompute pass this
    tick (tank level/flow_out, zone soil_moisture/soil_temperature,
    battery voltage/soc/current/solar_voltage, bore current/flow — those
    are set directly in main())."""
    key = (asset_id, metric)
    val = _state.get(key, _seed(metric, home_gps))

    if metric in ("latitude", "longitude"):
        home = (home_gps[0] if metric == "latitude" else home_gps[1]) if home_gps else val
        val = val + random.uniform(-0.00002, 0.00002)
        val = max(home - 0.0001, min(home + 0.0001, val))
    elif metric == "hours":
        val = val + (TICK_SECONDS / 3600.0)
    elif "wind_direction" in metric:
        val = (val + random.uniform(-CFG["weather"]["wind_direction_drift_deg_per_tick"],
                                     CFG["weather"]["wind_direction_drift_deg_per_tick"])) % 360.0
    elif "wind" in metric:
        val = max(0.0, min(40.0, val + random.uniform(-1.0, 1.0)))
    elif "humidity" in metric:
        val = max(0.0, min(100.0, val + random.uniform(-1.5, 1.0)))
    elif "ac_current" in metric:
        val = max(0.0, min(20.0, val + random.uniform(-0.5, 0.5)))
    elif "voltage" in metric:
        val = max(10.5, min(14.0, val + random.uniform(-0.05, 0.02)))  # vehicles only
    elif "fuel" in metric:
        val = 50.0
    elif "open" in metric:
        pass  # gate — set directly in main()
    elif metric == "temperature":
        lo, hi = CFG["weather"]["temperature_min_c"], CFG["weather"]["temperature_max_c"]
        val = max(lo, min(hi, val + random.uniform(-0.1, 0.1)))
    else:
        val += random.uniform(-0.1, 0.1)

    val = apply_scenario(asset_id, metric, val)
    _state[key] = val
    return round(val, 6) if metric in ("latitude", "longitude") else round(val, 3)


def fake_confidence(asset_id=None, metric=None):
    if ACTIVE_SCENARIO == "sensor_dropout":
        return 0
    return 1 if random.random() > CONFIDENCE_DROPOUT_PROB else 0


def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()

    registry = load_registry()
    bore_dependents = build_bore_dependents(registry)
    tank_ids = [aid for aid, a in registry.items() if a.get("type") == "tank"]
    bore_ids = [aid for aid, a in registry.items() if a.get("type") == "bore"]
    zone_ids = [aid for aid, a in registry.items() if a.get("type") == "crop"]
    actuator_targets = set(tank_ids) | set(bore_ids) | set(zone_ids)

    print(f"Simulator running with {len(registry)} assets...")
    if ACTIVE_SCENARIO:
        print(f"Scenario active: {ACTIVE_SCENARIO}")
    initial_faults = load_faults()
    if initial_faults:
        print(f"Faults active ({FAULTS_PATH}): {initial_faults}")
    else:
        print(f"No faults active — edit {FAULTS_PATH} any time to trigger one")

    while True:
        now = int(time.time())
        elapsed = now - SIM_START_TS
        dt = TICK_SECONDS

        newly_on = refresh_actuator_state()
        momentary_draw_wh = len(newly_on) * CFG["battery"]["actuator_momentary_wh"]
        faults = load_faults()

        tank_results = {tid: step_tank(tid, dt, bore_dependents, elapsed, faults) for tid in tank_ids}
        zone_results = {zid: step_zone(zid, dt, faults) for zid in zone_ids}
        bore_on_map = {bid: physical_pump_on(bid, faults) for bid in bore_ids}
        battery = step_battery(dt, elapsed, bore_on_map, momentary_draw_wh)

        gate_cfg = CFG["gate"]
        gate_open = 1.0 if gate_cfg["open_at_s"] <= elapsed < gate_cfg["open_at_s"] + gate_cfg["open_duration_s"] else 0.0
        raining = is_raining(elapsed)

        for asset_id, asset in registry.items():
            metrics_payload = {}
            confidence_payload = {}
            home_gps = asset.get("metadata", {}).get("gps")

            if mcu_is_down(faults, mcu_id_for(asset_id)):
                continue  # entire controller offline — nothing published for this asset

            for metric in asset["metrics"]:
                if asset_id in tank_results and metric == "level":
                    value = round(tank_results[asset_id][0], 3)
                elif asset_id in tank_results and metric == "flow_out":
                    value = round(tank_results[asset_id][1], 3)
                elif asset_id in zone_results and metric == "soil_moisture":
                    value = round(zone_results[asset_id][0], 4)
                elif asset_id in zone_results and metric == "soil_temperature":
                    watering = zone_results[asset_id][1]
                    prev = _state.get((asset_id, "soil_temperature"), 22.0)
                    drop = CFG["zone"]["soil_temp_watering_drop_per_tick"] if watering else 0.0
                    value = round(prev - drop + random.uniform(-0.02, 0.02), 3)
                    _state[(asset_id, "soil_temperature")] = value
                elif asset_id in bore_on_map and metric == "current":
                    on = bore_on_map[asset_id]
                    value = round((CFG["bore"]["power_w"] / battery["voltage"]) if on else 0.0, 3)
                elif asset_id in bore_on_map and metric == "flow":
                    on = bore_on_map[asset_id] and not has_no_effect(faults, asset_id)
                    value = round(CFG["bore"]["pump_lps"] + random.uniform(-0.3, 0.3), 3) if on else 0.0
                elif asset_id == "battery_1" and metric == "voltage":
                    value = round(battery["voltage"], 3)
                elif asset_id == "battery_1" and metric == "soc":
                    value = round(battery["soc"], 3)
                elif asset_id == "battery_1" and metric == "current":
                    value = round(battery["current"], 3)
                elif asset_id == "battery_1" and metric == "solar_voltage":
                    value = round(battery["solar_voltage"], 3)
                elif asset_id == "gate_1" and metric == "open":
                    value = gate_open
                elif asset_id == "weather_1" and metric == "rain":
                    value = round(random.uniform(2.0, 6.0), 3) if raining else 0.0
                else:
                    value = step_metric(asset_id, metric, home_gps)

                if value is None:
                    continue
                value = apply_sensor_stuck(value, asset_id, metric, faults)
                metrics_payload[metric] = value
                confidence_payload[metric] = fake_confidence(asset_id, metric)

            if asset_id in actuator_targets:
                ack = compute_ack(asset_id, faults, now)
                if ack:
                    for m, v in ack.items():
                        v = apply_sensor_stuck(v, asset_id, m, faults)
                        metrics_payload[m] = v
                        confidence_payload[m] = fake_confidence(asset_id, m)

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
