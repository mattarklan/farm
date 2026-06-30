def overlaps(a_start, a_end, b_start, b_end) -> bool:
    return not (a_end <= b_start or b_end <= a_start)


def check_time_conflicts(candidate, existing_rules) -> tuple[bool, str]:

    c_resource = candidate["action"]["type"]
    c_start = candidate["schedule"]["start"]
    c_end = candidate["schedule"]["end"]

    for rule in existing_rules.values():

        r = rule["data"]

        if r["action"]["type"] != c_resource:
            continue

        r_start = r["schedule"]["start"]
        r_end = r["schedule"]["end"]

        if overlaps(c_start, c_end, r_start, r_end):
            return False, f"Time conflict on {c_resource}"

    return True, "OK"