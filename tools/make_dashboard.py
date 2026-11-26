#!/usr/bin/env python3
"""Generate a Home Assistant dashboard (YAML) for HA-iDRAC servers.

Usage:  python make_dashboard.py entities.txt [--shutdown] [--server=idrac_<alias>] > idrac_dashboard.yaml
entities.txt: one entity_id per line (Developer Tools > States, filter "idrac").
Uses only built-in Home Assistant cards.
"""
import re
import sys

SKIP = ("update.", "device_tracker.")


def dump(value, indent=0):
    pad = "  " * indent
    lines = []
    if isinstance(value, dict):
        for k, v in value.items():
            if isinstance(v, (dict, list)) and v:
                lines.append(f"{pad}{k}:")
                lines.extend(dump(v, indent + 1))
            else:
                lines.append(f"{pad}{k}: {scalar(v)}")
    else:
        for item in value:
            if isinstance(item, dict) and item:
                sub = dump(item, indent + 1)
                lines.append(f"{pad}- {sub[0].lstrip()}")
                lines.extend(sub[1:])
            else:
                lines.append(f"{pad}- {scalar(item)}")
    return lines


def scalar(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return str(v)
    if v == [] or v == {}:
        return "[]" if v == [] else "{}"
    s = str(v)
    if re.fullmatch(r"[A-Za-z0-9_./ -]+", s) and s.lower() not in ("true", "false", "null", "yes", "no"):
        return s
    return '"' + s.replace('"', '\\"') + '"'


def pretty(alias):
    return alias.replace("_", " ").upper() if len(alias) <= 4 else alias.replace("_", " ").title()


def find(ents, domain, suffix):
    for e in ents:
        if e.startswith(domain + ".") and e.endswith(suffix):
            return e
    return None


def findall(ents, domain, pattern):
    rx = re.compile(pattern)
    return sorted(e for e in ents if e.startswith(domain + ".") and rx.search(e))


def natural(e):
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", e)]


def server_view(prefix, ents, shutdown):
    mine = [e for e in ents if f".{prefix}_" in e]
    f = lambda d, s: find(mine, d, s)
    reach, fault, dprob = f("binary_sensor", "_idrac_reachable"), f("binary_sensor", "_fan_fault"), f("binary_sensor", "_disks_problem")
    psred, fanred = f("binary_sensor", "_ps_redundancy"), f("binary_sensor", "_fan_redundancy")
    power, target = f("sensor", "_power"), f("sensor", "_target_fan_speed")
    hottest, inlet, exhaust = f("sensor", "_hottest_cpu_temp"), f("sensor", "_inlet_temp"), f("sensor", "_exhaust_temp")
    cpus = sorted(findall(mine, "sensor", r"_cpu_\d+_temp(erature)?$"), key=natural)
    fans = sorted(findall(mine, "sensor", r"_fan[a-z0-9_]*_rpm$"), key=natural)
    psus = sorted(findall(mine, "binary_sensor", r"_(psu|ps\d)"), key=natural)
    disks = sorted(findall(mine, "sensor", r"_disk_\d+_health$"), key=natural)
    mon_fail, checked = f("binary_sensor", "_disk_monitoring_failing"), f("sensor", "_disks_last_checked")
    electrical = sorted(findall(mine, "sensor", r"_psu\d+_(voltage|current)$"), key=natural)
    last_poll = f("sensor", "_last_successful_poll")
    button = find(mine, "button", "shutdown_server") or find(mine, "button", "shutdown_button")

    def tile(e, name=None, icon=None, **kw):
        c = {"type": "tile", "entity": e}
        if name:
            c["name"] = name
        if icon:
            c["icon"] = icon
        c.update(kw)
        return c

    def gauge(e, name, lo, hi, green, yellow, red):
        return {"type": "gauge", "entity": e, "name": name, "min": lo, "max": hi, "needle": True,
                "severity": {"green": green, "yellow": yellow, "red": red}}

    sections = []
    status = [{"type": "heading", "heading": "Status", "heading_style": "title", "icon": "mdi:server"}]
    if reach: status.append(tile(reach, "iDRAC", "mdi:lan-connect"))
    if fault: status.append(tile(fault, "Fans", "mdi:fan-alert", state_content=["state", "status"]))
    if dprob: status.append(tile(dprob, "Disks", "mdi:harddisk", state_content=["state", "summary"]))
    if psred: status.append(tile(psred, "PSU redundancy", "mdi:power-plug", state_content=["state", "reading"]))
    if fanred: status.append(tile(fanred, "Fan redundancy", "mdi:fan-chevron-up", state_content=["state", "reading"]))
    if power: status.append(tile(power, "Power", "mdi:flash"))
    if target: status.append(tile(target, "Fan target", "mdi:fan"))
    if button and shutdown:
        status.append(tile(button, "Shutdown", "mdi:power", tap_action={"action": "perform-action", "perform_action": "button.press",
                           "target": {"entity_id": button}, "confirmation": {"text": "Shut down this server?"}}))
    sections.append({"type": "grid", "cards": status})

    temps = [{"type": "heading", "heading": "Temperatures", "icon": "mdi:thermometer"}]
    if hottest: temps.append(gauge(hottest, "Hottest CPU", 20, 90, 0, 60, 75))
    if inlet: temps.append(gauge(inlet, "Inlet", 5, 45, 0, 28, 35))
    if exhaust: temps.append(gauge(exhaust, "Exhaust", 10, 70, 0, 45, 55))
    graph_ents = [{"entity": e} for e in (cpus or ([hottest] if hottest else [])) + ([inlet] if inlet else [])]
    if graph_ents:
        temps.append({"type": "history-graph", "title": "Temperature, 24 h", "hours_to_show": 24, "entities": graph_ents, "grid_options": {"columns": "full"}})
    sections.append({"type": "grid", "cards": temps})

    cool = [{"type": "heading", "heading": "Cooling and power", "icon": "mdi:fan"}]
    ge = [{"entity": e} for e in (target, power) if e]
    if ge:
        cool.append({"type": "history-graph", "title": "Fan target and power, 24 h", "hours_to_show": 24, "entities": ge, "grid_options": {"columns": "full"}})
    if fans:
        cool.append({"type": "entities", "title": "Fan speeds", "entities": [{"entity": e} for e in fans]})
    sections.append({"type": "grid", "cards": cool})

    hw = [{"type": "heading", "heading": "Hardware health", "icon": "mdi:shield-check"}]
    if psus or electrical:
        hw.append({"type": "entities", "title": "Power supplies", "entities": [{"entity": e} for e in psus + electrical]})
    if disks:
        hw.append({"type": "entities", "title": "Disks", "entities": [{"entity": e} for e in disks] +
                   [{"entity": e} for e in (checked, mon_fail) if e]})
    if last_poll:
        hw.append(tile(last_poll, "Last successful poll", "mdi:clock-check-outline"))
    sections.append({"type": "grid", "cards": hw})

    alias = prefix[len("idrac_"):] if prefix.startswith("idrac_") else prefix
    return {"title": pretty(alias), "path": alias.replace("_", "-"), "icon": "mdi:server", "type": "sections",
            "max_columns": 3, "sections": sections}


def main():
    path = sys.argv[1]
    shutdown = "--shutdown" in sys.argv
    ents = [l.strip() for l in open(path) if l.strip() and not l.startswith(SKIP)]
    prefixes = sorted({m.group(1) for e in ents for m in [re.search(r"\.(idrac_[a-z0-9]+(?:_[a-z0-9]+)*?)_(?:hottest_cpu_temp|inlet_temp|power|target_fan_speed|idrac_reachable|fan_fault)$", e)] if m})
    if not prefixes:
        sys.exit("No iDRAC server found in the entity list (expected names like sensor.idrac_<server>_hottest_cpu_temp).")
    only = [a.split("=", 1)[1] for a in sys.argv if a.startswith("--server=")]
    if only:
        prefixes = [p for p in prefixes if p in only]
        if not prefixes:
            sys.exit("None of the --server names were found. Found: " + ", ".join(sorted(set(p for p in prefixes))))
    dash = {"title": "Servers", "views": [server_view(p, ents, shutdown) for p in prefixes]}
    print("\n".join(dump(dash)))


if __name__ == "__main__":
    main()
