import datetime
import re
import subprocess
import threading
import time

TABLE_OID = "1.3.6.1.4.1.674.10892.5.5.1.20.130.4.1"

COLUMNS = {
    2: "name", 3: "manufacturer", 4: "state", 6: "model", 7: "serial", 8: "firmware",
    11: "size_mb", 24: "status", 31: "smart_alert", 33: "manufactured_week",
    34: "manufactured_year", 35: "media_type", 49: "endurance_remaining",
    54: "fqdn", 55: "display_name",
}

STATE_NAMES = {1: "unknown", 2: "ready", 3: "online", 4: "foreign", 5: "offline", 6: "blocked",
               7: "failed", 8: "non_raid", 9: "removed", 10: "read_only"}
STATUS_NAMES = {1: "other", 2: "unknown", 3: "ok", 4: "non_critical", 5: "critical", 6: "non_recoverable"}
MEDIA_NAMES = {1: "unknown", 2: "HDD", 3: "SSD"}

HEALTH_LABELS = {
    "ok": "OK",
    "warning": "Warning",
    "predicted_failure": "Predicted failure",
    "critical": "Critical",
    "failed": "Failed",
    "offline": "Offline",
    "unknown": "Unknown",
}
HEALTH_RANK = {"ok": 0, "unknown": 1, "warning": 2, "offline": 3, "predicted_failure": 4, "critical": 5, "failed": 6}

_LINE = re.compile(
    r'^(?:\.1|iso|1)\.3\.6\.1\.4\.1\.674\.10892\.5\.5\.1\.20\.130\.4\.1\.(\d+)\.(\d+)\s*=\s*([A-Za-z0-9-]+):\s*(.*)$'
)


class DiskPollError(Exception):
    pass


def _value(kind, raw):
    raw = raw.strip()
    if kind.upper() in ("INTEGER", "GAUGE32", "COUNTER32", "UNSIGNED32"):
        match = re.match(r'-?\d+', raw)
        return int(match.group(0)) if match else None
    if raw.startswith('"') and raw.endswith('"') and len(raw) >= 2:
        raw = raw[1:-1]
    return raw


def parse_walk(text):
    raw = {}
    for line in text.splitlines():
        match = _LINE.match(line.strip())
        if not match:
            continue
        column, index, kind, value = int(match.group(1)), int(match.group(2)), match.group(3), match.group(4)
        field = COLUMNS.get(column)
        if field:
            raw.setdefault(index, {})[field] = _value(kind, value)
    return {index: _build_disk(index, fields) for index, fields in sorted(raw.items())}


def _int_or_none(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def classify(state, status, smart_alert):
    if state == 7 or status == 6:
        return "failed"
    if status == 5:
        return "critical"
    if smart_alert:
        return "predicted_failure"
    if state in (5, 6, 9):
        return "offline"
    if status == 4 or state in (4, 10):
        return "warning"
    if state not in STATE_NAMES or status not in STATUS_NAMES or state == 1 or status in (1, 2):
        return "unknown"
    return "ok"


def _build_disk(index, fields):
    state = _int_or_none(fields.get("state"))
    status = _int_or_none(fields.get("status"))
    smart = _int_or_none(fields.get("smart_alert"))
    media = _int_or_none(fields.get("media_type"))
    endurance = _int_or_none(fields.get("endurance_remaining"))
    size_mb = _int_or_none(fields.get("size_mb"))
    health = classify(state, status, smart)
    smart_text = "none" if not smart else ("ssd_wear_out" if smart == 2 else "predicted_failure")
    return {
        "index": index,
        "name": fields.get("display_name") or fields.get("name") or f"Disk {index}",
        "slot": fields.get("name"),
        "fqdn": fields.get("fqdn"),
        "manufacturer": fields.get("manufacturer"),
        "model": fields.get("model"),
        "serial": fields.get("serial"),
        "firmware": fields.get("firmware"),
        "size_gb": round(size_mb / 1024) if size_mb else None,
        "media_type": MEDIA_NAMES.get(media, "unknown"),
        "raid_state": STATE_NAMES.get(state, "unknown"),
        "component_status": STATUS_NAMES.get(status, "unknown"),
        "smart_alert": smart_text,
        "endurance_remaining": endurance if endurance is not None and 0 <= endurance <= 100 else None,
        "manufactured": _manufactured(fields),
        "health": health,
        "health_label": HEALTH_LABELS[health],
    }


def _manufactured(fields):
    year = _int_or_none(fields.get("manufactured_year"))
    week = _int_or_none(fields.get("manufactured_week"))
    if not year:
        return None
    return f"{year}-W{week:02d}" if week else str(year)


def summarize(disks):
    items = list(disks.values()) if isinstance(disks, dict) else list(disks)
    worst = max(items, key=lambda d: HEALTH_RANK[d["health"]]) if items else None
    problems = [d for d in items if d["health"] != "ok"]
    return {
        "count": len(items),
        "problem_count": len(problems),
        "worst": worst["health"] if worst else "unknown",
        "has_problem": bool(problems),
        "problems": [f'{d["name"]}: {d["health_label"]}' for d in problems],
    }


def walk_physical_disks(host, community, port=161, timeout=3, retries=1):
    command = ["snmpwalk", "-v2c", "-c", community, "-On", "-t", str(timeout), "-r", str(retries),
               f"{host}:{port}", TABLE_OID]
    try:
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=timeout * (retries + 1) + 10)
    except FileNotFoundError:
        raise DiskPollError("snmpwalk is not installed in this container")
    except subprocess.TimeoutExpired:
        raise DiskPollError("snmpwalk timed out")
    if result.returncode != 0:
        message = (result.stderr or result.stdout).strip().splitlines()
        raise DiskPollError(message[0] if message else f"snmpwalk exited with {result.returncode}")
    disks = parse_walk(result.stdout)
    if not disks:
        raise DiskPollError("no physical disks returned (SNMP agent off, wrong community, or no RAID controller)")
    return disks


class DiskHealthPoller:
    def __init__(self, host, community, port=161, poll_minutes=10,
                 mqtt_client=None, log_fn=None, on_update_fn=None):
        self.host = host
        self.community = community
        self.port = port or 161
        self.poll_minutes = float(poll_minutes if poll_minutes is not None else 10)
        self.poll_interval = max(60.0, self.poll_minutes * 60.0)
        self.mqtt = mqtt_client
        self._log = log_fn or (lambda level, msg: None)
        self.on_update_fn = on_update_fn
        self.stop_event = threading.Event()

        self._thread = None
        self._started = False
        self._restarted = False
        self._start_time = time.time()

        self.last_success = None
        self.consecutive_failures = 0
        self.last_error = None
        self.had_problem = False
        self.is_stale = False
        self.warned_failure = False

        self.last_disks = {}
        self.last_summary = None
        self.discovered_disks = set()
        self.discovered_summary = False
        self._republish_needed = threading.Event()

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._started = True
        self._thread = threading.Thread(target=self.run, name="DiskHealthPoller", daemon=True)
        self._thread.start()

    def is_alive(self):
        return self._thread.is_alive() if self._thread is not None else False

    def join(self, timeout=None):
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def stop(self):
        self.stop_event.set()

    def _check_and_restart_poller_thread(self):
        if self._started and not self.is_alive() and not self._restarted and not self.stop_event.is_set():
            self._log("warning", "Disk health poller thread is not alive; restarting.")
            self._restarted = True
            self._thread = threading.Thread(target=self.run, name="DiskHealthPoller", daemon=True)
            self._thread.start()
            return True
        return False

    def run(self):
        # Wait shortly after startup / MQTT connection
        deadline = time.monotonic() + 10.0
        while not self.stop_event.is_set() and time.monotonic() < deadline:
            if self.mqtt and getattr(self.mqtt, "is_connected", False):
                break
            self.stop_event.wait(0.5)

        if not self.stop_event.is_set() and self.mqtt and getattr(self.mqtt, "is_connected", False):
            self.stop_event.wait(1.0)

        next_snmp_poll_time = 0.0
        republish_3s_time = None

        while not self.stop_event.is_set():
            now = time.monotonic()

            if self._republish_needed.is_set():
                self._republish_needed.clear()
                self._republish_all()
                republish_3s_time = time.monotonic() + 3.0

            if republish_3s_time is not None and now >= republish_3s_time:
                republish_3s_time = None
                self.republish_cached_states(reason="3s post-discovery")

            if now >= next_snmp_poll_time:
                try:
                    disks = walk_physical_disks(self.host, self.community, port=self.port)
                    first_discovery = not self.discovered_summary
                    self._handle_success(disks)
                    if first_discovery:
                        republish_3s_time = time.monotonic() + 3.0
                    next_snmp_poll_time = time.monotonic() + self.poll_interval
                except Exception as e:
                    self._handle_failure(e)
                    backoff_factor = 2 ** min(max(0, self.consecutive_failures - 1), 5)
                    next_snmp_poll_time = time.monotonic() + min(1800.0, 60.0 * backoff_factor)

            self.stop_event.wait(0.5)

    def _sleep(self, seconds):
        deadline = time.monotonic() + seconds
        while not self.stop_event.is_set() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            self.stop_event.wait(min(0.5, max(0.0, remaining)))

    def _handle_success(self, disks, now=None):
        current_time = time.time() if now is None else float(now)
        was_failing = (self.consecutive_failures > 0 or self.is_stale)
        self.consecutive_failures = 0
        self.last_error = None
        self.warned_failure = False
        self.is_stale = False
        self.last_success = current_time

        self.last_disks = disks
        summary = summarize(disks)
        self.last_summary = summary
        self.had_problem = summary["has_problem"]

        if was_failing:
            self._log("info", f"Disk health monitoring recovered ({len(disks)} disks found).")

        prob_str = f"{summary['problem_count']} problem(s)" if summary['has_problem'] else "all OK"
        self._log("debug", f"Disk health poll: {len(disks)} disks, summary: {prob_str}")

        if self.on_update_fn:
            try:
                self.on_update_fn(disks, summary)
            except Exception as e:
                self._log("error", f"Error in on_update_fn: {e}")

        if self.mqtt and getattr(self.mqtt, "is_connected", False):
            self._publish_disks_mqtt(disks, summary)

    def _handle_failure(self, err):
        self.consecutive_failures += 1
        self.last_error = str(err).splitlines()[0][:200]
        if not self.warned_failure:
            self._log("warning", f"Disk health poll failed: {err}")
            self.warned_failure = True

        if self.consecutive_failures >= 3:
            self.is_stale = True
            if self.mqtt and getattr(self.mqtt, "is_connected", False):
                self.check_freshness_and_republish(reason="3 failures")

    def _ensure_discovery(self, disks=None, summary=None):
        if not self.mqtt or not getattr(self.mqtt, "is_connected", False):
            return

        if not self.discovered_summary:
            summary_avail = f"{self.mqtt.base_topic}/sensor/disks_summary/availability"
            self.mqtt.publish_discovery(
                "sensor", "disks_summary", "Disks summary",
                icon="mdi:harddisk", extra_avail_topic=summary_avail
            )
            self.mqtt.publish_discovery(
                "binary_sensor", "disks_problem", "Disks problem",
                device_class="problem"
            )
            self.mqtt.publish_discovery(
                "binary_sensor", "disk_monitoring_failing", "Disk monitoring failing",
                device_class="problem", entity_category="diagnostic"
            )
            self.mqtt.publish_discovery(
                "sensor", "disks_last_checked", "Disks last checked",
                device_class="timestamp", entity_category="diagnostic"
            )
            self.discovered_summary = True

        disks = disks or self.last_disks
        current_indices = set(disks.keys()) if disks else set()
        if disks:
            for index, disk in disks.items():
                slug = f"disk_{index}_health"
                bay_match = re.search(r'Disk\.Bay\.(\d+)', disk.get('fqdn') or '')
                bay = int(bay_match.group(1)) if bay_match else index - 1
                name = f"Disk {bay} health"

                if index not in self.discovered_disks:
                    disk_avail = f"{self.mqtt.base_topic}/sensor/{slug}/availability"
                    self.mqtt.publish_discovery(
                        "sensor", slug, name,
                        icon="mdi:harddisk", extra_avail_topic=disk_avail
                    )
                    self.discovered_disks.add(index)

        # Cleanup removed disks
        removed_indices = self.discovered_disks - current_indices
        for removed_idx in list(removed_indices):
            slug = f"disk_{removed_idx}_health"
            if hasattr(self.mqtt, "remove_discovery"):
                self.mqtt.remove_discovery("sensor", slug)
            self.discovered_disks.discard(removed_idx)

    def _set_sensors_availability(self, online=True):
        if not self.mqtt or not getattr(self.mqtt, "is_connected", False):
            return
        payload = "online" if online else "offline"
        summary_avail_topic = f"{self.mqtt.base_topic}/sensor/disks_summary/availability"
        self.mqtt.publish(summary_avail_topic, payload, retain=True)

        for index in self.discovered_disks:
            disk_avail_topic = f"{self.mqtt.base_topic}/sensor/disk_{index}_health/availability"
            self.mqtt.publish(disk_avail_topic, payload, retain=True)

    def _publish_diagnostics(self):
        if not self.mqtt or not getattr(self.mqtt, "is_connected", False):
            return
        is_failing = self.is_stale or (self.consecutive_failures >= 3)
        iso_last_success = (
            datetime.datetime.fromtimestamp(self.last_success, datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
            if self.last_success is not None else None
        )
        failing_attrs = {
            "last_success": iso_last_success,
            "last_error": self.last_error,
            "consecutive_failures": self.consecutive_failures,
        }
        self.mqtt.publish_state(
            "binary_sensor", "disk_monitoring_failing",
            "ON" if is_failing else "OFF",
            attributes=failing_attrs,
            retain=True
        )
        if iso_last_success is not None:
            self.mqtt.publish_state("sensor", "disks_last_checked", iso_last_success, retain=True)
        else:
            self.mqtt.publish_state("sensor", "disks_last_checked", "unknown", retain=True)

    def _publish_states(self, disks, summary, stale=False):
        if not self.mqtt or not getattr(self.mqtt, "is_connected", False):
            return

        iso_last_success = (
            datetime.datetime.fromtimestamp(self.last_success, datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
            if self.last_success is not None else None
        )

        # 1. Summary sensor
        summary_state = (
            f"All {summary['count']} disks OK"
            if summary['problem_count'] == 0
            else f"{summary['problem_count']} problem(s)"
        )
        summary_attrs = {
            "count": summary["count"],
            "problem_count": summary["problem_count"],
            "problems": summary["problems"],
        }
        if stale:
            summary_attrs["stale"] = True
            summary_attrs["last_success"] = iso_last_success
        self.mqtt.publish_state("sensor", "disks_summary", summary_state, summary_attrs, retain=True)

        # 2. Binary sensor
        prob_attrs = {}
        if stale:
            prob_attrs["stale"] = True
            prob_attrs["last_success"] = iso_last_success
        self.mqtt.publish_state(
            "binary_sensor", "disks_problem",
            "ON" if summary["has_problem"] else "OFF",
            attributes=prob_attrs if prob_attrs else None,
            retain=True
        )

        # 3. Per disk sensor
        for index, disk in disks.items():
            slug = f"disk_{index}_health"
            attrs = {
                "model": disk.get("model"),
                "serial": disk.get("serial"),
                "firmware": disk.get("firmware"),
                "size_gb": disk.get("size_gb"),
                "media_type": disk.get("media_type"),
                "raid_state": disk.get("raid_state"),
                "component_status": disk.get("component_status"),
                "smart_alert": disk.get("smart_alert"),
                "manufactured": disk.get("manufactured"),
                "endurance_remaining": disk.get("endurance_remaining"),
                "slot": disk.get("slot"),
            }
            if stale:
                attrs["stale"] = True
                attrs["last_success"] = iso_last_success
            self.mqtt.publish_state("sensor", slug, disk.get("health_label", "Unknown"), attrs, retain=True)

    def check_freshness_and_republish(self, now=None, reason="periodic"):
        if not self.mqtt or not getattr(self.mqtt, "is_connected", False):
            return

        current_time = time.time() if now is None else float(now)

        # Restart thread once if dead
        self._check_and_restart_poller_thread()

        stale_limit = max(3.0 * self.poll_minutes * 60.0, 1800.0)
        ref_time = self.last_success if self.last_success is not None else self._start_time
        time_stale = (current_time - ref_time) > stale_limit
        fail_stale = self.consecutive_failures >= 3
        self.is_stale = time_stale or fail_stale

        self._ensure_discovery(self.last_disks, self.last_summary)

        if self.is_stale:
            if not self.had_problem:
                self._set_sensors_availability(online=False)
                # Do NOT keep republishing cached "OK"
            else:
                self._set_sensors_availability(online=True)
                if self.last_disks:
                    summary = self.last_summary or summarize(self.last_disks)
                    self._publish_states(self.last_disks, summary, stale=True)
        else:
            self._set_sensors_availability(online=True)
            if self.last_disks:
                summary = self.last_summary or summarize(self.last_disks)
                self._publish_states(self.last_disks, summary, stale=False)

        self._publish_diagnostics()
        if not (self.is_stale and not self.had_problem) and self.last_disks:
            total = len(self.last_disks) + 4
            self._log("debug", f"Republished {total} disk states to MQTT ({reason})")

    def republish_cached_states(self, reason="cached"):
        self.check_freshness_and_republish(reason=reason)

    def _publish_disks_mqtt(self, disks, summary):
        self._ensure_discovery(disks, summary)
        self._set_sensors_availability(online=True)
        self._publish_states(disks, summary, stale=False)
        self._publish_diagnostics()

    def republish_mqtt(self):
        self._republish_needed.set()

    def _republish_all(self):
        if not self.mqtt or not getattr(self.mqtt, "is_connected", False) or not self.last_disks:
            return
        self.discovered_disks.clear()
        self.discovered_summary = False
        summary = self.last_summary or summarize(self.last_disks)
        self._publish_disks_mqtt(self.last_disks, summary)

    def stop(self):
        self.stop_event.set()

