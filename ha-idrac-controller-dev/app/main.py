# HA-iDRAC/ha-idrac-controller-dev/app/main.py
import copy
import json
import logging
import os
import re
import signal
import tempfile
import threading
import time

from .fan_logic import SimpleFanState, clamp_speed, validate_fan_config
from .fan_health import FanMonitor
from .ipmi_manager import IPMIManager
from .mqtt_client import MqttClient
from .adaptive_pid import AdaptiveFanController
from .disk_health import DiskHealthPoller, walk_physical_disks
from . import web_server
from . import watchdog
from .webserver import WebServer

# --- Global Variables ---
running = True
threads = []
worker_instances = []
status_lock = threading.Lock()
pid_state_lock = threading.Lock()
heartbeat_lock = threading.Lock()


def update_watchdog_heartbeat(workers=None, now=None):
    """Write the current list of manually-controlled servers to the watchdog heartbeat."""
    active_workers = worker_instances if workers is None else workers
    with heartbeat_lock:
        servers = [
            {
                "id": getattr(w, "alias", str(id(w))),
                "host": w.ipmi.ip,
                "user": w.ipmi.user,
                "password": w.ipmi.password,
            }
            for w in active_workers
            if getattr(w, "in_manual_control", False)
        ]
        try:
            watchdog.write_heartbeat(servers, path=watchdog.HEARTBEAT_FILE, now=now)
        except OSError as e:
            print(f"[WARNING] Could not write watchdog heartbeat: {e}", flush=True)
ALL_SERVERS_STATUS = {}
STATUS_FILE = "/data/current_status.json"
PID_STATE_FILE = "/data/pid_states.json"


def save_pid_state(alias, pid_state, filepath=None):
    target = filepath or PID_STATE_FILE
    with pid_state_lock:
        all_states = {}
        if os.path.exists(target):
            try:
                with open(target, 'r') as f:
                    all_states = json.load(f)
            except (IOError, json.JSONDecodeError):
                all_states = {}
        all_states[alias] = pid_state
        temp_file = f"{target}.tmp.{threading.get_ident()}"
        with open(temp_file, 'w') as f:
            json.dump(all_states, f, indent=4)
        os.replace(temp_file, target)


def delete_pid_state(alias, filepath=None):
    target = filepath or PID_STATE_FILE
    with pid_state_lock:
        if not os.path.exists(target):
            return
        try:
            with open(target, 'r') as f:
                all_states = json.load(f)
        except (IOError, json.JSONDecodeError):
            return
        if alias in all_states:
            del all_states[alias]
            temp_file = f"{target}.tmp.{threading.get_ident()}"
            with open(temp_file, 'w') as f:
                json.dump(all_states, f, indent=4)
            os.replace(temp_file, target)


# --- Graceful Shutdown ---
def graceful_shutdown(signum, frame):
    global running
    print("[MAIN] Shutdown signal received. Cleaning up...", flush=True)
    running = False


signal.signal(signal.SIGTERM, graceful_shutdown)
signal.signal(signal.SIGINT, graceful_shutdown)


# --- Server Worker Class ---
class ServerWorker:
    def __init__(self, server_config, global_opts):
        self.config = server_config
        self.global_opts = global_opts
        self.alias = self.config['alias']
        self.log_level = self.global_opts['log_level']
        self.running = True

        self.ipmi = IPMIManager(
            ip=self.config['idrac_ip'],
            user=self.config['idrac_username'],
            password=self.config['idrac_password'],
            log_level=self.log_level,
        )
        self.mqtt = MqttClient(client_id=f"ha_idrac_{self.alias}")
        pid_config = self.config.get('pid_config', {})
        target_temp = pid_config.get('target_temp')
        if target_temp is None or target_temp == "":
            target_temp = self.config.get('target_temp', 50)
        self.min_safe_fan_speed = max(5, min(100, int(self.config.get('min_safe_fan_speed', 20))))
        min_speed = max(int(pid_config.get('min_speed', 15)), self.min_safe_fan_speed)
        max_speed = pid_config.get('max_speed', 95)
        deadband = pid_config.get('deadband', 0.75)
        ti_seconds = pid_config.get('ti_seconds', 300.0)
        ramp_down = pid_config.get('ramp_down_percent_per_min')
        if ramp_down is None or ramp_down == "":
            ramp_down = 4.0

        critical_temp = self.config.get('critical_temp_threshold')
        if critical_temp is not None and str(critical_temp).strip() != "":
            try:
                critical_temp = float(critical_temp)
            except (TypeError, ValueError):
                critical_temp = None
        else:
            critical_temp = None

        self.pid = AdaptiveFanController(
            setpoint=target_temp,
            min_speed=min_speed,
            max_speed=max_speed,
            deadband=deadband,
            ti_seconds=ti_seconds,
            slew_down_per_min=float(ramp_down),
            critical_temp=critical_temp,
        )
        self.fan_state = SimpleFanState()
        self._cycle_lock = threading.Lock()
        self.in_manual_control = False
        self._consecutive_read_failures = 0
        self._idrac_unreachable = False
        self._needs_reconnect_handback = False
        self._last_heartbeat_time = 0.0

        poll_int = self.config.get('poll_interval_seconds')
        if poll_int is not None and str(poll_int).strip() != "":
            try:
                self.poll_interval = max(5.0, float(poll_int))
            except (TypeError, ValueError):
                self.poll_interval = float(self.global_opts.get('check_interval_seconds', 60))
        else:
            self.poll_interval = float(self.global_opts.get('check_interval_seconds', 60))
        self._interval_overrun_warned = False

        self.server_info = {}
        self.discovered_sensors = set()
        self.discovered_components = {}
        self._next_mqtt_attempt = 0.0
        self._pcie_response_disabled = False
        self.alive = True
        self.fan_cfg = validate_fan_config(
            self.config.get('low_temp_threshold', 45),
            self.config.get('critical_temp_threshold', 65),
            self.config.get('base_fan_speed_percent', 20),
            self.config.get('high_temp_fan_speed_percent', 50),
        )
        self.disk_poller = None
        self._last_disk_republish_time = 0.0
        self.current_disks = []
        self.current_disks_summary = None

        self.fan_monitor = FanMonitor(poll_seconds=self.poll_interval)
        self._fan_fault_prev_ok = True
        self._fan_fault_last_fault_time = None
        self._fan_fault_dell_auto_latched = False
        self._fan_fault_status = {"faulty": {}, "ok": True}
        self._fan_fault_description = "OK"
        self._fan_readings = {}
        self._fan_republish_at = None
        self._last_mqtt_status = None

    def _log(self, level, message):
        print(f"[{level.upper()}] [{self.alias}] {message}", flush=True)

    def _on_disks_updated(self, disks, summary):
        formatted_disks = []
        if disks:
            for disk in disks.values():
                bay_match = re.search(r'Disk\.Bay\.(\d+)', disk.get('fqdn') or '')
                bay = int(bay_match.group(1)) if bay_match else disk['index'] - 1
                formatted_disks.append({
                    "bay": bay,
                    "model": disk.get("model"),
                    "size_gb": disk.get("size_gb"),
                    "media_type": disk.get("media_type"),
                    "health": disk.get("health"),
                    "health_label": disk.get("health_label"),
                })
        self.current_disks = formatted_disks
        self.current_disks_summary = summary
        with status_lock:
            if self.alias in ALL_SERVERS_STATUS:
                ALL_SERVERS_STATUS[self.alias]["disks"] = formatted_disks
                ALL_SERVERS_STATUS[self.alias]["disks_summary"] = summary

    def _on_mqtt_message(self, topic, payload):
        command_topic = f"{self.mqtt.base_topic}/command/shutdown"
        if topic == command_topic and payload == "PRESS":
            self._log("info", "Shutdown command received via MQTT.")
            self.ipmi.chassis_shutdown()

    def _on_mqtt_connected(self):
        """Called on every (re)connect: subscriptions and discovery are lost
        when the broker restarts, so restore them instead of staying silent
        until the add-on is restarted."""
        self._log("info", "MQTT connected; subscribing to command topic and re-publishing discovery.")
        self.mqtt.subscribe(f"{self.mqtt.base_topic}/command/shutdown")
        self.discovered_sensors.clear()
        if self.disk_poller:
            self.disk_poller.republish_mqtt()

    def _initialize(self):
        self._log("info", "Initializing server worker...")

        fan_mode = self.config.get('fan_mode', 'simple')
        if fan_mode == 'target' and self.poll_interval > 30:
            self._log("warning", f"Target temperature mode works best with poll_interval_seconds <= 10 "
                                 f"(current: {int(self.poll_interval)}s) so the controller receives enough samples.")

        with pid_state_lock:
            if os.path.exists(PID_STATE_FILE):
                with open(PID_STATE_FILE, 'r') as f:
                    try:
                        all_states = json.load(f)
                        if self.alias in all_states:
                            self.pid.load_state(all_states[self.alias])
                            self._log("info", f"Loaded persistent PID state: "
                                             f"theta={[round(t, 4) for t in self.pid.theta]}, "
                                             f"samples={self.pid.samples}")
                    except json.JSONDecodeError:
                        self._log("warning", "Could not decode PID state file.")

        model_data = self.ipmi.get_server_model_info()
        if model_data:
            self.server_info.update(model_data)
        firmware = self.ipmi.get_firmware_version()
        if firmware:
            self.server_info["firmware"] = firmware
            self._log("info", f"iDRAC firmware version: {firmware}")

        if fan_mode == 'target':
            target_str = int(self.pid.setpoint) if self.pid.setpoint.is_integer() else self.pid.setpoint
            min_str = int(self.pid.min_speed) if self.pid.min_speed.is_integer() else self.pid.min_speed
            max_str = int(self.pid.max_speed) if self.pid.max_speed.is_integer() else self.pid.max_speed
            self._log("info", f"PID target {target_str} C, min {min_str}%, max {max_str}%")
        else:
            for warning in self.fan_cfg["warnings"]:
                self._log("warning", warning)
            self._log("info", f"Fan profile: base {self.fan_cfg['base']}%, high {self.fan_cfg['high']}% at "
                               f"{self.fan_cfg['low']} C, Dell auto at {self.fan_cfg['critical']} C")

        if self.config.get("disable_pcie_cooling_response") and self.config.get('fan_control_enabled', True):
            if self.ipmi.set_third_party_pcie_cooling_response(enabled=False):
                self._pcie_response_disabled = True
            else:
                self._log("warning", "This server did not accept the third-party PCIe cooling-response "
                                     "command; leaving that setting unchanged.")

        self.mqtt.configure_broker(
            self.global_opts["mqtt_host"], self.global_opts["mqtt_port"],
            self.global_opts["mqtt_username"], self.global_opts["mqtt_password"],
            self.log_level,
        )
        self.mqtt.set_device_info(
            server_alias=self.alias,
            manufacturer=self.server_info.get("manufacturer"),
            model=self.server_info.get("model"),
            ip_address=self.config.get("idrac_ip"),
            serial_number=self.server_info.get("serial"),
            sw_version=self.server_info.get("firmware"),
        )
        self.mqtt.on_connect_callback = self._on_mqtt_connected

        # MQTT being down must not stop IPMI monitoring/control. Try once at
        # startup, then keep retrying in the background of the control loop.
        if self.global_opts.get("mqtt_host"):
            self.mqtt.connect()
            if not self.mqtt.wait_connected(timeout=3):
                self._log("warning", "MQTT broker not reachable yet; will keep retrying in the background.")
        else:
            self._log("warning", "No MQTT broker configured; Home Assistant entities will not be published.")

        if self.config.get("snmp_enabled", False):
            community = self.config.get("snmp_community", "")
            port = self.config.get("snmp_port", 161)
            poll_mins = self.config.get("disk_poll_minutes", 10)
            self.disk_poller = DiskHealthPoller(
                host=self.config.get("idrac_ip"),
                community=community,
                port=port,
                poll_minutes=poll_mins,
                mqtt_client=self.mqtt,
                log_fn=self._log,
                on_update_fn=self._on_disks_updated,
            )
            self.disk_poller.start()
        else:
            def _probe():
                try:
                    disks = walk_physical_disks(
                        self.config.get("idrac_ip"),
                        "public",
                        port=161,
                        timeout=2,
                        retries=0,
                    )
                    if disks:
                        self._log("info", "Disk health is available for this server; enable it in the server settings.")
                except Exception:
                    pass
            threading.Thread(target=_probe, daemon=True).start()

        return True

    def run(self):
        try:
            self._initialize()
            self._run_loop()
        finally:
            # Runs on SIGTERM as well as on an unexpected exception, so the
            # fans do not stay in manual mode after the add-on stops.
            self.cleanup()
            self.alive = False

    def _run_loop(self):
        next_run = time.monotonic()
        while self.running and running:
            cycle_start = time.monotonic()

            if (not self.mqtt.is_connected and self.global_opts.get("mqtt_host")
                    and time.time() >= self._next_mqtt_attempt):
                self.mqtt.connect()
                self._next_mqtt_attempt = time.time() + 15

            sleep_duration = self.step_once()
            if sleep_duration is not None:
                self._sleep(sleep_duration)
                next_run = time.monotonic()
                continue

            cycle_duration = time.monotonic() - cycle_start
            self._log("debug", f"Cycle completed in {cycle_duration:.2f}s")

            next_run += self.poll_interval
            now = time.monotonic()
            if now > next_run:
                if not self._interval_overrun_warned:
                    self._interval_overrun_warned = True
                    self._log(
                        "warning",
                        f"Poll interval ({self.poll_interval:.1f}s) is too short for this iDRAC "
                        f"(cycle took {cycle_duration:.2f}s); effective interval will be ~{cycle_duration:.2f}s."
                    )
                next_run = now
            else:
                self._sleep(next_run - now)

    def _emit_heartbeat(self):
        self._last_heartbeat_time = time.monotonic()
        update_watchdog_heartbeat()

    def _set_manual_control(self, manual):
        changed = (self.in_manual_control != manual)
        self.in_manual_control = manual
        if changed:
            self._emit_heartbeat()

    def _publish_idrac_reachable(self, reachable):
        if self.mqtt.is_connected:
            self.mqtt.publish_state(
                "binary_sensor",
                "idrac_reachable",
                "ON" if reachable else "OFF",
                retain=True,
            )

    def step_once(self):
        """Execute a single poll cycle. Returns the recommended sleep duration in seconds."""
        with self._cycle_lock:
            return self._step_once_locked()

    def _step_once_locked(self):
        raw_temp_data = self.ipmi.retrieve_temperatures_raw()
        if raw_temp_data is None:
            self._consecutive_read_failures += 1
            if self._consecutive_read_failures >= 3:
                if not self._idrac_unreachable:
                    self._log(
                        "warning",
                        "iDRAC unreachable after 3 consecutive failed temperature reads; "
                        "attempting hand-back to Dell auto every 60 s.",
                    )
                    self._idrac_unreachable = True
                    self._needs_reconnect_handback = True
                if self.ipmi.apply_dell_fan_control_profile() is not None:
                    self._set_manual_control(False)
                self._publish_idrac_reachable(False)
            else:
                self._log("warning", "Could not read temperature SDR data; skipping cycle.")

            if self.mqtt.is_connected:
                self.mqtt.publish(self.mqtt.availability_topic, "online", retain=True)
            self._emit_heartbeat()
            return 60

        self._consecutive_read_failures = 0
        if self._needs_reconnect_handback:
            self._log("info", "iDRAC reconnected; handing fans back to Dell auto and resetting slew state.")
            self.ipmi.apply_dell_fan_control_profile(force=True)
            self._set_manual_control(False)
            self.pid.speed = float(self.pid.min_speed)
            self.pid.last_time = None
            self.fan_state.in_auto = False
            self.fan_state.bucket = None
            self._needs_reconnect_handback = False
        self._idrac_unreachable = False

        if self.mqtt.is_connected:
            self.mqtt.publish(self.mqtt.availability_topic, "online", retain=True)
            self._publish_idrac_reachable(True)

        temps = self.ipmi.parse_temperatures(raw_temp_data)
        raw_fan_data = self.ipmi.retrieve_fan_rpms_raw()
        fans = self.ipmi.parse_fan_rpms(raw_fan_data)
        fan_readings = self.ipmi.parse_fan_readings(raw_fan_data)
        self.evaluate_fan_health(fan_readings)
        power_sdr_data = self.ipmi.retrieve_power_sdr_raw()
        power = self.ipmi.parse_power_consumption(power_sdr_data)
        psu_statuses = self.ipmi.get_power_status(power_sdr_data)

        hottest_cpu = max(temps['cpu_temps']) if temps['cpu_temps'] else None
        target_fan_speed = self._apply_fan_policy(hottest_cpu, temps, power)
        self._emit_heartbeat()

        fan_mode = self.config.get('fan_mode', 'simple')
        pid_status = (
            f"Learning ({self.pid.samples} samples)"
            if self.pid.samples < 5
            else "Adapted"
        ) if fan_mode == 'target' else None

        status_data = {
            "hottest_cpu_temp": hottest_cpu,
            "inlet_temp": temps.get('inlet_temp'),
            "exhaust_temp": temps.get('exhaust_temp'),
            "power": power,
            "target_fan_speed": None if isinstance(target_fan_speed, str) else target_fan_speed,
            "cpus": temps.get('cpu_temps', []),
            "fans": fans,
            "psus": psu_statuses,
            "pid_status": pid_status,
            "fan_ok": self._fan_fault_status.get("ok", True),
            "fan_status": self._fan_fault_description,
            "fan_readings": fan_readings,
        }
        with status_lock:
            ALL_SERVERS_STATUS[self.alias] = {
                "alias": self.alias,
                "ip": self.config['idrac_ip'],
                "last_updated": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
                "hottest_cpu_temp_c": hottest_cpu,
                "inlet_temp_c": temps.get('inlet_temp'),
                "exhaust_temp_c": temps.get('exhaust_temp'),
                "power_consumption_watts": power,
                "target_fan_speed_percent": target_fan_speed,
                "cpu_temps_c": temps.get('cpu_temps', []),
                "actual_fan_rpms": fans,
                "psu_statuses": psu_statuses,
                "pid_status": pid_status,
                "disks": getattr(self, 'current_disks', []),
                "disks_summary": getattr(self, 'current_disks_summary', None),
                "fan_fault": not self._fan_fault_status.get("ok", True),
                "fan_status": self._fan_fault_description,
            }

        self._publish_mqtt_data(status_data)
        return None

    def evaluate_fan_health(self, readings, now=None):
        """Update FanMonitor with {fan_name: rpm_or_None} and track fault state."""
        now = time.monotonic() if now is None else now
        if self.fan_monitor.poll_seconds != self.poll_interval:
            self.fan_monitor.set_poll_seconds(self.poll_interval)
        self._fan_readings = dict(readings)
        result = self.fan_monitor.update(now, readings)
        self._fan_fault_status = result
        self._fan_fault_description = self.fan_monitor.describe()

        if not result["ok"]:
            self._fan_fault_last_fault_time = now
            if self._fan_fault_prev_ok:
                self._log("warning", f"Fan fault detected: {self._fan_fault_description}")
            self._fan_fault_prev_ok = False
        else:
            if not self._fan_fault_prev_ok:
                self._log("info", "Fan fault cleared; all fans healthy.")
            self._fan_fault_prev_ok = True
        return result

    def _apply_fan_policy(self, hottest_cpu, temps, power, now=None):
        """Compute and apply fan speed while enforcing the server's on_fan_fault policy."""
        now = time.monotonic() if now is None else now
        if not self.config.get('fan_control_enabled', True):
            self._log("info", "Fan control is disabled for this server; leaving the iDRAC fan settings untouched.")
            return "Monitor only"
        if self.ipmi.fan_control_refused:
            return "Dell Auto (refused)"

        policy = self.config.get("on_fan_fault", "dell_auto")
        fan_ok = self._fan_fault_status.get("ok", True)

        if policy == "dell_auto":
            if not fan_ok:
                if not self._fan_fault_dell_auto_latched:
                    self._fan_fault_dell_auto_latched = True
                    self.ipmi.apply_dell_fan_control_profile(force=True)
                    self._set_manual_control(False)
                return "Dell Auto (fan fault)"
            if self._fan_fault_dell_auto_latched:
                if (self._fan_fault_last_fault_time is not None
                        and (now - self._fan_fault_last_fault_time) >= 600.0):
                    self._fan_fault_dell_auto_latched = False
                else:
                    return "Dell Auto (fan fault cooldown)"

        if hottest_cpu is None:
            self._log("warning", "No CPU temperature sensor found; handing fans back to the iDRAC for safety.")
            self.ipmi.apply_dell_fan_control_profile()
            self._set_manual_control(False)
            return "Dell Auto (no CPU sensor)"

        crit_thresh = self.fan_cfg["critical"]
        if self.fan_state.should_use_auto(hottest_cpu, crit_thresh):
            self.ipmi.apply_dell_fan_control_profile()
            self._set_manual_control(False)
            return "Dell Auto"

        fan_mode = self.config.get('fan_mode', 'simple')
        target_fan_speed = "Dell Auto"

        if fan_mode == 'simple':
            low_thresh = self.fan_cfg["low"]
            target_fan_speed = self.fan_state.simple_speed(
                hottest_cpu,
                low_thresh,
                self.fan_cfg["base"],
                self.fan_cfg["high"],
            )
        elif fan_mode == 'target':
            inlet = temps.get('inlet_temp')
            speed = self.pid.update(hottest_cpu, inlet, power)
            feed_speed = self.pid.last_feedforward if self.pid.last_feedforward is not None else 0.0
            self._log(
                "debug",
                f"PID update: temp={hottest_cpu}°C, inlet={inlet}°C, power={power}W, "
                f"feedforward={feed_speed:.1f}%, trim={self.pid.trim:.2f}%, "
                f"theta={[round(t, 4) for t in self.pid.theta]}, samples={self.pid.samples}, "
                f"overcooled={self.pid.overcooled}, fan_power={self.pid.fan_power_k:.1f}W",
            )
            if speed is not None:
                target_fan_speed = clamp_speed(
                    speed,
                    minimum=int(self.pid.min_speed),
                    maximum=int(self.pid.max_speed),
                )
            else:
                target_fan_speed = getattr(self, '_last_target_fan_speed', "Target (in deadband)")
        elif fan_mode == 'curve':
            fan_curve = self.config.get('fan_curve', [])
            if len(fan_curve) >= 2:
                lower, upper = fan_curve[0], fan_curve[-1]
                for i in range(len(fan_curve) - 1):
                    if fan_curve[i]['temp'] <= hottest_cpu < fan_curve[i + 1]['temp']:
                        lower, upper = fan_curve[i], fan_curve[i + 1]
                        break

                if hottest_cpu < lower['temp']:
                    speed = lower['speed']
                elif hottest_cpu >= upper['temp']:
                    speed = upper['speed']
                else:
                    temp_range = upper['temp'] - lower['temp']
                    speed_range = upper['speed'] - lower['speed']
                    speed = (lower['speed'] + ((hottest_cpu - lower['temp']) / temp_range * speed_range)
                             if temp_range > 0 else lower['speed'])

                target_fan_speed = clamp_speed(speed)

        if policy == "raise_fans" and not fan_ok:
            if isinstance(target_fan_speed, (int, float)):
                target_fan_speed = max(int(target_fan_speed), 80)
            else:
                target_fan_speed = 80

        if isinstance(target_fan_speed, (int, float)):
            target_fan_speed = max(int(target_fan_speed), self.min_safe_fan_speed)
            if self.ipmi.apply_user_fan_control_profile(target_fan_speed) is None:
                self._set_manual_control(False)
                return "Dell Auto (fan command failed)"
            self._set_manual_control(True)
            if fan_mode == 'target':
                self._last_target_fan_speed = target_fan_speed
        return target_fan_speed

    def _check_disk_health_freshness(self):
        if self.disk_poller and self.mqtt.is_connected:
            now = time.monotonic()
            if now - self._last_disk_republish_time >= 60.0:
                self._last_disk_republish_time = now
                try:
                    self.disk_poller.check_freshness_and_republish()
                except Exception as e:
                    self._log("error", f"Error checking disk freshness: {e}")

    def _check_fan_republish(self):
        if (self._fan_republish_at is not None and time.monotonic() >= self._fan_republish_at
                and self.mqtt.is_connected and self._last_mqtt_status is not None):
            self._fan_republish_at = None
            self._publish_fan_health_mqtt(self._last_mqtt_status)

    def _sleep(self, seconds):
        """Sleep in small steps so SIGTERM can run cleanup promptly."""
        deadline = time.time() + seconds
        while time.time() < deadline and self.running and running:
            self._check_disk_health_freshness()
            self._check_fan_republish()
            if time.monotonic() - self._last_heartbeat_time >= 60.0:
                self._emit_heartbeat()
            time.sleep(0.2)

    def _publish_fan_health_mqtt(self, status):
        if not self.mqtt.is_connected:
            return
        fan_state = "ON" if not status.get("fan_ok", True) else "OFF"
        self.mqtt.publish_state("binary_sensor", "fan_fault", fan_state, retain=True)
        attrs = {
            (f"{k}_rpm" if k.lower() == "state" else k): v
            for k, v in status.get("fan_readings", {}).items()
        }
        self.mqtt.publish_state("sensor", "fan_status", status.get("fan_status", "OK"), attrs, retain=True)

    def _publish_mqtt_data(self, status):
        if not self.mqtt.is_connected:
            return

        self._last_mqtt_status = status

        sensors = {
            "shutdown_button": {"component": "button", "name": "Shutdown Server", "device_class": "restart", "icon": "mdi:server-off"},
            "hottest_cpu_temp": {"component": "sensor", "device_class": "temperature", "unit": "°C", "state_class": "measurement"},
            "inlet_temp": {"component": "sensor", "device_class": "temperature", "unit": "°C", "state_class": "measurement"},
            "exhaust_temp": {"component": "sensor", "device_class": "temperature", "unit": "°C", "state_class": "measurement"},
            "power": {"component": "sensor", "device_class": "power", "unit": "W", "state_class": "measurement", "icon": "mdi:flash"},
            "target_fan_speed": {"component": "sensor", "unit": "%", "icon": "mdi:fan-chevron-up"},
            "fan_fault": {"component": "binary_sensor", "name": "Fan fault", "device_class": "problem"},
            "fan_status": {"component": "sensor", "name": "Fan status", "icon": "mdi:fan-alert"},
            "idrac_reachable": {"component": "binary_sensor", "name": "iDRAC reachable", "device_class": "connectivity"},
        }
        for i, _ in enumerate(status.get('cpus', [])):
            sensors[f"cpu_{i}_temp"] = {"component": "sensor", "name": f"CPU {i} Temperature", "device_class": "temperature", "unit": "°C", "state_class": "measurement"}
        for fan in status.get('fans', []):
            sensors[f"fan_{re.sub(r'[^a-zA-Z0-9_]+', '', fan['name']).lower()}_rpm"] = {"component": "sensor", "name": f"{fan['name']} RPM", "unit": "RPM", "icon": "mdi:fan"}
        for psu in status.get('psus', []):
            sensors[f"psu_{re.sub(r'[^a-zA-Z0-9_]+', '', psu['name']).lower()}"] = {"component": "binary_sensor", "name": psu['name'], "device_class": "problem"}

        newly_discovered_fan = False
        for slug, desc in sensors.items():
            if slug not in self.discovered_sensors:
                cmd_topic = f"{self.mqtt.base_topic}/command/shutdown" if desc['component'] == 'button' else None
                self.mqtt.publish_discovery(
                    desc['component'], slug, desc.get('name', slug.replace("_", " ").title()),
                    desc.get('device_class'), desc.get('unit'), desc.get('icon'),
                    cmd_topic, None, desc.get('state_class'),
                )
                self.discovered_sensors.add(slug)
                self.discovered_components[slug] = desc['component']
                if slug in ("fan_fault", "fan_status"):
                    newly_discovered_fan = True

            if slug in ("fan_fault", "fan_status"):
                continue
            elif slug == "idrac_reachable":
                self.mqtt.publish_state('binary_sensor', 'idrac_reachable', 'ON', retain=True)
            elif desc['component'] == 'sensor':
                value = None
                if slug.startswith('fan_'):
                    value = next((f['rpm'] for f in status['fans']
                                  if f"fan_{re.sub(r'[^a-zA-Z0-9_]+', '', f['name']).lower()}_rpm" == slug), None)
                elif slug.startswith('cpu_'):
                    index = int(slug.split('_')[1])
                    value = status['cpus'][index] if index < len(status['cpus']) else None
                else:
                    value = status.get(slug)
                self.mqtt.publish_state('sensor', slug, value)
            elif desc['component'] == 'binary_sensor' and slug.startswith('psu_'):
                psu_name = desc['name']
                psu_data = next((p for p in status['psus'] if p['name'] == psu_name), None)
                state = "ON" if psu_data and not psu_data['ok'] else "OFF"
                self.mqtt.publish_state('binary_sensor', slug, state)

        self._publish_fan_health_mqtt(status)
        if newly_discovered_fan:
            self._fan_republish_at = time.monotonic() + 3.0

    def cleanup(self):
        if self.disk_poller:
            self.disk_poller.stop()
            self.disk_poller.join(timeout=3.0)

        if self.config.get('fan_control_enabled', True):
            self._log("info", "Worker shutting down. Reverting to Dell auto fans.")
            self.ipmi.apply_dell_fan_control_profile()
            self._set_manual_control(False)
        else:
            self._log("info", "Worker shutting down with fan control disabled; leaving iDRAC fan settings untouched.")

        if self._pcie_response_disabled:
            self._log("info", "Restoring Dell default third-party PCIe cooling response.")
            if not self.ipmi.set_third_party_pcie_cooling_response(enabled=True):
                self._log("warning", "Could not restore the third-party PCIe cooling response; it is still disabled.")

        if self.mqtt.is_connected:
            self.mqtt.disconnect()

        save_pid_state(self.alias, self.pid.get_state())
        self._log("info", f"Saved persistent PID state: theta={[round(t, 4) for t in self.pid.theta]}, samples={self.pid.samples}")

    def stop(self):
        self.running = False
        if self.disk_poller:
            self.disk_poller.stop()


class ServerManager:
    """Manages server configs, worker threads, and live settings updates."""

    def __init__(self, global_opts, config_file="/data/servers_config.json",
                 config_lock=None, workers=None, threads_list=None):
        self.global_opts = global_opts
        self.config_file = config_file
        self._config_lock = config_lock or threading.Lock()
        self._manager_lock = threading.Lock()
        self.workers = workers if workers is not None else []
        self.threads = threads_list if threads_list is not None else []
        self._worker_threads = {}

    def _load_configs(self):
        with self._config_lock:
            if not os.path.exists(self.config_file):
                return []
            with open(self.config_file, "r") as f:
                try:
                    return json.load(f)
                except json.JSONDecodeError:
                    return []

    def _save_configs(self, servers):
        with self._config_lock:
            parent = os.path.dirname(self.config_file) or "."
            os.makedirs(parent, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(dir=parent, prefix=".servers_config_", suffix=".tmp")
            try:
                with os.fdopen(fd, "w") as f:
                    json.dump(servers, f, indent=4)
                os.replace(tmp_path, self.config_file)
            except Exception:
                if os.path.exists(tmp_path):
                    os.unlink(tmp_path)
                raise

    @staticmethod
    def validate_settings(settings):
        alias = (settings.get("alias") or "").strip()
        if not alias:
            raise ValueError("Server alias is required.")
        ip = (settings.get("idrac_ip") or "").strip()
        if not ip:
            raise ValueError("iDRAC IP / Hostname is required.")

        safe_speed = settings.get("min_safe_fan_speed", 20)
        try:
            safe_val = int(safe_speed)
        except (TypeError, ValueError):
            raise ValueError("Minimum safe fan speed must be a whole number.")
        if safe_val < 5 or safe_val > 100:
            raise ValueError("Minimum safe fan speed must be between 5% and 100%.")

        poll_int = settings.get("poll_interval_seconds")
        if poll_int is not None and str(poll_int).strip() != "":
            try:
                if float(poll_int) < 5:
                    raise ValueError("Poll interval must be at least 5 seconds.")
            except (TypeError, ValueError) as exc:
                if "at least 5" in str(exc):
                    raise
                raise ValueError("Poll interval must be a number.")

        fan_mode = settings.get("fan_mode", "simple")
        if fan_mode == "simple":
            base = int(settings.get("base_fan_speed_percent", 20))
            high = int(settings.get("high_temp_fan_speed_percent", 50))
            low = int(settings.get("low_temp_threshold", 45))
            crit = int(settings.get("critical_temp_threshold", 65))
            if not (5 <= base <= 100) or not (5 <= high <= 100):
                raise ValueError("Fan speeds must be between 5% and 100%.")
            if base > high:
                raise ValueError("Base fan speed cannot exceed high-temp fan speed.")
            if low >= crit:
                raise ValueError("Low temperature threshold must be below critical temperature threshold.")
        elif fan_mode == "target":
            pid = settings.get("pid_config") or {}
            min_s = pid.get("min_speed", 15)
            max_s = pid.get("max_speed", 95)
            target = pid.get("target_temp", 50)
            ramp_down = pid.get("ramp_down_percent_per_min", 4)
            if min_s is not None and min_s != "" and max_s is not None and max_s != "":
                if int(min_s) > int(max_s):
                    raise ValueError("PID minimum fan speed cannot exceed maximum fan speed.")
                if not (5 <= int(min_s) <= 100) or not (5 <= int(max_s) <= 100):
                    raise ValueError("PID fan speeds must be between 5% and 100%.")
            if target is not None and target != "" and not (20 <= int(target) <= 90):
                raise ValueError("Target temperature must be between 20°C and 90°C.")
            if ramp_down is not None and ramp_down != "" and not (1 <= float(ramp_down) <= 50):
                raise ValueError("Ramp-down rate must be between 1 and 50 %/min.")
        elif fan_mode == "curve":
            for pt in settings.get("fan_curve", []):
                spd = int(pt.get("speed", 0))
                if not (5 <= spd <= 100):
                    raise ValueError("Fan curve speeds must be between 5% and 100%.")

    def get_worker(self, server_id):
        return next((w for w in self.workers if w.alias == server_id), None)

    def start_all(self, configs):
        with self._manager_lock:
            for conf in configs:
                if conf.get("enabled", False):
                    self._start_worker_locked(copy.deepcopy(conf))
            update_watchdog_heartbeat(self.workers)

    def _start_worker_locked(self, server_conf, pid_state=None):
        worker = ServerWorker(server_conf, self.global_opts)
        if pid_state:
            worker.pid.load_state(pid_state)
        self.workers.append(worker)
        thread = threading.Thread(target=worker.run, daemon=True)
        self.threads.append(thread)
        self._worker_threads[worker.alias] = thread
        thread.start()
        return worker

    def _stop_worker_locked(self, server_id, remove_discovery=False, timeout=10.0):
        worker = self.get_worker(server_id)
        if not worker:
            return None
        pid_state = worker.pid.get_state()
        thread = self._worker_threads.pop(server_id, None)
        worker.stop()
        if thread and thread.is_alive():
            thread.join(timeout=timeout)
        # Ensure Dell auto hand-back even if worker was never started in a thread
        if worker.config.get("fan_control_enabled", True):
            worker.ipmi.apply_dell_fan_control_profile()
        worker._set_manual_control(False)
        if remove_discovery and worker.mqtt.is_connected:
            for slug in list(worker.discovered_sensors):
                comp = worker.discovered_components.get(slug, "sensor")
                worker.mqtt.remove_discovery(comp, slug)
            worker.mqtt.disconnect()
        if worker in self.workers:
            self.workers.remove(worker)
        if thread in self.threads:
            self.threads.remove(thread)
        update_watchdog_heartbeat(self.workers)
        return pid_state

    def add_server(self, settings):
        self.validate_settings(settings)
        with self._manager_lock:
            configs = self._load_configs()
            if any(s["alias"] == settings["alias"] for s in configs):
                raise ValueError(f"Server alias '{settings['alias']}' already exists.")
            saved = copy.deepcopy(settings)
            configs.append(saved)
            self._save_configs(configs)
            if saved.get("enabled", True):
                self._start_worker_locked(copy.deepcopy(saved))
                update_watchdog_heartbeat(self.workers)

    def remove_server(self, server_id):
        with self._manager_lock:
            self._stop_worker_locked(server_id, remove_discovery=True)
            configs = self._load_configs()
            configs = [s for s in configs if s["alias"] != server_id]
            self._save_configs(configs)
            delete_pid_state(server_id)
            with status_lock:
                ALL_SERVERS_STATUS.pop(server_id, None)

    def update_connection(self, server_id, host=None, user=None, password=None):
        with self._manager_lock:
            configs = self._load_configs()
            target = next((s for s in configs if s["alias"] == server_id), None)
            if not target:
                raise ValueError(f"Server '{server_id}' not found.")
            updated = copy.deepcopy(target)
            if host is not None:
                updated["idrac_ip"] = host.strip()
            if user is not None:
                updated["idrac_username"] = user
            if password is not None and password != "":
                updated["idrac_password"] = password
            self.validate_settings(updated)
            idx = configs.index(target)
            configs[idx] = updated

            worker = self.get_worker(server_id)
            pid_state = None
            if worker:
                pid_state = self._stop_worker_locked(server_id, remove_discovery=False)
            self._save_configs(configs)
            if updated.get("enabled", True) and (worker is not None):
                self._start_worker_locked(copy.deepcopy(updated), pid_state=pid_state)
                update_watchdog_heartbeat(self.workers)

    def apply_settings(self, server_id, new_settings):
        self.validate_settings(new_settings)
        with self._manager_lock:
            configs = self._load_configs()
            idx = next((i for i, s in enumerate(configs) if s["alias"] == server_id), None)
            if idx is None:
                raise ValueError(f"Server '{server_id}' not found.")
            old_settings = configs[idx]
            conn_changed = (
                old_settings.get("idrac_ip") != new_settings.get("idrac_ip")
                or old_settings.get("idrac_username") != new_settings.get("idrac_username")
                or old_settings.get("idrac_password") != new_settings.get("idrac_password")
            )
            saved = copy.deepcopy(new_settings)
            configs[idx] = saved
            self._save_configs(configs)

            worker = self.get_worker(server_id)
            enabled = saved.get("enabled", True)

            if not enabled:
                if worker:
                    self._stop_worker_locked(server_id, remove_discovery=False)
                return

            if worker is None:
                self._start_worker_locked(copy.deepcopy(saved))
                update_watchdog_heartbeat(self.workers)
                return

            if conn_changed:
                pid_state = self._stop_worker_locked(server_id, remove_discovery=False)
                self._start_worker_locked(copy.deepcopy(saved), pid_state=pid_state)
                update_watchdog_heartbeat(self.workers)
                return

            with worker._cycle_lock:
                worker.config = copy.deepcopy(saved)
                worker.min_safe_fan_speed = max(5, min(100, int(saved.get("min_safe_fan_speed", 20))))

                pid_cfg = saved.get("pid_config", {})
                target_temp = pid_cfg.get("target_temp")
                if target_temp is None or target_temp == "":
                    target_temp = saved.get("target_temp", 50)
                min_speed = max(int(pid_cfg.get("min_speed", 15)), worker.min_safe_fan_speed)
                max_speed = int(pid_cfg.get("max_speed", 95))
                if max_speed < min_speed:
                    max_speed = min_speed
                deadband = pid_cfg.get("deadband", 0.75)
                ti_seconds = pid_cfg.get("ti_seconds", 300.0)
                ramp_down = pid_cfg.get("ramp_down_percent_per_min")
                if ramp_down is None or ramp_down == "":
                    ramp_down = 4.0
                crit_raw = saved.get("critical_temp_threshold")
                critical_temp = float(crit_raw) if crit_raw is not None and str(crit_raw).strip() != "" else None

                worker.pid.set_gains(
                    setpoint=target_temp,
                    min_speed=min_speed,
                    max_speed=max_speed,
                    deadband=deadband,
                    ti_seconds=ti_seconds,
                    ramp_down_per_min=float(ramp_down),
                    critical_temp=critical_temp,
                )

                worker.fan_cfg = validate_fan_config(
                    saved.get("low_temp_threshold", 45),
                    saved.get("critical_temp_threshold", 65),
                    saved.get("base_fan_speed_percent", 20),
                    saved.get("high_temp_fan_speed_percent", 50),
                )

                poll_int = saved.get("poll_interval_seconds")
                if poll_int is not None and str(poll_int).strip() != "":
                    worker.poll_interval = max(5.0, float(poll_int))
                else:
                    worker.poll_interval = float(self.global_opts.get("check_interval_seconds", 60))
                worker._interval_overrun_warned = False
                worker.fan_monitor.set_poll_seconds(worker.poll_interval)

                if not saved.get("fan_control_enabled", True) and worker.in_manual_control:
                    worker.ipmi.apply_dell_fan_control_profile()
                    worker._set_manual_control(False)

                want_disable_pcie = bool(saved.get("disable_pcie_cooling_response", False))
                if want_disable_pcie and not worker._pcie_response_disabled:
                    if worker.ipmi.set_third_party_pcie_cooling_response(enabled=False):
                        worker._pcie_response_disabled = True
                elif not want_disable_pcie and worker._pcie_response_disabled:
                    if worker.ipmi.set_third_party_pcie_cooling_response(enabled=True):
                        worker._pcie_response_disabled = False

                snmp_changed = (
                    old_settings.get("snmp_enabled") != saved.get("snmp_enabled")
                    or old_settings.get("snmp_community") != saved.get("snmp_community")
                    or old_settings.get("snmp_port") != saved.get("snmp_port")
                    or old_settings.get("disk_poll_minutes") != saved.get("disk_poll_minutes")
                )
                if snmp_changed:
                    if worker.disk_poller:
                        worker.disk_poller.stop()
                        worker.disk_poller.join(timeout=3.0)
                        worker.disk_poller = None
                    if saved.get("snmp_enabled", False) and saved.get("snmp_community"):
                        worker.disk_poller = DiskHealthPoller(
                            server_alias=worker.alias,
                            ip=saved["idrac_ip"],
                            community=saved["snmp_community"],
                            port=int(saved.get("snmp_port", 161)),
                            poll_minutes=int(saved.get("disk_poll_minutes", 10)),
                            mqtt_client=worker.mqtt,
                            log_fn=worker._log,
                            on_update_fn=worker._on_disks_updated,
                        )
                        worker.disk_poller.start()

            update_watchdog_heartbeat(self.workers)


# --- Main Execution ---
if __name__ == "__main__":
    print("[MAIN] ===== HA iDRAC Multi-Server Controller Starting =====", flush=True)

    global_options = {
        "log_level": os.getenv("LOG_LEVEL", "info"),
        "check_interval_seconds": int(os.getenv("CHECK_INTERVAL_SECONDS", 60)),
        "mqtt_host": os.getenv("MQTT_HOST", "core-mosquitto"),
        "mqtt_port": int(os.getenv("MQTT_PORT", 1883)),
        "mqtt_username": os.getenv("MQTT_USERNAME", ""),
        "mqtt_password": os.getenv("MQTT_PASSWORD", ""),
        "base_fan_speed_percent": int(os.getenv("BASE_FAN_SPEED_PERCENT", 20)),
        "low_temp_threshold": int(os.getenv("LOW_TEMP_THRESHOLD", 45)),
        "high_temp_fan_speed_percent": int(os.getenv("HIGH_TEMP_FAN_SPEED_PERCENT", 50)),
        "critical_temp_threshold": int(os.getenv("CRITICAL_TEMP_THRESHOLD", 65)),
    }

    is_debug = str(global_options.get("log_level", "info")).lower() == "debug"
    logging.basicConfig(
        level=logging.DEBUG if is_debug else logging.INFO,
        format="[%(levelname)s] %(message)s",
    )
    web_log_level = logging.DEBUG if is_debug else logging.WARNING
    logging.getLogger("waitress").setLevel(web_log_level)
    logging.getLogger("werkzeug").setLevel(web_log_level)

    servers_configs_list = []
    if not os.path.exists("/data/servers_config.json"):
        with open("/data/servers_config.json", 'w') as f:
            json.dump([], f)
    else:
        with open("/data/servers_config.json", 'r') as f:
            try:
                servers_configs_list = json.load(f)
            except json.JSONDecodeError:
                pass

    web_server.global_config = global_options
    web_server.STATUS_FILE = STATUS_FILE
    web_server.status_lock = status_lock
    web_server_port = int(os.getenv("INGRESS_PORT", 8099))
    web = WebServer(web_server.app, host="0.0.0.0", port=web_server_port)
    web.start()

    worker_instances.clear()
    server_manager = ServerManager(
        global_options,
        config_file="/data/servers_config.json",
        config_lock=web_server.config_lock,
        workers=worker_instances,
        threads_list=threads,
    )
    web_server.server_manager = server_manager
    server_manager.start_all(servers_configs_list)

    # Supervisor watchdog endpoint: 503 only if every configured worker has
    # died, not when the iDRAC/MQTT is merely unreachable (the worker stays
    # alive and keeps retrying in that case).
    web_server.set_health_check(lambda: not worker_instances or any(w.alive for w in worker_instances))

    try:
        while running:
            with status_lock:
                temp_status_file = f"{STATUS_FILE}.tmp.{os.getpid()}"
                with open(temp_status_file, 'w') as f:
                    json.dump(list(ALL_SERVERS_STATUS.values()), f, indent=4)
                os.replace(temp_status_file, STATUS_FILE)
            deadline = time.time() + 2
            while time.time() < deadline and running:
                time.sleep(0.2)
    except KeyboardInterrupt:
        graceful_shutdown(None, None)

    print("[MAIN] Waiting for all server threads to terminate...", flush=True)
    for worker in worker_instances:
        worker.stop()
    deadline = time.time() + 50.0
    for worker, thread in zip(worker_instances, threads):
        remaining = max(0.0, deadline - time.time())
        thread.join(timeout=remaining)
        if thread.is_alive():
            print(f"[WARNING] [{worker.alias}] Worker thread did not terminate within shutdown deadline.", flush=True)
    web.stop()
    print("[MAIN] ===== HA iDRAC Controller Stopped =====", flush=True)
