# HA-iDRAC/ha-idrac-controller-dev/app/main.py
import json
import os
import re
import signal
import threading
import time

from .fan_logic import SimpleFanState, clamp_speed, validate_fan_config
from .ipmi_manager import IPMIManager
from .mqtt_client import MqttClient
from .adaptive_pid import AdaptiveFanController
from .disk_health import DiskHealthPoller, walk_physical_disks
from . import web_server

# --- Global Variables ---
running = True
threads = []
status_lock = threading.Lock()
pid_state_lock = threading.Lock()
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
        min_speed = pid_config.get('min_speed', 15)
        max_speed = pid_config.get('max_speed', 95)
        deadband = pid_config.get('deadband', 0.75)
        ti_seconds = pid_config.get('ti_seconds', 300.0)
        ramp_down = pid_config.get('ramp_down_percent_per_min')
        if ramp_down is None or ramp_down == "":
            ramp_down = 4.0

        self.pid = AdaptiveFanController(
            setpoint=target_temp,
            min_speed=min_speed,
            max_speed=max_speed,
            deadband=deadband,
            ti_seconds=ti_seconds,
            slew_down_per_min=float(ramp_down),
        )
        self.fan_state = SimpleFanState()

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
        self.current_disks = []
        self.current_disks_summary = None

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

            raw_temp_data = self.ipmi.retrieve_temperatures_raw()
            if raw_temp_data is None:
                if self.mqtt.is_connected:
                    self.mqtt.publish(self.mqtt.availability_topic, "offline", retain=True)
                self._log("warning", "Could not read temperature SDR data; skipping cycle.")
                self._sleep(60)
                next_run = time.monotonic()
                continue

            if self.mqtt.is_connected:
                self.mqtt.publish(self.mqtt.availability_topic, "online", retain=True)

            temps = self.ipmi.parse_temperatures(raw_temp_data)
            fans = self.ipmi.parse_fan_rpms(self.ipmi.retrieve_fan_rpms_raw())
            power_sdr_data = self.ipmi.retrieve_power_sdr_raw()
            power = self.ipmi.parse_power_consumption(power_sdr_data)
            psu_statuses = self.ipmi.get_power_status(power_sdr_data)

            hottest_cpu = max(temps['cpu_temps']) if temps['cpu_temps'] else None
            target_fan_speed = "Dell Auto"

            if not self.config.get('fan_control_enabled', True):
                self._log("info", "Fan control is disabled for this server; leaving the iDRAC fan settings untouched.")
                target_fan_speed = "Monitor only"
            elif self.ipmi.fan_control_refused:
                target_fan_speed = "Dell Auto (refused)"
            elif hottest_cpu is None:
                self._log("warning", "No CPU temperature sensor found; handing fans back to the iDRAC for safety.")
                self.ipmi.apply_dell_fan_control_profile()
                target_fan_speed = "Dell Auto (no CPU sensor)"
            else:
                crit_thresh = self.fan_cfg["critical"]
                if self.fan_state.should_use_auto(hottest_cpu, crit_thresh):
                    self.ipmi.apply_dell_fan_control_profile()
                    target_fan_speed = "Dell Auto"
                else:
                    fan_mode = self.config.get('fan_mode', 'simple')

                    if fan_mode == 'simple':
                        low_thresh = self.fan_cfg["low"]
                        target_fan_speed = self.fan_state.simple_speed(
                            hottest_cpu,
                            low_thresh,
                            self.fan_cfg["base"],
                            self.fan_cfg["high"],
                        )
                        if self.ipmi.apply_user_fan_control_profile(target_fan_speed) is None:
                            target_fan_speed = "Dell Auto (fan command failed)"

                    elif fan_mode == 'target':
                        inlet = temps.get('inlet_temp')
                        speed = self.pid.update(hottest_cpu, inlet, power)
                        feed_speed = self.pid.last_feedforward if self.pid.last_feedforward is not None else 0.0
                        self._log(
                            "debug",
                            f"PID update: temp={hottest_cpu}°C, inlet={inlet}°C, power={power}W, "
                            f"feedforward={feed_speed:.1f}%, trim={self.pid.trim:.2f}%, "
                            f"theta={[round(t, 4) for t in self.pid.theta]}, samples={self.pid.samples}, "
                            f"overcooled={self.pid.overcooled}",
                        )
                        if speed is not None:
                            target_fan_speed = clamp_speed(
                                speed,
                                minimum=int(self.pid.min_speed),
                                maximum=int(self.pid.max_speed),
                            )
                            if self.ipmi.apply_user_fan_control_profile(target_fan_speed) is None:
                                target_fan_speed = "Dell Auto (fan command failed)"
                            else:
                                self._last_target_fan_speed = target_fan_speed
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
                            if self.ipmi.apply_user_fan_control_profile(target_fan_speed) is None:
                                target_fan_speed = "Dell Auto (fan command failed)"

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
                }

            self._publish_mqtt_data(status_data)

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

    def _sleep(self, seconds):
        """Sleep in small steps so SIGTERM can run cleanup promptly."""
        deadline = time.time() + seconds
        while time.time() < deadline and self.running and running:
            time.sleep(0.2)

    def _publish_mqtt_data(self, status):
        if not self.mqtt.is_connected:
            return

        sensors = {
            "shutdown_button": {"component": "button", "name": "Shutdown Server", "device_class": "restart", "icon": "mdi:server-off"},
            "hottest_cpu_temp": {"component": "sensor", "device_class": "temperature", "unit": "°C", "state_class": "measurement"},
            "inlet_temp": {"component": "sensor", "device_class": "temperature", "unit": "°C", "state_class": "measurement"},
            "exhaust_temp": {"component": "sensor", "device_class": "temperature", "unit": "°C", "state_class": "measurement"},
            "power": {"component": "sensor", "device_class": "power", "unit": "W", "state_class": "measurement", "icon": "mdi:flash"},
            "target_fan_speed": {"component": "sensor", "unit": "%", "icon": "mdi:fan-chevron-up"},
        }
        for i, _ in enumerate(status.get('cpus', [])):
            sensors[f"cpu_{i}_temp"] = {"component": "sensor", "name": f"CPU {i} Temperature", "device_class": "temperature", "unit": "°C", "state_class": "measurement"}
        for fan in status.get('fans', []):
            sensors[f"fan_{re.sub(r'[^a-zA-Z0-9_]+', '', fan['name']).lower()}_rpm"] = {"component": "sensor", "name": f"{fan['name']} RPM", "unit": "RPM", "icon": "mdi:fan"}
        for psu in status.get('psus', []):
            sensors[f"psu_{re.sub(r'[^a-zA-Z0-9_]+', '', psu['name']).lower()}"] = {"component": "binary_sensor", "name": psu['name'], "device_class": "problem"}

        for slug, desc in sensors.items():
            if slug not in self.discovered_sensors:
                cmd_topic = f"{self.mqtt.base_topic}/command/shutdown" if desc['component'] == 'button' else None
                self.mqtt.publish_discovery(
                    desc['component'], slug, desc.get('name', slug.replace("_", " ").title()),
                    desc.get('device_class'), desc.get('unit'), desc.get('icon'),
                    cmd_topic, None, desc.get('state_class'),
                )
                self.discovered_sensors.add(slug)

            if desc['component'] == 'sensor':
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

    def cleanup(self):
        if self.disk_poller:
            self.disk_poller.stop()
            self.disk_poller.join(timeout=3.0)

        if self.config.get('fan_control_enabled', True):
            self._log("info", "Worker shutting down. Reverting to Dell auto fans.")
            self.ipmi.apply_dell_fan_control_profile()
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
    web_server_port = int(os.getenv("INGRESS_PORT", 8099))
    web_thread = threading.Thread(target=web_server.run_web_server, args=(web_server_port, STATUS_FILE, status_lock), daemon=True)
    web_thread.start()

    worker_instances = []
    for server_conf in servers_configs_list:
        if server_conf.get("enabled", False):
            worker = ServerWorker(server_conf, global_options)
            worker_instances.append(worker)
            thread = threading.Thread(target=worker.run, daemon=True)
            threads.append(thread)
            thread.start()

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
    print("[MAIN] ===== HA iDRAC Controller Stopped =====", flush=True)
