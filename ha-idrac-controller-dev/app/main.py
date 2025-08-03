# HA-iDRAC/ha-idrac-controller-dev/app/main.py
import os
import time
import sys
import signal
import threading
import re
import json
from .ipmi_manager import IPMIManager
from .mqtt_client import MqttClient
from .pid_controller import PIDController # Import the new PID class
from . import web_server

# --- Global Variables ---
running = True
threads = []
status_lock = threading.Lock()
ALL_SERVERS_STATUS = {}
STATUS_FILE = "/data/current_status.json"
PID_STATE_FILE = "/data/pid_states.json"

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
        
        self.ipmi = IPMIManager(ip=self.config['idrac_ip'], user=self.config['idrac_username'], password=self.config['idrac_password'], log_level=self.log_level)
        self.mqtt = MqttClient(client_id=f"ha_idrac_{self.alias}")
        self.pid = PIDController()

        self.server_info = {}
        self.discovered_sensors = set()

    def _log(self, level, message):
        print(f"[{level.upper()}] [{self.alias}] {message}", flush=True)

    def _on_mqtt_message(self, topic, payload):
        command_topic = f"{self.mqtt.base_topic}/command/shutdown"
        if topic == command_topic and payload == "PRESS":
            self._log("info", "Shutdown command received via MQTT.")
            self.ipmi.chassis_shutdown()

    def _initialize(self):
        self._log("info", "Initializing server worker...")
        
        pid_config = self.config.get('pid_config', {})
        self.pid.setpoint = pid_config.get('target_temp', 55)
        self.pid.set_gains(pid_config.get('kp', 4.0), pid_config.get('ki', 0.2), pid_config.get('kd', 0.1))

        if os.path.exists(PID_STATE_FILE):
            with open(PID_STATE_FILE, 'r') as f:
                try:
                    all_states = json.load(f)
                    if self.alias in all_states:
                        self.pid.load_state(all_states[self.alias])
                        self._log("info", f"Loaded persistent PID state: integral={self.pid.integral}")
                except json.JSONDecodeError:
                    self._log("warning", "Could not decode PID state file.")

        model_data = self.ipmi.get_server_model_info()
        if model_data: self.server_info.update(model_data)
        
        self.mqtt.configure_broker(self.global_opts["mqtt_host"], self.global_opts["mqtt_port"], self.global_opts["mqtt_username"], self.global_opts["mqtt_password"], self.log_level)
        self.mqtt.set_device_info(server_alias=self.alias, manufacturer=self.server_info.get("manufacturer"), model=self.server_info.get("model"), ip_address=self.config.get("idrac_ip"))
        self.mqtt.connect()
        self.mqtt.message_callback = self._on_mqtt_message
        self.mqtt.subscribe(f"{self.mqtt.base_topic}/command/shutdown")

        for _ in range(10):
            if self.mqtt.is_connected: return True
            time.sleep(1)
        return False

    def run(self):
        if not self._initialize():
            self._log("error", "Initialization failed. Stopping worker.")
            return

        while self.running and running:
            start_time = time.time()
            
            raw_temp_data = self.ipmi.retrieve_temperatures_raw()
            if raw_temp_data is None:
                self.mqtt.publish(self.mqtt.availability_topic, "offline", retain=True)
                time.sleep(60)
                continue

            self.mqtt.publish(self.mqtt.availability_topic, "online", retain=True)
            
            temps = self.ipmi.parse_temperatures(raw_temp_data, r"Temp", r"Inlet Temp", r"Exhaust Temp")
            fans = self.ipmi.parse_fan_rpms(self.ipmi.retrieve_fan_rpms_raw())
            power_sdr_data = self.ipmi.retrieve_power_sdr_raw()
            power = self.ipmi.parse_power_consumption(power_sdr_data)
            psu_statuses = self.ipmi.get_power_status(power_sdr_data)

            hottest_cpu = max(temps['cpu_temps']) if temps['cpu_temps'] else None
            fan_mode = self.config.get('fan_mode', 'simple')
            target_fan_speed = "Dell Auto"
            
            if hottest_cpu:
                crit_thresh = self.config.get('critical_temp_threshold', 65)
                
                if hottest_cpu >= crit_thresh:
                    self.ipmi.apply_dell_fan_control_profile()
                elif fan_mode == 'simple':
                    low_thresh = self.config.get('low_temp_threshold', 45)
                    if hottest_cpu >= low_thresh: target_fan_speed = self.config.get('high_temp_fan_speed_percent', 50)
                    else: target_fan_speed = self.config.get('base_fan_speed_percent', 20)
                    self.ipmi.apply_user_fan_control_profile(target_fan_speed)
                elif fan_mode == 'target':
                    # Get the base fan speed to use with the PID controller
                    base_fan = self.config.get('base_fan_speed_percent', 20)
                    speed = self.pid.update(hottest_cpu, base_fan) # Pass the base speed
                    if speed is not None:
                        target_fan_speed = speed
                        self.ipmi.apply_user_fan_control_profile(target_fan_speed)
                elif fan_mode == 'curve':
                    fan_curve = self.config.get('fan_curve', [])
                    if len(fan_curve) >= 2:
                        lower, upper = fan_curve[0], fan_curve[-1]
                        for i in range(len(fan_curve) - 1):
                            if fan_curve[i]['temp'] <= hottest_cpu < fan_curve[i+1]['temp']:
                                lower, upper = fan_curve[i], fan_curve[i+1]; break
                        
                        if hottest_cpu < lower['temp']: speed = lower['speed']
                        elif hottest_cpu >= upper['temp']: speed = upper['speed']
                        else:
                            temp_range = upper['temp'] - lower['temp']
                            speed_range = upper['speed'] - lower['speed']
                            speed = lower['speed'] + ((hottest_cpu - lower['temp']) / temp_range * speed_range) if temp_range > 0 else lower['speed']
                        
                        target_fan_speed = int(speed)
                        self.ipmi.apply_user_fan_control_profile(target_fan_speed)
            else:
                 self.ipmi.apply_dell_fan_control_profile()

            status_data = {"hottest_cpu_temp": hottest_cpu, "inlet_temp": temps.get('inlet_temp'), "exhaust_temp": temps.get('exhaust_temp'), "power": power, "target_fan_speed": None if isinstance(target_fan_speed, str) else target_fan_speed, "cpus": temps.get('cpu_temps', []), "fans": fans, "psus": psu_statuses}
            with status_lock:
                ALL_SERVERS_STATUS[self.alias] = {"alias": self.alias, "ip": self.config['idrac_ip'], "last_updated": time.strftime("%Y-%m-%d %H:%M:%S %Z"), "hottest_cpu_temp_c": hottest_cpu, "inlet_temp_c": temps.get('inlet_temp'), "exhaust_temp_c": temps.get('exhaust_temp'), "power_consumption_watts": power, "target_fan_speed_percent": target_fan_speed, "cpu_temps_c": temps.get('cpu_temps', []), "actual_fan_rpms": fans, "psu_statuses": psu_statuses}
            
            self._publish_mqtt_data(status_data)
            time.sleep(max(0.1, self.global_opts["check_interval_seconds"] - (time.time() - start_time)))


    def _publish_mqtt_data(self, status):
        sensors = {
            "shutdown_button": {"component": "button", "name": "Shutdown Server", "device_class": "restart", "icon": "mdi:server-off"},
            "hottest_cpu_temp": {"component": "sensor", "device_class": "temperature", "unit": "°C"},
            "inlet_temp": {"component": "sensor", "device_class": "temperature", "unit": "°C"},
            "exhaust_temp": {"component": "sensor", "device_class": "temperature", "unit": "°C"},
            "power": {"component": "sensor", "device_class": "power", "unit": "W", "state_class": "measurement", "icon": "mdi:flash"},
            "target_fan_speed": {"component": "sensor", "unit": "%", "icon": "mdi:fan-chevron-up"},
        }
        for i, _ in enumerate(status.get('cpus', [])): sensors[f"cpu_{i}_temp"] = {"component": "sensor", "name": f"CPU {i} Temperature", "device_class": "temperature", "unit": "°C"}
        for fan in status.get('fans', []): sensors[f"fan_{re.sub(r'[^a-zA-Z0-9_]+', '', fan['name']).lower()}_rpm"] = {"component": "sensor", "name": f"{fan['name']} RPM", "unit": "RPM", "icon": "mdi:fan"}
        for psu in status.get('psus', []): sensors[f"psu_{re.sub(r'[^a-zA-Z0-9_]+', '', psu['name']).lower()}"] = {"component": "binary_sensor", "name": psu['name'], "device_class": "problem"}

        for slug, desc in sensors.items():
            if slug not in self.discovered_sensors:
                cmd_topic = f"{self.mqtt.base_topic}/command/shutdown" if desc['component'] == 'button' else None
                self.mqtt.publish_discovery(desc['component'], slug, desc.get('name', slug.replace("_", " ").title()), desc.get('device_class'), desc.get('unit'), desc.get('icon'), cmd_topic, None, desc.get('state_class'))
                self.discovered_sensors.add(slug)
            
            if desc['component'] == 'sensor':
                value = None
                if slug.startswith('fan_'): value = next((f['rpm'] for f in status['fans'] if f"fan_{re.sub(r'[^a-zA-Z0-9_]+', '', f['name']).lower()}_rpm" == slug), None)
                elif slug.startswith('cpu_'): value = status['cpus'][int(slug.split('_')[1])] if int(slug.split('_')[1]) < len(status['cpus']) else None
                else: value = status.get(slug)
                self.mqtt.publish_state('sensor', slug, value)
            elif desc['component'] == 'binary_sensor' and slug.startswith('psu_'):
                psu_name = desc['name']
                psu_data = next((p for p in status['psus'] if p['name'] == psu_name), None)
                state = "ON" if psu_data and not psu_data['ok'] else "OFF"
                self.mqtt.publish_state('binary_sensor', slug, state)

    def cleanup(self):
        self._log("info", "Worker shutting down. Reverting to Dell auto fans.")
        self.ipmi.apply_dell_fan_control_profile()
        if self.mqtt.is_connected: self.mqtt.disconnect()
        
        if os.path.exists(PID_STATE_FILE):
            try:
                with open(PID_STATE_FILE, 'r') as f:
                    all_states = json.load(f)
            except (IOError, json.JSONDecodeError):
                all_states = {}
        else:
            all_states = {}
        all_states[self.alias] = self.pid.get_state()
        with open(PID_STATE_FILE, 'w') as f:
            json.dump(all_states, f, indent=4)
        self._log("info", f"Saved persistent PID state: integral={self.pid.integral}")

    def stop(self):
        self.running = False

# --- Main Execution ---
if __name__ == "__main__":
    print("[MAIN] ===== HA iDRAC Multi-Server Controller Starting =====", flush=True)

    global_options = {
        "log_level": os.getenv("LOG_LEVEL", "info"), "check_interval_seconds": int(os.getenv("CHECK_INTERVAL_SECONDS", 60)),
        "mqtt_host": os.getenv("MQTT_HOST", "core-mosquitto"), "mqtt_port": int(os.getenv("MQTT_PORT", 1883)),
        "mqtt_username": os.getenv("MQTT_USERNAME", ""), "mqtt_password": os.getenv("MQTT_PASSWORD", ""),
        "base_fan_speed_percent": int(os.getenv("BASE_FAN_SPEED_PERCENT", 20)), "low_temp_threshold": int(os.getenv("LOW_TEMP_THRESHOLD", 45)),
        "high_temp_fan_speed_percent": int(os.getenv("HIGH_TEMP_FAN_SPEED_PERCENT", 50)), "critical_temp_threshold": int(os.getenv("CRITICAL_TEMP_THRESHOLD", 65)),
    }

    servers_configs_list = []
    if not os.path.exists("/data/servers_config.json"):
        with open("/data/servers_config.json", 'w') as f: json.dump([], f)
    else:
        with open("/data/servers_config.json", 'r') as f:
            try: servers_configs_list = json.load(f)
            except json.JSONDecodeError: pass

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

    try:
        while running:
            with status_lock:
                with open(STATUS_FILE, 'w') as f: json.dump(list(ALL_SERVERS_STATUS.values()), f, indent=4)
            time.sleep(2)
    except KeyboardInterrupt:
        graceful_shutdown(None, None)

    print("[MAIN] Waiting for all server threads to terminate...", flush=True)
    for worker in worker_instances: worker.stop()
    for thread in threads: thread.join(timeout=1)
    print("[MAIN] ===== HA iDRAC Controller Stopped =====", flush=True)