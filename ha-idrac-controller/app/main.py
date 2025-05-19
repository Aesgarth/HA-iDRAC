# HA-iDRAC/ha-idrac-controller/app/main.py
import os
import time
import sys
import signal
import threading
import re
import json 

from . import ipmi_manager
from . import web_server
from . import mqtt_client

# --- Global Variables ---
running = True
addon_options = {}
server_info = {
    "manufacturer": "Unknown", "model": "Unknown", "is_gen14_plus": False,
    "cpu_generic_temp_pattern": None, 
    "inlet_temp_name_pattern": None, 
    "exhaust_temp_name_pattern": None
}
app_config = {} 
loop_count = 0
current_parsed_status = { # For sharing with web_server via file
    "cpu_temps_c": [], "hottest_cpu_temp_c": "N/A",
    "inlet_temp_c": "N/A", "exhaust_temp_c": "N/A",
    "target_fan_speed_percent": "N/A", "actual_fan_rpms": [],
    "last_updated": "Never"
}
STATUS_FILE = "/data/current_status.json"

# MQTT Discovery Tracking
# Use sets to store unique identifiers (slugs) of sensors for which discovery has been published
discovered_cpu_sensors = set()
discovered_fan_rpm_sensors = set()
static_sensors_discovered = False # For Inlet, Exhaust, Target Fan Speed

# --- Graceful Shutdown & Helpers ---
def graceful_shutdown(signum, frame):
    global running
    print("[MAIN] Shutdown signal received. Cleaning up...", flush=True)
    running = False

signal.signal(signal.SIGTERM, graceful_shutdown)
signal.signal(signal.SIGINT, graceful_shutdown)

def determine_server_generation(model_name):
    if not model_name: return False
    match = re.search(r"^[RT]\s?(\d)(\d)\d+", model_name.upper())
    if match:
        try:
            gen_indicator_digit = int(match.group(2))
            if gen_indicator_digit >= 4: return True
        except (IndexError, ValueError): pass
    return False

def celsius_to_fahrenheit(celsius):
    if celsius is None: return None
    return round((celsius * 9/5) + 32, 1)

def fahrenheit_to_celsius(fahrenheit):
    if fahrenheit is None: return None
    return round((fahrenheit - 32) * 5/9, 1)

def save_current_status_to_file(status_dict):
    try:
        with open(STATUS_FILE, 'w') as f:
            json.dump(status_dict, f, indent=4)
    except (IOError, PermissionError) as e:
        print(f"[ERROR] Could not save status to {STATUS_FILE}: {e}", flush=True)

# --- Main Application Logic ---
def load_and_configure(mqtt_handler): # Pass mqtt_handler to set device_info
    global addon_options, app_config, server_info
    print("[MAIN] Loading configuration and initializing...", flush=True)
    
    addon_options = {
        "idrac_ip": os.getenv("IDRAC_IP"), "idrac_username": os.getenv("IDRAC_USERNAME"),
        "idrac_password": os.getenv("IDRAC_PASSWORD"),
        "check_interval_seconds": int(os.getenv("CHECK_INTERVAL_SECONDS", "60")),
        "log_level": os.getenv("LOG_LEVEL", "info").lower(),
        "temperature_unit": os.getenv("TEMPERATURE_UNIT", "C").upper(),
        "base_fan_speed_percent": int(os.getenv("BASE_FAN_SPEED_PERCENT", "20")),
        "low_temp_threshold": int(os.getenv("LOW_TEMP_THRESHOLD", "45")),
        "high_temp_fan_speed_percent": int(os.getenv("HIGH_TEMP_FAN_SPEED_PERCENT", "50")),
        "critical_temp_threshold": int(os.getenv("CRITICAL_TEMP_THRESHOLD", "65")),
        "mqtt_host": os.getenv("MQTT_HOST", "core-mosquitto"),
        "mqtt_port": int(os.getenv("MQTT_PORT", "1883")),
        "mqtt_username": os.getenv("MQTT_USERNAME", ""),
        "mqtt_password": os.getenv("MQTT_PASSWORD", "")
    }
    log_level = addon_options['log_level']
    print(f"[{log_level.upper()}] Add-on options loaded: IDRAC_IP={addon_options['idrac_ip']}, LogLevel={log_level}", flush=True)

    ipmi_manager.configure_ipmi(
        addon_options["idrac_ip"], addon_options["idrac_username"], 
        addon_options["idrac_password"], log_level=log_level
    )

    model_data = ipmi_manager.get_server_model_info()
    if model_data and model_data.get("model") != "Unknown":
        server_info.update(model_data)
        server_info["is_gen14_plus"] = determine_server_generation(server_info["model"])
        print(f"[{log_level.upper()}] Server: {server_info['manufacturer']} {server_info['model']} (Gen14+: {server_info['is_gen14_plus']})", flush=True)
    else:
        print(f"[WARNING] Could not determine server model.", flush=True)
    
    if mqtt_handler: # Configure MQTT client with device info
        mqtt_handler.configure_broker(
            addon_options["mqtt_host"], addon_options["mqtt_port"],
            addon_options["mqtt_username"], addon_options["mqtt_password"],
            log_level
        )
        mqtt_handler.set_device_info(
            server_info.get("manufacturer"), 
            server_info.get("model"), 
            addon_options.get("idrac_ip")
        )

    server_info["cpu_generic_temp_pattern"] = r"^Temp$" 
    server_info["inlet_temp_name_pattern"] = r"Inlet Temp"
    server_info["exhaust_temp_name_pattern"] = r"Exhaust Temp"
    print(f"[{log_level.upper()}] Using temp patterns: CPU_generic='{server_info['cpu_generic_temp_pattern']}', Inlet='{server_info['inlet_temp_name_pattern']}', Exhaust='{server_info['exhaust_temp_name_pattern']}'", flush=True)

    app_config = web_server.load_app_config()
    print(f"[{log_level.upper()}] Loaded app config: {app_config}", flush=True)

    temp_unit = addon_options["temperature_unit"]
    if temp_unit == "F":
        addon_options["low_temp_threshold_c"] = fahrenheit_to_celsius(addon_options["low_temp_threshold"])
        addon_options["critical_temp_threshold_c"] = fahrenheit_to_celsius(addon_options["critical_temp_threshold"])
        print(f"[{log_level.upper()}] Temp thresholds (F input converted to C): Low={addon_options['low_temp_threshold_c']:.1f}C, Critical={addon_options['critical_temp_threshold_c']:.1f}C", flush=True)
    else:
        addon_options["low_temp_threshold_c"] = float(addon_options["low_temp_threshold"])
        addon_options["critical_temp_threshold_c"] = float(addon_options["critical_temp_threshold"])
        print(f"[{log_level.upper()}] Temp thresholds (C input): Low={addon_options['low_temp_threshold_c']}C, Critical={addon_options['critical_temp_threshold_c']}C", flush=True)


def main_control_loop(mqtt_handler):
    global running, app_config, addon_options, server_info, loop_count, current_parsed_status
    global discovered_cpu_sensors, discovered_fan_rpm_sensors, static_sensors_discovered 
    log_level = addon_options['log_level']

    if not (addon_options["idrac_ip"] and addon_options["idrac_username"] and addon_options["idrac_password"]):
        print("[ERROR] iDRAC credentials not fully configured. Exiting.", flush=True)
        return 
    
    print(f"[{log_level.upper()}] Entering main control loop. Interval: {addon_options['check_interval_seconds']}s", flush=True)

    # Static sensor discovery is now handled in mqtt_client.on_connect
    # Dynamic discovery for CPUs and Fans will happen within the loop as needed

    while running:
        start_time = time.time()
        print(f"[{log_level.upper()}] --- Cycle {loop_count + 1} Start ---", flush=True)

        if loop_count > 0 and loop_count % 5 == 0:
             print(f"[{log_level.upper()}] Reloading app config from /data/app_config.json", flush=True)
             app_config = web_server.load_app_config()

        # --- Temperatures ---
        raw_temp_sdr_data = ipmi_manager.retrieve_temperatures_raw()
        parsed_temperatures_c = {"cpu_temps": [], "inlet_temp": None, "exhaust_temp": None}
        if raw_temp_sdr_data:
            # ... (logging raw_temp_sdr_data if trace/debug) ...
            parsed_temperatures_c = ipmi_manager.parse_temperatures(
                raw_temp_sdr_data, server_info["cpu_generic_temp_pattern"],
                server_info["inlet_temp_name_pattern"], server_info["exhaust_temp_name_pattern"]
            )
            print(f"[{log_level.upper()}] Parsed Temperatures (C): {parsed_temperatures_c}", flush=True)
        else:
            print(f"[WARNING] Failed to retrieve temp SDR data.", flush=True)

        # --- Fan RPMs ---
        raw_fan_sdr_data = ipmi_manager.retrieve_fan_rpms_raw()
        parsed_fan_rpms = []
        if raw_fan_sdr_data:
            # ... (logging raw_fan_sdr_data if trace/debug) ...
            parsed_fan_rpms = ipmi_manager.parse_fan_rpms(raw_fan_sdr_data)
            print(f"[{log_level.upper()}] Parsed Fan RPMs: {parsed_fan_rpms}", flush=True)
        else:
            print(f"[WARNING] Failed to retrieve fan SDR data.", flush=True)

        # --- Power Consumption (NEW) ---
        raw_power_sdr_data = ipmi_manager.retrieve_power_sdr_raw()
        power_consumption_watts = None
        if raw_power_sdr_data:
            if log_level in ["trace", "debug"]:
                 print(f"[{log_level.upper()}] RAW POWER SDR DATA:\n{raw_power_sdr_data}\n-------------------------", flush=True)
            power_consumption_watts = ipmi_manager.parse_power_consumption(raw_power_sdr_data)
            print(f"[{log_level.upper()}] Parsed Power Consumption: {power_consumption_watts}W", flush=True)
        else:
            print(f"[WARNING] Failed to retrieve power SDR data.", flush=True)


        # --- Dynamic MQTT Discovery (CPUs and Fans) ---
        if mqtt_handler and mqtt_handler.is_connected:
            # CPU Temp Discovery 
            cpu_temps_list_c_current_cycle = parsed_temperatures_c.get("cpu_temps", [])
            if len(discovered_cpu_sensors) != len(cpu_temps_list_c_current_cycle): # Or a more robust check
                print(f"[{log_level.upper()}] CPU count changed/initial. Discovering {len(cpu_temps_list_c_current_cycle)} CPUs.", flush=True)
                new_cpu_slugs = set()
                for i in range(len(cpu_temps_list_c_current_cycle)):
                    slug = f"cpu_{i}_temp"
                    mqtt_handler.publish_sensor_discovery(
                        sensor_type_slug=slug, sensor_name=f"CPU {i} Temperature",
                        device_class="temperature", unit_of_measurement="°C",
                        value_template="{{ value_json.temperature | round(1) }}"
                    )
                    new_cpu_slugs.add(slug)
                discovered_cpu_sensors = new_cpu_slugs
            
            # Fan RPM Discovery
            for i, fan_info in enumerate(parsed_fan_rpms):
                fan_name = fan_info["name"]
                safe_fan_name_slug = re.sub(r'[^a-zA-Z0-9_]+', '_', fan_name).lower().strip('_')
                if not safe_fan_name_slug: safe_fan_name_slug = f"fan_{i}"
                rpm_sensor_slug = f"fan_{safe_fan_name_slug}_rpm" # This is the sensor_type_slug
                if rpm_sensor_slug not in discovered_fan_rpm_sensors:
                    mqtt_handler.publish_sensor_discovery(
                        sensor_type_slug=rpm_sensor_slug, # Used to build unique_id and state_topic
                        sensor_name=f"{fan_name} RPM",
                        unit_of_measurement="RPM", icon="mdi:fan",
                        value_template="{{ value_json.rpm | round(0) }}"
                    )
                    discovered_fan_rpm_sensors.add(rpm_sensor_slug)

        # --- Determine Hottest CPU & Fan Control Logic (keep as is) ---
        # ... (hottest_cpu_temp_c logic) ...
        # ... (target_fan_speed_display logic and ipmi_manager calls for fan control) ...
        # Example of setting target_fan_speed_display from your previous logs
        hottest_cpu_temp_c = None
        cpu_temps_list_c = parsed_temperatures_c.get("cpu_temps", [])
        if cpu_temps_list_c:
            hottest_cpu_temp_c = max(cpu_temps_list_c)
        
        target_fan_speed_display = "N/A" # For status and MQTT
        if hottest_cpu_temp_c is not None:
            low_thresh_c = addon_options["low_temp_threshold_c"]
            crit_thresh_c = addon_options["critical_temp_threshold_c"]
            if hottest_cpu_temp_c >= crit_thresh_c:
                # ... (Dell auto profile) ...
                target_fan_speed_display = "Dell Auto"
            elif hottest_cpu_temp_c >= low_thresh_c:
                target_fan_speed_val = addon_options["high_temp_fan_speed_percent"]
                # ... (apply user profile) ...
                target_fan_speed_display = target_fan_speed_val
            else: 
                target_fan_speed_val = addon_options["base_fan_speed_percent"]
                # ... (apply user profile) ...
                target_fan_speed_display = target_fan_speed_val
        else:
            # ... (Dell auto profile) ...
            target_fan_speed_display = "Dell Auto (Safety)"
        
        # --- Update Shared Status File for Web UI ---
        current_parsed_status = {
            "cpu_temps_c": cpu_temps_list_c,
            "hottest_cpu_temp_c": hottest_cpu_temp_c,
            "inlet_temp_c": parsed_temperatures_c.get("inlet_temp"),
            "exhaust_temp_c": parsed_temperatures_c.get("exhaust_temp"),
            "target_fan_speed_percent": target_fan_speed_display,
            "actual_fan_rpms": parsed_fan_rpms,
            "power_consumption_watts": power_consumption_watts, # ADDED
            "last_updated": time.strftime("%Y-%m-%d %H:%M:%S %Z")
        }
        save_current_status_to_file(current_parsed_status)

        # --- MQTT State Publishing ---
        if mqtt_handler and mqtt_handler.is_connected:
            # CPU Temps
            for i, cpu_temp_val in enumerate(cpu_temps_list_c):
                mqtt_handler.publish_sensor_state(sensor_type_slug=f"cpu_{i}_temp", value_dict={"temperature": cpu_temp_val})
            # Inlet/Exhaust
            if parsed_temperatures_c.get("inlet_temp") is not None:
                 mqtt_handler.publish_sensor_state(sensor_type_slug="inlet_temp", value_dict={"temperature": parsed_temperatures_c["inlet_temp"]})
            if parsed_temperatures_c.get("exhaust_temp") is not None:
                 mqtt_handler.publish_sensor_state(sensor_type_slug="exhaust_temp", value_dict={"temperature": parsed_temperatures_c["exhaust_temp"]})
            # Hottest CPU
            if hottest_cpu_temp_c is not None:
                 mqtt_handler.publish_sensor_state(sensor_type_slug="hottest_cpu_temp", value_dict={"temperature": hottest_cpu_temp_c})
            # Target Fan Speed
            if target_fan_speed_display not in ["N/A", "Dell Auto", "Dell Auto (Safety)"]:
                mqtt_handler.publish_sensor_state(sensor_type_slug="target_fan_speed", value_dict={"speed": int(target_fan_speed_display)})
            else: 
                 mqtt_handler.publish_sensor_state(sensor_type_slug="target_fan_speed", value_dict={"speed": None}) 
            # Power Consumption (NEW)
            if power_consumption_watts is not None:
                mqtt_handler.publish_sensor_state(sensor_type_slug="power_consumption", value_dict={"power": power_consumption_watts})
            # Actual Fan RPMs
            for i, fan_info in enumerate(parsed_fan_rpms):
                fan_name = fan_info["name"]
                safe_fan_name_slug = re.sub(r'[^a-zA-Z0-9_]+', '_', fan_name).lower().strip('_')
                if not safe_fan_name_slug: safe_fan_name_slug = f"fan_{i}" # Fallback if name is all special chars
                mqtt_handler.publish_sensor_state(sensor_type_slug=f"fan_{safe_fan_name_slug}_rpm", value_dict={"rpm": fan_info["rpm"]})

        print(f"[{log_level.upper()}] --- Cycle {loop_count + 1} End ---", flush=True)
        loop_count += 1
        # ... (sleep logic as before) ...

        
        for _ in range(int(sleep_duration / 0.1)):
            if not running: break
            time.sleep(0.1)
        if not running: break

# --- Global mqtt_handler_instance ---
mqtt_handler_instance = None

if __name__ == "__main__":
    print("[MAIN] ===== HA iDRAC Controller Python Application Starting =====", flush=True)
    
    mqtt_handler_instance = mqtt_client.MqttClient() # Create instance early

    load_and_configure(mqtt_handler_instance) # Pass instance to configure it

    if addon_options.get("mqtt_host") and addon_options["mqtt_host"] != "YOUR_MQTT_BROKER_IP_OR_HOSTNAME":
        # Client ID should be unique, can be based on some config or generated
        mqtt_handler_instance.client_id = f"ha_idrac_controller_{addon_options.get('idrac_ip','unknown').replace('.','_')}"
        mqtt_handler_instance.connect()
    else:
        print("[INFO] MQTT host not configured or is default placeholder. MQTT client will not connect.", flush=True)
        mqtt_handler_instance = None 

    web_server_port = 8099 
    web_thread = threading.Thread(target=web_server.run_web_server, args=(web_server_port,), daemon=True)
    web_thread.start()
    print(f"[MAIN] Admin Web Panel server starting in background thread on port {web_server_port}...", flush=True)

    try:
        main_control_loop(mqtt_handler_instance)
    except Exception as e:
        print(f"[FATAL] Unhandled exception in main execution: {e}", flush=True)
        import traceback; traceback.print_exc(file=sys.stdout); sys.stdout.flush()
    finally:
        print("[MAIN] Main execution finished. Initiating final cleanup...", flush=True)
        if addon_options.get("idrac_ip") and ipmi_manager._IPMI_BASE_ARGS: 
            print("[MAIN] Attempting to set fans to Dell default profile...", flush=True)
            ipmi_manager.apply_dell_fan_control_profile()
        if mqtt_handler_instance and getattr(mqtt_handler_instance, 'is_connected', False):
            mqtt_handler_instance.disconnect()
        print("[MAIN] ===== HA iDRAC Controller Python Application Stopped =====", flush=True)
        sys.stdout.flush()