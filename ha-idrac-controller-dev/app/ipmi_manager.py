# HA-iDRAC/ha-idrac-controller-dev/app/ipmi_manager.py
import subprocess
import time
import re

class IPMIManager:
    def __init__(self, ip, user, password, conn_type="lanplus", log_level="info"):
        self.ip = ip
        self.user = user
        self.password = password
        self.log_level = log_level.lower()
        self.base_args = self._build_base_args(conn_type)
        self._log("info", f"IPMI Manager initialized for host: {self.ip}")

    def _build_base_args(self, conn_type):
        if conn_type.lower() in ["local", "open"]:
            return ["-I", "open"]
        else:
            return ["-I", "lanplus", "-H", self.ip, "-U", self.user, "-P", self.password]

    def _log(self, level, message):
        levels = {"trace": -1, "debug": 0, "info": 1, "warning": 2, "error": 3, "fatal": 4}
        if levels.get(self.log_level, levels["info"]) <= levels.get(level.lower(), levels["info"]):
            print(f"[{level.upper()}] IPMI ({self.ip}): {message}", flush=True)

    def _run_ipmi_command(self, args_list, is_raw_command=True, timeout=15):
        if not self.base_args:
            self._log("error", "IPMI not configured.")
            return None

        base_command = ["ipmitool"] + self.base_args
        command_to_run = base_command + (["raw"] + args_list if is_raw_command else args_list)
        
        self._log("debug", f"Executing command: {' '.join(command_to_run)}")

        try:
            result = subprocess.run(command_to_run, capture_output=True, text=True, check=False, timeout=timeout)
            
            if result.returncode != 0:
                self._log("error", f"Command failed: {' '.join(command_to_run)}")
                self._log("error", f"STDOUT: {result.stdout.strip()}")
                self._log("error", f"STDERR: {result.stderr.strip()}")
                return None
            
            self._log("debug", f"Command STDOUT: {result.stdout.strip()}")
            return result.stdout.strip()
            
        except FileNotFoundError:
            self._log("error", "ipmitool command not found. Is it installed and in the system PATH?")
        except subprocess.TimeoutExpired:
            self._log("error", f"Command timed out: {' '.join(command_to_run)}")
        except Exception as e:
            self._log("error", f"An unexpected error occurred with command: {e}")
        return None

    def _decimal_to_hex_for_ipmi(self, decimal_value):
        try:
            val = int(decimal_value)
            if 0 <= val <= 100:
                return f"0x{val:02x}"
            self._log("warning", f"Value {val} out of range (0-100). Clamping.")
            return f"0x{max(0, min(100, val)):02x}"
        except ValueError:
            self._log("warning", f"Invalid decimal value '{decimal_value}'. Using 0x00.")
            return "0x00"

    def apply_dell_fan_control_profile(self):
        self._log("info", "Applying Dell default dynamic fan control.")
        return self._run_ipmi_command(["0x30", "0x30", "0x01", "0x01"])

    def apply_user_fan_control_profile(self, decimal_fan_speed):
        hex_fan_speed = self._decimal_to_hex_for_ipmi(decimal_fan_speed)
        self._log("info", f"Applying user static fan control: {decimal_fan_speed}% ({hex_fan_speed})")
        
        if self._run_ipmi_command(["0x30", "0x30", "0x01", "0x00"]) is None:
            self._log("error", "Failed to enable manual fan control mode.")
            return None
        time.sleep(0.5)
        
        result = self._run_ipmi_command(["0x30", "0x30", "0x02", "0xff", hex_fan_speed])
        if result is None:
            self._log("error", f"Failed to set fan speed to {hex_fan_speed}.")
        else:
            self._log("info", f"Successfully applied user fan control: {decimal_fan_speed}%")
        return result

    def get_server_model_info(self):
        self._log("info", "Retrieving server model information...")
        fru_data = self._run_ipmi_command(["fru"], is_raw_command=False, timeout=20)
        if not fru_data:
            self._log("warning", "Could not retrieve FRU data.")
            return None
        
        model_info = {"manufacturer": "Unknown", "model": "Unknown"}
        patterns = {
            "manufacturer": r"Product Manufacturer\s*:\s*(.*)",
            "model": r"Product Name\s*:\s*(.*)",
            "board_mfg": r"Board Mfg\s*:\s*(.*)",
            "board_product": r"Board Product\s*:\s*(.*)"
        }
        
        m_manuf = re.search(patterns["manufacturer"], fru_data, re.IGNORECASE)
        m_model = re.search(patterns["model"], fru_data, re.IGNORECASE)
        
        model_info["manufacturer"] = m_manuf.group(1).strip() if m_manuf else "Unknown"
        model_info["model"] = m_model.group(1).strip() if m_model else "Unknown"

        if model_info["manufacturer"] == "Unknown":
            m_board_mfg = re.search(patterns["board_mfg"], fru_data, re.IGNORECASE)
            if m_board_mfg: model_info["manufacturer"] = m_board_mfg.group(1).strip()
            
        if model_info["model"] == "Unknown":
            m_board_prod = re.search(patterns["board_product"], fru_data, re.IGNORECASE)
            if m_board_prod: model_info["model"] = m_board_prod.group(1).strip()
            
        if "dell" in model_info["manufacturer"].lower():
            model_info["manufacturer"] = "DELL"
            
        self._log("info", f"Server Info: Manufacturer='{model_info['manufacturer']}', Model='{model_info['model']}'")
        return model_info

    def retrieve_temperatures_raw(self):
        self._log("debug", "Retrieving raw temperature SDR data...")
        return self._run_ipmi_command(["sdr", "type", "temperature"], is_raw_command=False)

    def parse_temperatures(self, sdr_data, cpu_pattern_str, inlet_pattern_str, exhaust_pattern_str):
        temps = {"cpu_temps": [], "inlet_temp": None, "exhaust_temp": None}
        if not sdr_data:
            self._log("warning", "SDR data empty for temp parsing.")
            return temps

        temp_line_regex = re.compile(r"^(.*?)\s*\|\s*[\da-fA-F]+h\s*\|\s*ok\s*.*?\|\s*([-+]?\d*\.?\d+)\s*degrees C", re.IGNORECASE)
        
        for line in sdr_data.splitlines():
            match = temp_line_regex.match(line.strip())
            if not match: continue
            
            sensor_name, temp_val_str = match.groups()
            sensor_name = sensor_name.strip()
            try:
                temp_value = int(float(temp_val_str))
            except ValueError:
                continue

            if re.search(inlet_pattern_str, sensor_name, re.IGNORECASE):
                temps["inlet_temp"] = temp_value
            elif re.search(exhaust_pattern_str, sensor_name, re.IGNORECASE):
                temps["exhaust_temp"] = temp_value
            elif re.search(cpu_pattern_str, sensor_name, re.IGNORECASE):
                temps["cpu_temps"].append(temp_value)
                
        return temps

    def retrieve_fan_rpms_raw(self):
        self._log("debug", "Retrieving raw fan SDR data...")
        return self._run_ipmi_command(["sdr", "type", "fan"], is_raw_command=False, timeout=10)

    def parse_fan_rpms(self, sdr_data):
        fans = []
        if not sdr_data:
            return fans

        fan_line_regex = re.compile(r"^(.*?)\s*\|\s*[\da-fA-F]+h\s*\|\s*ok\s*.*?\|\s*([\d\.]+)\s*RPM", re.IGNORECASE)
        for line in sdr_data.splitlines():
            match = fan_line_regex.match(line.strip())
            if not match: continue
            
            fan_name, rpm_str = match.groups()
            try:
                fans.append({"name": fan_name.strip(), "rpm": int(float(rpm_str))})
            except ValueError:
                continue
        return fans

    def retrieve_power_sdr_raw(self):
        self._log("debug", "Retrieving raw power SDR data...")
        return self._run_ipmi_command(["sdr", "type", "current"], is_raw_command=False, timeout=10)

    def parse_power_consumption(self, sdr_data):
        if not sdr_data:
            self._log("warning", "SDR data is empty for power consumption parsing.")
            return None
        
        # This more robust regex looks for a line with "Pwr Consumption" and "Watts"
        power_line_regex = re.compile(r"^(Pwr Consumption.*?)\s*\|.*?\s*([\d\.]+)\s*Watts", re.IGNORECASE)
        
        for line in sdr_data.splitlines():
            match = power_line_regex.search(line)
            if match:
                try:
                    power_watts = int(float(match.group(2)))
                    self._log("debug", f"MATCHED POWER: '{match.group(1).strip()}' as {power_watts} Watts")
                    return power_watts
                except (ValueError, IndexError):
                    continue # Try next line if parsing fails
        
        self._log("warning", "Power Consumption sensor (Watts) not found in SDR data.")
        return None
    
    def get_psu_status(self):
        """Retrieves the status of all Power Supply Units."""
        self._log("debug", "Retrieving PSU status from SDR...")
        sdr_data = self._run_ipmi_command(["sdr", "elist"], is_raw_command=False)
        psu_statuses = []
        if not sdr_data:
            return psu_statuses

        # Regex to find PSU status sensors, e.g., "PSU1 Status      | 6Fh | ok  | 10.1 | Presence Detected"
        psu_regex = re.compile(r"^(PSU\d+\sStatus)\s*\|.*?\|\s*(ok|nr)")

        for line in sdr_data.splitlines():
            match = psu_regex.search(line)
            if match:
                psu_name = match.group(1).strip()
                # Status 'ok' means the PSU is healthy. 'nr' (Not Redundant) can also mean OK if it's the only one.
                # Any other status (e.g., failure, not present) will not match 'ok|nr' and be considered a problem.
                status_ok = match.group(2) in ['ok', 'nr']
                psu_statuses.append({"name": psu_name, "ok": status_ok})
                self._log("debug", f"Found PSU: {psu_name}, Status OK: {status_ok}")
        
        return psu_statuses

    def chassis_shutdown(self):
        """Sends a graceful ACPI shutdown command to the server."""
        self._log("info", "Sending graceful shutdown command to server...")
        # We use 'power soft' for a graceful shutdown.
        return self._run_ipmi_command(["chassis", "power", "soft"], is_raw_command=False)