# HA-iDRAC/ha-idrac-controller-dev/app/ipmi_manager.py
"""IPMI access for the iDRAC add-on.

The parsers below are written for both 12th and 13th generation PowerEdge
servers. 13th generation machines (R630/R730/T430/...) name their CPU
temperature sensors "CPU1 Temp"/"CPU2 Temp" and expose them as processor
entities (3.x), while 12th generation machines use a sensor literally named
"Temp" (also on entity 3.x). Entity IDs are therefore used first and names
only as a fallback.
"""
import os
import re
import subprocess
import time

MIN_FAN_PERCENT = 5
MAX_FAN_PERCENT = 100

# "CPU1 Temp        | 01h | ok  |  3.1 | 45 degrees C"
_TEMP_LINE_RE = re.compile(
    r"^(?P<name>.+?)\s*\|\s*[0-9a-fA-F]+h\s*\|\s*(?P<status>ok|ns|nr|cr|u)\s*\|"
    r"\s*(?P<entity>\d+\.\d+)\s*\|\s*(?P<value>[-+]?\d*\.?\d+)\s*degrees C",
    re.IGNORECASE,
)
# "Fan1A Tach       | 30h | ok  |  7.1 | 2040 RPM"
_FAN_LINE_RE = re.compile(
    r"^(?P<name>.+?)\s*\|\s*[0-9a-fA-F]+h\s*\|\s*(?P<status>ok|ns|nr|cr|u)\s*\|"
    r"\s*(?P<entity>\d+\.\d+)\s*\|\s*(?P<value>[\d.]+)\s*RPM",
    re.IGNORECASE,
)
# "Pwr Consumption  | 77h | ok | 10.1 | 196 Watts" / "System Level | 126.000 | Watts"
_POWER_RE = re.compile(
    r"^(?:Pwr Consumption|System Level)\b.*?\|\s*(?P<value>[\d.]+)\s*Watts",
    re.IGNORECASE,
)
# "PS1 Status       | 0Ah | ok | 10.1 | Presence detected"
_PSU_STATUS_RE = re.compile(
    r"^PS(?P<index>\d)\s+Status\s*\|\s*[0-9a-fA-F]+h\s*\|\s*(?P<status>ok|ns|nr|cr|u)\s*\|"
    r"[^|]*\|\s*(?P<reading>.*)$",
    re.IGNORECASE,
)
# "PS1 PG Fail      | 0Ch | ok | 10.1 |"
_PSU_FAIL_RE = re.compile(
    r"^PS(?P<index>\d)\s+PG Fail\s*\|\s*[0-9a-fA-F]+h\s*\|\s*(?P<status>ok|ns|nr|cr|u)",
    re.IGNORECASE,
)
_CPU_NAME_RE = re.compile(r"(?:^|\b)CPU\s?\d", re.IGNORECASE)


class IPMIManager:
    def __init__(self, ip, user, password, conn_type="lanplus", log_level="info"):
        self.ip = ip
        self.user = user
        self.password = password
        self.log_level = log_level.lower()
        self.base_args = self._build_base_args(conn_type)
        self.last_stderr = ""
        # Set when the iDRAC answers "I do not have that command" or "not for
        # this account". Only ever set while no fan command has succeeded, so
        # a later failure can never leave the fans pinned at a user speed.
        self.fan_control_refused = False
        self.fan_control_ever_accepted = False
        self._log("info", f"IPMI Manager initialized for host: {self.ip}")

    def _build_base_args(self, conn_type):
        if conn_type.lower() in ["local", "open"]:
            return ["-I", "open"]
        # -E makes ipmitool read IPMI_PASSWORD from the environment instead of
        # exposing the password in the process list via -P.
        return ["-I", "lanplus", "-H", self.ip, "-U", self.user, "-E"]

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

        env = os.environ.copy()
        if self.password:
            env["IPMI_PASSWORD"] = self.password

        try:
            result = subprocess.run(
                command_to_run, capture_output=True, text=True, check=False,
                timeout=timeout, env=env,
            )

            if result.returncode != 0:
                self.last_stderr = result.stderr.strip()
                self._log("error", f"Command failed: {' '.join(command_to_run)}")
                self._log("error", f"STDOUT: {result.stdout.strip()}")
                self._log("error", f"STDERR: {self.last_stderr}")
                return None

            self.last_stderr = ""

            self._log("debug", f"Command STDOUT: {result.stdout.strip()}")
            return result.stdout.strip()

        except FileNotFoundError:
            self._log("error", "ipmitool command not found. Is it installed and in the system PATH?")
        except subprocess.TimeoutExpired:
            self._log("error", f"Command timed out: {' '.join(command_to_run)}")
        except Exception as e:
            self._log("error", f"An unexpected error occurred with command: {e}")
        return None

    def _clamp_fan_speed(self, decimal_value):
        """Clamp a requested fan speed into the safe 5-100% range."""
        try:
            val = int(decimal_value)
        except (TypeError, ValueError):
            self._log("warning", f"Invalid decimal value '{decimal_value}'. Using {MIN_FAN_PERCENT}%.")
            val = MIN_FAN_PERCENT
        if val < MIN_FAN_PERCENT:
            self._log("warning", f"Fan speed {val}% is below the safe minimum. Clamping to {MIN_FAN_PERCENT}%.")
            val = MIN_FAN_PERCENT
        if val > MAX_FAN_PERCENT:
            self._log("warning", f"Fan speed {val}% is above 100%. Clamping to 100%.")
            val = MAX_FAN_PERCENT
        return val

    def _decimal_to_hex_for_ipmi(self, decimal_value):
        val = self._clamp_fan_speed(decimal_value)
        return f"0x{val:02x}"

    # --- Fan control refusal handling -------------------------------------
    # iDRAC 9/10 and accounts below Administrator answer the Dell OEM fan
    # commands with 0xc1/0xd5 (no such command) or 0xd4 (privilege). Those are
    # permanent for the life of the add-on, so retrying them every cycle only
    # produces noise. A 0xcc ("invalid data field") is not a refusal of fan
    # control itself and is handled as a failed write.
    _REFUSAL_SIGNATURES = ("rsp=0xc1", "rsp=0xd5", "rsp=0xd4")

    def _is_fan_control_refusal(self):
        if self.fan_control_ever_accepted:
            return False
        stderr = (self.last_stderr or "").lower()
        return any(sig in stderr for sig in self._REFUSAL_SIGNATURES)

    def _note_fan_control_refusal(self):
        if self.fan_control_refused or not self._is_fan_control_refusal():
            return
        self.fan_control_refused = True
        self._log(
            "warning",
            "This iDRAC refused the Dell fan-control command (0xc1/0xd4/0xd5). "
            "It is either a 14G+ iDRAC, or the configured account is not an "
            "Administrator. Fan control is disabled and temperatures will keep "
            "being monitored.",
        )

    def apply_dell_fan_control_profile(self, force=False):
        if self.fan_control_refused and not force:
            self._log("debug", "Skipping Dell fan profile: this server refused fan control.")
            return None
        self._log("info", "Applying Dell default dynamic fan control.")
        result = self._run_ipmi_command(["0x30", "0x30", "0x01", "0x01"])
        if result is None and not self._is_fan_control_refusal():
            # This is the safety-critical hand-back; one retry for a transient
            # BMC/session error before reporting failure.
            time.sleep(0.5)
            self._log("warning", "Retrying Dell default fan control once.")
            result = self._run_ipmi_command(["0x30", "0x30", "0x01", "0x01"])
        if result is None:
            self._note_fan_control_refusal()
        return result

    def apply_user_fan_control_profile(self, decimal_fan_speed):
        if self.fan_control_refused:
            self._log("debug", "Skipping user fan profile: this server refused fan control.")
            return None

        speed = self._clamp_fan_speed(decimal_fan_speed)
        hex_fan_speed = f"0x{speed:02x}"
        self._log("info", f"Applying user static fan control: {speed}% ({hex_fan_speed})")

        if self._run_ipmi_command(["0x30", "0x30", "0x01", "0x00"]) is None:
            self._log("error", "Failed to enable manual fan control mode.")
            self._note_fan_control_refusal()
            return None
        time.sleep(0.5)

        result = self._run_ipmi_command(["0x30", "0x30", "0x02", "0xff", hex_fan_speed])
        if result is None:
            # The manual command landed but the speed did not. Leaving the fans
            # in manual mode would pin them at whatever the previous cycle set,
            # so hand them straight back to Dell's dynamic profile.
            self._log("error", f"Failed to set fan speed to {hex_fan_speed}; handing fans back to Dell auto.")
            self.apply_dell_fan_control_profile(force=True)
            return None

        self.fan_control_ever_accepted = True
        self._log("info", f"Successfully applied user fan control: {speed}%")
        return result

    # --- Third-party PCIe card cooling response ---------------------------
    # Dell OEM command from the 12G/13G IPMI set (same bytes used by
    # tigerblue77's controller): the last-but-two byte is 0x00 to leave Dell's
    # default cooling response enabled and 0x01 to disable it.
    _PCIE_COOLING_RESPONSE_CMD = [
        "0x30", "0xce", "0x00", "0x16", "0x05", "0x00", "0x00", "0x00", "0x05", "0x00",
    ]

    def set_third_party_pcie_cooling_response(self, enabled):
        payload = self._PCIE_COOLING_RESPONSE_CMD + ["0x00" if enabled else "0x01", "0x00", "0x00"]
        self._log("info", f"Setting third-party PCIe cooling response to {'Dell default' if enabled else 'disabled'}.")
        return self._run_ipmi_command(payload) is not None

    def get_firmware_version(self):
        output = self._run_ipmi_command(["mc", "info"], is_raw_command=False, timeout=20)
        if not output:
            return None
        match = re.search(r"Firmware Revision\s*:\s*(\S+)", output, re.IGNORECASE)
        return match.group(1) if match else None

    def get_server_model_info(self):
        self._log("info", "Retrieving server model information...")
        fru_data = self._run_ipmi_command(["fru"], is_raw_command=False, timeout=20)
        if not fru_data:
            self._log("warning", "Could not retrieve FRU data.")
            return None

        model_info = {"manufacturer": "Unknown", "model": "Unknown", "serial": None}
        patterns = {
            "manufacturer": r"Product Manufacturer\s*:\s*(.*)",
            "model": r"Product Name\s*:\s*(.*)",
            "board_mfg": r"Board Mfg\s*:\s*(.*)",
            "board_product": r"Board Product\s*:\s*(.*)",
        }

        m_manuf = re.search(patterns["manufacturer"], fru_data, re.IGNORECASE)
        m_model = re.search(patterns["model"], fru_data, re.IGNORECASE)

        model_info["manufacturer"] = m_manuf.group(1).strip() if m_manuf else "Unknown"
        model_info["model"] = m_model.group(1).strip() if m_model else "Unknown"

        if model_info["manufacturer"] == "Unknown":
            m_board_mfg = re.search(patterns["board_mfg"], fru_data, re.IGNORECASE)
            if m_board_mfg:
                model_info["manufacturer"] = m_board_mfg.group(1).strip()

        if model_info["model"] == "Unknown":
            m_board_prod = re.search(patterns["board_product"], fru_data, re.IGNORECASE)
            if m_board_prod:
                model_info["model"] = m_board_prod.group(1).strip()

        for serial_field in (r"Product Serial\s*:\s*(.*)", r"Chassis Serial\s*:\s*(.*)",
                             r"Board Serial\s*:\s*(.*)"):
            m_serial = re.search(serial_field, fru_data, re.IGNORECASE)
            if m_serial and m_serial.group(1).strip():
                model_info["serial"] = m_serial.group(1).strip()
                break

        if "dell" in model_info["manufacturer"].lower():
            model_info["manufacturer"] = "DELL"

        self._log("info", f"Server Info: Manufacturer='{model_info['manufacturer']}', "
                           f"Model='{model_info['model']}', Serial={model_info['serial']}")
        return model_info

    def retrieve_temperatures_raw(self):
        self._log("debug", "Retrieving raw temperature SDR data...")
        return self._run_ipmi_command(["sdr", "type", "temperature"], is_raw_command=False)

    def parse_temperatures(self, sdr_data, cpu_pattern_str=None, inlet_pattern_str=r"^Inlet Temp\b",
                           exhaust_pattern_str=r"^Exhaust Temp\b"):
        """Parse `ipmitool sdr type temperature` output.

        CPU sensors are identified by processor entity ID (3.x), which covers
        both "CPU1 Temp" (13G) and "Temp" (12G). `cpu_pattern_str` remains as
        an optional extra name filter for unusual firmware.
        """
        temps = {"cpu_temps": [], "inlet_temp": None, "exhaust_temp": None}
        if not sdr_data:
            return temps

        for line in sdr_data.splitlines():
            match = _TEMP_LINE_RE.match(line.strip())
            if not match:
                continue
            try:
                temp_value = int(round(float(match.group("value"))))
            except ValueError:
                continue
            if not -20 <= temp_value <= 125:
                self._log("debug", f"Ignoring implausible temperature {temp_value} C from {line.strip()}")
                continue

            sensor_name = match.group("name").strip()
            entity = match.group("entity")

            if inlet_pattern_str and re.search(inlet_pattern_str, sensor_name, re.IGNORECASE):
                if temps["inlet_temp"] is None:
                    temps["inlet_temp"] = temp_value
                continue
            if exhaust_pattern_str and re.search(exhaust_pattern_str, sensor_name, re.IGNORECASE):
                if temps["exhaust_temp"] is None:
                    temps["exhaust_temp"] = temp_value
                continue

            is_cpu = entity.startswith("3.") or bool(_CPU_NAME_RE.search(sensor_name))
            if cpu_pattern_str:
                is_cpu = is_cpu or bool(re.search(cpu_pattern_str, sensor_name, re.IGNORECASE))
            if is_cpu:
                temps["cpu_temps"].append(temp_value)

        return temps

    def retrieve_fan_rpms_raw(self):
        self._log("debug", "Retrieving raw fan SDR data...")
        return self._run_ipmi_command(["sdr", "type", "fan"], is_raw_command=False, timeout=10)

    def parse_fan_rpms(self, sdr_data):
        fans = []
        if not sdr_data:
            return fans

        for line in sdr_data.splitlines():
            match = _FAN_LINE_RE.match(line.strip())
            if not match:
                continue
            try:
                fans.append({"name": match.group("name").strip(),
                             "rpm": int(float(match.group("value")))})
            except ValueError:
                continue
        return fans

    def retrieve_power_sdr_raw(self):
        self._log("debug", "Retrieving raw power SDR data...")
        return self._run_ipmi_command(["sdr", "elist"], is_raw_command=False, timeout=20)

    def parse_power_consumption(self, sdr_data):
        if not sdr_data:
            return None

        for line in sdr_data.splitlines():
            match = _POWER_RE.search(line.strip())
            if match:
                try:
                    return int(float(match.group("value")))
                except (ValueError, IndexError):
                    continue
        return None

    def get_power_status(self, sdr_data):
        """Return [{"name": "PSU1", "ok": bool}] based on presence and PG fail.

        The voltage heuristic used previously is gone: 13G iDRACs expose
        "PS1 Status"/"PS1 PG Fail" but not a sensor named "Voltage 1", which
        made healthy supplies look failed.
        """
        if not sdr_data:
            self._log("warning", "SDR data is empty for power status parsing.")
            return []

        psus = {}
        for line in sdr_data.splitlines():
            line = line.strip()
            match = _PSU_STATUS_RE.match(line)
            if match:
                index = match.group("index")
                reading = match.group("reading").lower()
                psus.setdefault(index, {})["present"] = "presence detected" in reading
                continue

            match = _PSU_FAIL_RE.match(line)
            if match:
                index = match.group("index")
                status = match.group("status").lower()
                psus.setdefault(index, {})["fault"] = status not in ("ok", "nr", "ns")
                continue

        final_status = []
        for index, data in sorted(psus.items()):
            if not data.get("present", False):
                # Empty PSU bay: no entity, instead of a permanent problem sensor.
                continue
            name = f"PSU{index}"
            is_ok = not data.get("fault", False)
            final_status.append({"name": name, "ok": is_ok})
            self._log("debug", f"PSU Status for {name}: Fault={data.get('fault', False)} -> Final OK={is_ok}")

        return final_status

    def chassis_shutdown(self):
        """Sends a graceful ACPI shutdown command to the server."""
        self._log("info", "Sending graceful shutdown command to server...")
        return self._run_ipmi_command(["chassis", "power", "soft"], is_raw_command=False)
