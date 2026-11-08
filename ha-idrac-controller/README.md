# Home Assistant iDRAC Controller Add-on

**Control your Dell PowerEdge server's fan speeds based on CPU temperature and monitor key server metrics directly from Home Assistant.**

This add-on connects to your server's iDRAC interface using IPMI to:
* Read temperatures (CPU, Inlet, Exhaust).
* Read actual fan speeds (RPM).
* Read power consumption (Watts).
* Control fan speeds based on configurable temperature thresholds.
* Publish all data to MQTT for integration with Home Assistant, creating sensors automatically via MQTT Discovery.
* Provide a Web UI (via Ingress) for status overview and advanced configuration (like a multi-point fan curve, though primary logic uses simpler thresholds from HA config for now).

## Features

* **Dynamic Fan Control:** Adjusts fan speeds based on the hottest CPU core temperature using a 3-tier threshold system (Base, High, Critical).
* **Server Monitoring:** Creates Home Assistant sensors for:
    * Individual CPU Temperatures
    * Hottest CPU Temperature
    * Inlet Temperature
    * Exhaust Temperature
    * Individual Fan Speeds (RPM)
    * Power Consumption (Watts)
    * Target Fan Speed Percentage
    * Add-on Connectivity Status
* **MQTT Auto-Discovery:** Automatically creates and configures entities in Home Assistant.
* **Web UI via Ingress:**
    * View live server status (temperatures, fan speeds, power).
    * View current fan control settings.
    * (Future/Optional) Configure an advanced multi-point fan curve.
* **Configurable:** Set iDRAC credentials, fan thresholds, temperature units, and MQTT details via the Home Assistant add-on configuration panel.

## Prerequisites

1.  **Dell PowerEdge Server with iDRAC:** This add-on uses IPMI, which is available on most Dell PowerEdge servers with an iDRAC (iDRAC 7, 8, 9 and newer should generally work). This add-on has been tested with an R720 (iDRAC7).
2.  **Network Connectivity:** Your Home Assistant instance must be able to reach your server's iDRAC IP address over the network.
3.  **IPMI over LAN Enabled in iDRAC:** This is crucial.

    **How to Enable IPMI over LAN in iDRAC (General Steps):**

    The exact steps can vary slightly depending on your iDRAC version (iDRAC7, iDRAC8, iDRAC9, etc.).

    * **Access iDRAC Web Interface:** Open a web browser and navigate to your iDRAC's IP address. Log in with your iDRAC credentials.
    * **Navigate to Network Settings:**
        * **iDRAC 7/8:** Look for "iDRAC Settings" -> "Network" or "Connectivity".
        * **iDRAC 9:** Look for "iDRAC Settings" -> "Connectivity" -> "Network Settings".
    * **Find IPMI Settings:** Within the network settings, look for a section specifically labeled "IPMI Settings" or "IPMI over LAN."
        * Common path: (iDRAC Settings -> Network -> IPMI Settings link at the top or bottom of the page).
    * **Enable IPMI over LAN:**
        * There will be a checkbox or toggle: "Enable IPMI Over LAN". Ensure this is **checked/enabled**.
        * **Channel Privilege Level Limit:** Set this to "ADMINISTRATOR" or "OPERATOR" to allow fan control and sensor reading. Administrator is usually required for control.
        * **Encryption Key:** Leave as `0000000000000000000000000000000000000000` (40 zeros) unless you have specific IPMI 2.0 encryption needs (this add-on uses standard IPMItool commands that work with this default).
    * **Apply Settings:** Save and apply the changes. The iDRAC might restart its network interface.

    *If you cannot find these settings, consult your specific Dell PowerEdge server's iDRAC manual for "IPMI over LAN" configuration.*

4.  **MQTT Broker:** You need an MQTT broker accessible by Home Assistant. The `core-mosquitto` Home Assistant add-on is recommended.

## Installation

1.  **Add the Repository to Home Assistant:**
    * In Home Assistant, go to **Settings > Add-ons**.
    * Click the **"ADD-ON STORE"** button (usually bottom right).
    * Click the **three-dots menu** (⋮) in the top right corner and select **"Repositories"**.
    * In the "Manage add-on repositories" dialog, paste the following URL:
        ```
        [https://github.com/Aesgarth/HA-iDRAC](https://github.com/Aesgarth/HA-iDRAC)
        ```
    * Click **"ADD"** and then **"CLOSE"**.

2.  **Install the Add-on:**
    * Refresh your browser page if needed.
    * Your "HA iDRAC Controller" add-on should now appear in the Add-on Store (you might find it at the bottom under a section like "Aesgarth's Custom iDRAC Add-on" or in the main list).
    * Click on the "HA iDRAC Controller" add-on.
    * Click **"INSTALL"**. This will download and build the add-on, which may take a few minutes.

## Configuration

After installation, you **must** configure the add-on before starting it.

1.  Go to the add-on page (Settings > Add-ons > HA iDRAC Controller).
2.  Switch to the **"Configuration"** tab.
3.  Fill in the following options:

    * **iDRAC Connection:**
        * `idrac_ip`: (Required) The IP address or hostname of your server's iDRAC.
        * `idrac_username`: (Required) Your iDRAC username (default: `root`).
        * `idrac_password`: (Required) Your iDRAC password.
    * **Fan Control Logic (Simple Mode):**
        * `temperature_unit`: `C` (Celsius) or `F` (Fahrenheit). Temperatures from iDRAC are read in Celsius; if you set this to 'F', thresholds you enter will be assumed to be in Fahrenheit and converted internally. Display in the web UI will also respect this.
        * `base_fan_speed_percent`: (Default: `20`) The fan speed (0-100%) to use when the hottest CPU core is below the `low_temp_threshold`.
        * `low_temp_threshold`: (Default: `45`) The CPU temperature (in your selected `temperature_unit`) above which fans will switch from `base_fan_speed_percent` to `high_temp_fan_speed_percent`.
        * `high_temp_fan_speed_percent`: (Default: `50`) The fan speed (0-100%) to use when the hottest CPU core is at or above `low_temp_threshold` but below `critical_temp_threshold`.
        * `critical_temp_threshold`: (Default: `65`) The CPU temperature (in your selected `temperature_unit`) at or above which fan control will be handed back to the iDRAC (Dell's automatic mode) for safety.
    * **Polling and Logging:**
        * `check_interval_seconds`: (Default: `30`) How often (in seconds) to check temperatures and adjust fans.
        * `log_level`: (Default: `info`) Set the verbosity of logs. Options: `trace`, `debug`, `info`, `notice`, `warning`, `error`, `fatal`. Use `debug` or `trace` for troubleshooting.
    * **MQTT Configuration:**
        * `mqtt_host`: (Default: `core-mosquitto`) Hostname or IP address of your MQTT broker.
        * `mqtt_port`: (Default: `1883`) Port for your MQTT broker.
        * `mqtt_username`: (Optional) Username for MQTT broker authentication. Leave blank for anonymous access if your broker allows it (common for `core-mosquitto` from other add-ons).
        * `mqtt_password`: (Optional) Password for MQTT broker authentication.

4.  Click **"SAVE"**.

5.  Go to the **"Info"** tab and click **"START"**.

## Web UI (Ingress Panel)

Once the add-on is started, you can access its web UI:
* Click on "iDRAC Control" in your Home Assistant sidebar.
* Or, go to the add-on's "Info" tab and click "OPEN WEB UI".

The Web UI currently provides:
* A status overview showing live temperatures, fan RPMs, power consumption, and the current target fan speed.
* Displays the "Simple Fan Mode" settings currently active from your HA add-on configuration.
* A link to a settings page for an "Advanced Fan Curve" (note: the main control logic currently uses the "Simple Fan Mode" settings from the HA configuration tab; the advanced curve is for future use or if you modify the Python script to prioritize it).

## Sensors Created in Home Assistant (via MQTT)

If MQTT is configured correctly, the following entities will be automatically discovered and created under a device representing your iDRAC:

* **Connectivity:**
    * `binary_sensor.idrac_controller_connectivity`: `online`/`offline` status of the add-on.
* **Temperatures (°C):**
    * `sensor.idrac_cpu_0_temperature` (and `_1`, `_2`, `_3` etc., based on detected CPUs)
    * `sensor.idrac_hottest_cpu_temp`
    * `sensor.idrac_inlet_temperature`
    * `sensor.idrac_exhaust_temperature`
* **Fans:**
    * `sensor.idrac_target_fan_speed`: Current target fan speed percentage (or "Auto").
    * `sensor.idrac_fan_X_rpm`: Actual speed for each detected fan (e.g., `sensor.idrac_fan_fan1a_tach_rpm`).
* **Power:**
    * `sensor.idrac_power_consumption`: Current server power usage in Watts.

*(Sensor entity IDs might vary slightly based on your iDRAC IP and sensor names).*

## Troubleshooting

* **No Output in Add-on Log:** If the add-on starts but the log is empty after `legacy-services successfully started`, there might be a fundamental issue with the s6-overlay execution or output redirection in the base image. This was an earlier debugging step; the current version using `python:3.11-slim-bookworm` should show logs.
* **"Manifest Unknown" or Docker Build Errors:** Ensure your Home Assistant OS and Supervisor are up to date. If issues persist with pulling base images, check your HA host's network and DNS settings. Sometimes specific tags for HA base images can be problematic; using standard Docker Hub images like `python:3.11-slim-bookworm` (as currently configured) is often more stable.
* **IPMI Communication Errors:**
    * Verify "IPMI over LAN" is enabled in your iDRAC settings (see Prerequisites).
    * Double-check the iDRAC IP, username, and password in the add-on configuration.
    * Ensure no firewall is blocking UDP port 623 (standard IPMI port) between Home Assistant and your iDRAC.
* **MQTT "Not Authorized" Errors:**
    * Ensure the MQTT username and password configured in the add-on match a valid user in your MQTT broker (e.g., Mosquitto).
    * If using the `core-mosquitto` add-on, you may need to create a new Home Assistant user and configure Mosquitto to use HA users for authentication, then use those credentials in this add-on.
* **Incorrect Temperature/Fan Parsing:**
    * The add-on tries to parse sensor data from `ipmitool`. If values are incorrect or missing, the regex patterns in `app/ipmi_manager.py` or `app/main.py` might need adjustment for your specific server model or iDRAC firmware version's output.
    * Set Log Level to `debug` or `trace` in the add-on configuration to see detailed parsing attempts.
* **To get detailed logs:** Set the `log_level` to `debug` or `trace` in the add-on's Configuration tab and restart the add-on. View logs in the "Log" tab.

## Contributing / Reporting Issues

Please open an issue on the [GitHub repository](https://github.com/Aesgarth/HA-iDRAC/issues) for any bugs, feature requests, or questions.

## License

This project uses the [MIT License](LICENSE).
