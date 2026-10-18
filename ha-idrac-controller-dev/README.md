# Home Assistant iDRAC Controller Add-on (Development Version)

**<font color="red">⚠️ DEVELOPMENT VERSION - USE WITH CAUTION! ⚠️</font>**

**This is a work-in-progress, development version of the HA iDRAC Controller add-on. It is intended for testing and feedback. Unexpected behavior or bugs are possible. Use this version at your own risk, as incorrect fan configuration could potentially lead to server overheating if not carefully monitored.**

---

**Control multiple Dell PowerEdge servers' fan speeds and monitor key server metrics directly from a single instance in Home Assistant.**

This add-on connects to your servers' iDRAC interfaces using IPMI to:
* Read key metrics like temperatures (CPU, Inlet, Exhaust), fan speeds (RPM), and power consumption (Watts).
* Monitor the status of individual Power Supply Units (PSUs) to detect power loss or hardware failure.
* Provide advanced, per-server fan control with multiple modes.
* Publish all data to MQTT for seamless integration with Home Assistant, creating sensors automatically via MQTT Discovery.
* Provide a comprehensive Web UI (via Ingress) for a live status dashboard and full configuration of all connected servers.

## Features

* **Multi-Server Support:** Monitor and control all your Dell PowerEdge servers from a single add-on instance.
* **Advanced Fan Control:** Choose your preferred fan management mode on a per-server basis:
    * **Simple Thresholds:** A 3-tier system (Base, High, Critical) for straightforward fan management.
    * **Multi-Point Curve:** Define a custom temperature-to-fan-speed curve for smooth and granular control.
    * **Target Temperature:** Set a desired CPU temperature, and the add-on will automatically adjust fan speeds to maintain it.
* **Comprehensive Server Monitoring:** Creates a dedicated device in Home Assistant for each server with sensors for:
    * Individual CPU Temperatures
    * Hottest CPU Temperature
    * Inlet & Exhaust Temperatures
    * Individual Fan Speeds (RPM)
    * Power Consumption (Watts)
    * Target Fan Speed Percentage
    * PSU Status (OK/Problem)
    * Server Connectivity (Online/Offline)
* **Remote Actions:**
    * **Graceful Shutdown:** A "Shutdown Server" button is created for each server in Home Assistant.
* **Web UI via Ingress:**
    * View a live dashboard of all monitored servers.
    * A dedicated "Manage Servers" page to add, edit, and delete servers and configure their fan control settings.
* **MQTT Auto-Discovery:** Automatically creates and configures all entities in Home Assistant.

## Changes in this fork (0.1.0-dev.15)

* CPU temperature parsing uses the IPMI processor entity (`3.x`), so both
  `CPU1 Temp` (13G/iDRAC 8) and `Temp` (12G/iDRAC 7) are found.
* Inlet/exhaust matching is anchored to the start of the sensor name, so
  `PSU1 Inlet Temp` cannot replace the chassis inlet reading.
* 2 °C hysteresis on the base/high and critical thresholds; speeds below 5 %
  are clamped to 5 %.
* If the manual-mode command lands but the speed command fails, the fans are
  handed back to Dell auto instead of being pinned at the previous speed.
* A server that refuses fan control (0xc1/0xd4/0xd5 — iDRAC 9/10 or an account
  below Administrator) is detected once, fan control is disabled and
  monitoring continues.
* Optional per-server **Third-party PCIe cooling** setting (12G/13G OEM
  command); restored on shutdown.
* MQTT device gets `sw_version` (iDRAC firmware) and `serial_number` (service
  tag) from the server.
* Contradictory thresholds are corrected with a warning; implausible
  temperatures outside -20..125 °C are ignored.
* MQTT outages no longer stop a worker; passwords are passed with `ipmitool -E`;
  the Supervisor watchdog is enabled.

---

## <font color="orange">⚠️ Important Note for Testers ⚠️</font>
* This version is for active development. Please report any issues or bugs you encounter.
* **Closely monitor your server's temperatures after configuring and enabling fan control.**
* The developer is not responsible for any damage arising from the use of this development software.

## Prerequisites

1.  **Dell PowerEdge Server with iDRAC 7 or 8:** The add-on uses IPMI. 12th-generation (iDRAC 7, e.g. R720) and 13th-generation (iDRAC 8, e.g. **T430**, R630, R730) sensor layouts are supported and covered by unit tests. The original add-on was hardware-tested on an R720; 13G hardware verification is still pending, so check the log output on first start.
    * For a T430, `ipmitool sdr type temperature` should show `CPU1 Temp`/`CPU2 Temp` on entities `3.1`/`3.2` plus `Inlet Temp`/`Exhaust Temp` on `7.x`.
2.  **Network Connectivity:** Your Home Assistant instance must be able to reach each server's iDRAC IP address.
3.  **IPMI over LAN Enabled in iDRAC:** This is crucial for the add-on to function.
    * Log in to your iDRAC's web interface.
    * Navigate to **iDRAC Settings** -> **Network** (or **Connectivity**).
    * Find the **IPMI Settings** section.
    * Ensure **Enable IPMI Over LAN** is checked.
    * Set the **Channel Privilege Level Limit** to **Administrator**.
    * Save the settings.
4.  **MQTT Broker:** You need an MQTT broker accessible by Home Assistant. The `core-mosquitto` add-on is recommended.

## Installation

1.  **Add the Repository to Home Assistant:**
    * In Home Assistant, go to **Settings > Add-ons**.
    * Click the **ADD-ON STORE** button.
    * Click the **three-dots menu** (⋮) in the top right and select **Repositories**.
    * Paste the following URL and click **ADD**:
        ```
        https://github.com/sam2kb/HA-iDRAC
        ```
2.  **Install the Add-on:**
    * Refresh the page. You should now see the "HA iDRAC Controller BETA" add-on in the store.
    * Click on it and then click **INSTALL**.

## Configuration

Configuration is now handled almost entirely through the add-on's Web UI.

1.  **Initial Add-on Setup:**
    * Go to the add-on page (**Settings > Add-ons > HA iDRAC Controller BETA**).
    * Switch to the **Configuration** tab.
    * Fill in your **MQTT Broker** details.
    * The fan speed and temperature thresholds on this page act as **global defaults** for newly added servers.
    * Click **SAVE**.

2.  **Adding Servers:**
    * Go to the **Info** tab and **START** the add-on.
    * Click **OPEN WEB UI**.
    * Click the **Manage Servers** link.
    * Use the "Add New Server" form to add your first server. The form will be pre-filled with the global defaults you just set.
    * After adding or editing servers, a link will appear prompting you to restart the add-on. You **must restart the add-on** for your changes to take effect.

## Web UI (Ingress Panel)

The Web UI is the primary interface for this add-on:
* **Dashboard:** Shows a live status overview for every enabled server. The page auto-refreshes.
* **Manage Servers Page:** Allows you to add, edit, or delete your server configurations. When editing a server, you can select the desired fan control mode and configure its specific parameters.

## Entities Created in Home Assistant

For each server, the add-on will create a new device in Home Assistant with the following entities:
* **Controls:**
    * `button.idrac_server_alias_shutdown_server`
* **Sensors:**
    * `binary_sensor.idrac_server_alias_status` (Online/Offline)
    * `binary_sensor.idrac_server_alias_psu_status` for each power supply.
    * Numerous sensors for temperatures, fan speeds, and power usage.

*(Entity IDs will be based on the unique alias you give each server).*

## Troubleshooting

* **Check the Add-on Log:** The first place to look for errors is the "Log" tab of the add-on. Set the "Log Level" to `debug` or `trace` in the Configuration tab for more detail.
* **IPMI Errors:** Verify "IPMI over LAN" is enabled and that all credentials are correct for each server in the Web UI.
* **MQTT Errors:** Check your MQTT credentials in the add-on's Configuration tab.
* **Incorrect Sensor Data:** The regex patterns for parsing sensor data in `app/ipmi_manager.py` may need to be adjusted for your specific server model if you see incorrect or missing values.

## Contributing / Reporting Issues

This is a development version. Please report any bugs, issues, or feature suggestions by opening an issue on the [GitHub repository](https://github.com/sam2kb/HA-iDRAC/issues). Please provide logs and details about your server model if you encounter problems.

## License

This project uses the [MIT License](LICENSE).
