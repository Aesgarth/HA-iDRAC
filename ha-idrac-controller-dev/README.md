# Home Assistant iDRAC Controller Add-on (Development Version)

**<font color="red">⚠️ DEVELOPMENT VERSION - USE WITH CAUTION! ⚠️</font>**
**<font color="red">⚠️ Known issue: When first started and learning, PID mode can and probably will ramp fans up to near their highest configured speed. There is a very good chance your server will be LOUD ⚠️</font>**

**This is a work-in-progress, development version of the HA iDRAC Controller add-on. It is intended for testing and feedback. Unexpected behavior or bugs are possible. Use this version at your own risk, as incorrect fan configuration could potentially lead to server overheating if not carefully monitored.**

---

**Control multiple Dell PowerEdge servers' fan speeds and monitor key server metrics directly from a single instance in Home Assistant.**

## Features

* **Multi-Server Support:** Monitor and control multiple Dell PowerEdge servers from a single add-on instance.
* **Three Fan Control Modes (per server):**
  * **Simple Thresholds:** 3-tier control (Base, High, Critical) with 2 °C hysteresis.
  * **Multi-Point Curve:** Custom temperature-to-fan-speed curve.
  * **Target Temperature:** Self-learning controller that maintains a target CPU temperature while keeping fans quiet and steady.
* **Fan Failure Detection:** Detects stopped, missing, or slow fans from RPM readings using a learned per-fan baseline, and applies the configured **On fan fault** response (`Dell Auto` by default, `Raise working fans to 80%+`, or `Alert only`; `Dell Auto` latches until all fans are healthy for 10 minutes).
* **Disk Health Monitoring (SNMP):** Automatically detects all physical disks via `IDRAC-MIB::physicalDiskTable` and includes a staleness safety net so failed polls or a hung poller never silently hide a problem or keep reporting stale `"OK"` states.
* **Safety Hand-Back:** Automatically reverts fans to Dell auto on add-on shutdown, after 3 consecutive failed temperature reads from iDRAC (retrying every 60 s), and via an in-container watchdog process if the main controller stops updating its heartbeat.
* **Live Settings & Production Web Server:** All per-server UI changes validate, save atomically, and take effect immediately without restarting the add-on. Served via `waitress` with quiet access logging.
* **MQTT Auto-Discovery & Diagnostics:** Publishes Home Assistant entities with `sw_version` (iDRAC firmware) and `serial_number` (service tag), plus a **Shutdown Server** button.

## What's New

* **Self-learning Target Temperature controller (PID v3):** Time-based smoothing, scaled emergency rule, learned fan-power compensation, and configurable ramp-down rate.
* **Fan failure detection:** Compares each fan against the median of all fans relative to its learned baseline and executes your chosen fault policy.
* **SNMP disk health & staleness safety net:** Autodetects any number of disks, retains states across restarts, and marks healthy sensors unavailable if polling goes stale while keeping active problems latched `ON`.
* **In-container watchdog & unreachable-iDRAC hand-back:** Hands fans back to Dell auto if iDRAC stops responding or if the controller hangs, plus an `iDRAC reachable` connectivity sensor and a configurable `Minimum safe fan speed` floor.
* **Live UI settings:** Adding, editing, or deleting servers in the Web UI applies immediately without restarting the add-on.
* **Consolidated MQTT entities:** Merged `Fan status` into `Fan fault` attributes and `Disks summary` into `Disks problem` attributes, with automatic cleanup of old entities on startup.

---

## How the Target Temperature Mode Works

Target Temperature mode learns how much cooling your specific server needs instead of relying on fixed manual guesses:

* **Learning & persistence:** The controller collects steady-load samples over a 150 s window (when CPU temperature, fan speed, inlet temperature, and power are all steady and power is at least 20 W) to fit the server's cooling curve (`theta`) and fan power draw (`power_floor`). In the first hours before enough samples are gathered, it starts conservatively and may run fans higher than needed. Learned state is saved to `/data/pid_state.json` and survives restarts and live settings edits.
* **Fan power compensation:** Because total system power includes the fans themselves (which rises steeply with speed), the controller learns each speed band's baseline power floor and subtracts estimated fan power from total power so spinning up the fans does not look like increased CPU load.
* **Key parameters:**
  * **Setpoint (`Target Temperature`):** Desired CPU temperature (°C).
  * **Min / Max Fan Speed (%):** Hard bounds for PID output (also bounded below by `Minimum safe fan speed`).
  * **Deadband (°C):** Temperature band around the setpoint where integral trim pauses to prevent hunting.
  * **Ramp Down Rate (% / min):** Limits how quickly fans slow down after a temperature spike (doubled automatically when overcooled).
  * **Critical Temp Threshold (°C):** Above this threshold (with 2 °C hysteresis), fans return to **Dell Auto**.
* **Emergency rule:** If CPU temperature reaches the scaled emergency limit (halfway between setpoint and Critical temperature, or `setpoint + 8 °C` minimum), 2 consecutive hot polls jump fans to `Max Fan Speed`. If CPU temperature comes within 5 °C of Critical, it jumps immediately on the first poll.
* **Poll interval:** A `10 s` poll interval is recommended for Target Temperature mode; intervals above `30 s` log a warning because slower polling reduces learning accuracy and spike response.

---

## <font color="orange">⚠️ Important Note for Testers ⚠️</font>
* This version is for active development. Please report any issues or bugs you encounter.
* **Closely monitor your server's temperatures after configuring and enabling fan control.**
* The developer is not responsible for any damage arising from the use of this development software.

## Prerequisites

1. **Dell PowerEdge Server with iDRAC 7 or 8:** The add-on uses IPMI. 12th-generation (iDRAC 7, e.g. R720) and 13th-generation (iDRAC 8, e.g. **T430**, R630, R730) sensor layouts are supported and covered by unit tests.
2. **Network Connectivity:** Your Home Assistant instance must be able to reach each server's iDRAC IP address (UDP 623 for IPMI, and UDP 161 if SNMP disk monitoring is enabled).
3. **IPMI over LAN Enabled in iDRAC:**
   * Log in to your iDRAC web interface.
   * Navigate to **iDRAC Settings** > **Network** (or **Connectivity**) > **IPMI Settings**.
   * Check **Enable IPMI Over LAN** and set **Channel Privilege Level Limit** to **Administrator**.
   * Save the settings.
4. **MQTT Broker:** An MQTT broker accessible by Home Assistant (the `core-mosquitto` add-on is recommended).

## Installation

1. In Home Assistant, go to **Settings > Add-ons > Add-on Store**.
2. Open the three-dots menu (**⋮**) in the top right and select **Repositories**.
3. Paste `https://github.com/Aesgarth/HA-iDRAC` and click **ADD**.
4. Refresh the store, select **HA iDRAC Controller BETA**, and click **INSTALL**.

## Configuration

1. **Initial Add-on Setup (HA Configuration tab):**
   * Go to **Settings > Add-ons > HA iDRAC Controller BETA > Configuration**.
   * Enter your **MQTT Broker** details (`mqtt_host`, `mqtt_port`, `mqtt_username`, `mqtt_password`), `log_level`, and default fan thresholds.
   * Click **SAVE**. *(Changes on the Home Assistant Configuration tab require an add-on restart because Supervisor passes them in at container start.)*
2. **Adding and Editing Servers (Web UI):**
   * Start the add-on and click **OPEN WEB UI** > **Manage Servers**.
   * Add or edit your servers. All changes made in the Web UI are validated, saved atomically, and **applied live with no add-on restart** (`Saved and applied`).

## Web UI (Ingress Panel)

* **Dashboard:** Shows a live auto-refreshing overview of temperatures, fan RPMs, power draw, PSU statuses, fan health, PID status, and physical disks for every enabled server.
* **Manage Servers:** Add, edit, or delete servers, switch fan control modes, and test SNMP disk polling without restarting the add-on.

---

## Safety and Limits

* **Normal shutdown:** On `SIGTERM`/`SIGINT`, every worker hands its server back to Dell auto within a 50 s shutdown deadline before the container exits.
* **iDRAC unreachable:** After 3 consecutive failed temperature SDR reads, the controller marks `iDRAC reachable` as `OFF`, logs a warning, and attempts to send the Dell auto command every 60 s. When iDRAC reconnects, Dell auto is sent first and PID slew state is reset before resuming control.
* **Controller hang (watchdog):** A separate `watchdog.py` process runs inside the container. Every poll cycle (and at least every 60 s while sleeping), the controller writes a heartbeat listing servers in manual fan control. If the heartbeat goes stale for 180 s, the watchdog hands those servers back to Dell auto every 30 s until it succeeds.
* **Fan failure:** `FanMonitor` ignores the first 60 s after a speed change, requires a fault to persist for ~90 s, and clears after 5 healthy polls (with fewer than 3 fans, only stopped/missing fans are flagged). Under the default `Dell Auto` policy, a fault immediately hands fans back to Dell auto and latches for 10 minutes after recovery.
* **Honest limit — if the Home Assistant host dies:** Nothing running on the host can restore Dell auto if the entire machine loses power or crashes, and **Dell IPMI manual fan mode has no built-in timeout**, so fans stay at their last commanded speed until iDRAC's own thermal protection overrides them. Set **Minimum safe fan speed** high enough to keep your server safe under worst-case load if the host goes down.

---

## Settings Reference

### Per-Server Settings (Web UI — all apply live without restarting)

| Setting | Meaning | Default | Applies Live |
|---|---|---|---|
| `Alias` | Unique identifier for the server and its MQTT entities | *(required)* | Add / Delete only |
| `iDRAC IP / Hostname` | IP address or hostname of the server's iDRAC | *(required)* | Yes (restarts worker, keeps PID model) |
| `iDRAC Username` | IPMI administrator username | `root` | Yes (restarts worker, keeps PID model) |
| `iDRAC Password` | IPMI password (blank on edit keeps existing) | `""` | Yes (restarts worker, keeps PID model) |
| `Enabled` | Enable or pause monitoring and control for this server | `true` | Yes |
| `Fan Control Enabled` | Allow manual fan commands (`false` = monitor only) | `true` | Yes |
| `Disable 3rd-Party PCIe Cooling` | Send Dell OEM command to disable non-Dell PCIe fan boost | `false` | Yes |
| `On fan fault` | Fault policy: `dell_auto`, `raise_fans` (80%+), or `alert_only` | `dell_auto` | Yes |
| `Minimum safe fan speed (%)` | Hard floor (5–100%) applied across all modes; warns if `< 15%` | `20` | Yes |
| `Poll Interval (seconds)` | Per-server poll interval (min `5 s`; blank uses global) | Global | Yes |
| `Fan Mode` | `simple`, `curve`, or `target` (PID) | `simple` | Yes |
| `Base Fan Speed (%)` | Simple mode low-temperature speed (5–100%) | `20` | Yes |
| `Low Temp Threshold (°C)` | Simple mode threshold to switch from Base to High | `45` | Yes |
| `High Temp Fan Speed (%)` | Simple mode high-temperature speed (5–100%) | `50` | Yes |
| `Critical Temp Threshold (°C)` | Temperature to hand fans back to Dell Auto (all modes) | `65` | Yes |
| `Target Temp (°C)` | PID setpoint temperature (20–90 °C) | `50` | Yes |
| `Min / Max Fan Speed (%)` | PID speed bounds (5–100%, `min <= max`) | `15` / `95` | Yes |
| `Deadband (°C)` | PID integral pause band around setpoint | `0.75` | Yes |
| `Integral Time (Ti, s)` | PID trim integral time constant | `300.0` | Yes |
| `Ramp Down Rate (% / min)` | Max PID fan ramp-down rate (1–50 %/min) | `4` | Yes |
| `Fan Curve Points` | `(temp, speed)` pairs for `curve` mode | `[]` | Yes |
| `Disk Health Monitoring (SNMP)` | Enable SNMP physical disk polling | `false` | Yes |
| `SNMP Community String` | SNMPv2c community string (blank on edit keeps existing) | `""` | Yes |
| `SNMP Port` | SNMP UDP port | `161` | Yes |
| `Disk Poll Interval (minutes)` | Minutes between SNMP walks (1–1440) | `10` | Yes |

### Global Options (Home Assistant Configuration Tab — requires add-on restart)

| Option | Meaning | Default |
|---|---|---|
| `mqtt_host`, `mqtt_port`, `mqtt_username`, `mqtt_password` | MQTT broker connection details | `core-mosquitto`, `1883`, `""`, `""` |
| `check_interval_seconds` | Default poll interval when a server leaves it blank | `30` |
| `log_level` | Logging verbosity (`trace`, `debug`, `info`, `notice`, `warning`, `error`, `fatal`) | `info` |
| `base_fan_speed_percent`, `low_temp_threshold`, `high_temp_fan_speed_percent`, `critical_temp_threshold` | Global defaults pre-filled when adding a new server | `20`, `45`, `50`, `65` |

---

## Entities Created in Home Assistant

For each enabled server (`<server>` is the sanitized server alias):

* **Controls:**
  * `button.idrac_<server>_shutdown_server` — Triggers a graceful IPMI chassis shutdown.
* **Connectivity & Health Binary Sensors:**
  * `binary_sensor.idrac_<server>_idrac_reachable` (`connectivity`) — `ON` when iDRAC responds to SDR reads, `OFF` after 3 consecutive failures.
  * `binary_sensor.idrac_<server>_fan_fault` (`problem`) — `ON` on fan fault, `OFF` when healthy. Attributes: `status` (e.g. `OK` or `Fan4: stopped`), `faulty_fans`, and per-fan RPM readings.
  * `binary_sensor.idrac_<server>_psu_<name>` (`problem`) — One per PSU; `ON` on power/presence fault.
* **Temperature, Fan & Power Sensors:**
  * `sensor.idrac_<server>_hottest_cpu_temp` (°C), `sensor.idrac_<server>_cpu_<i>_temp` (°C)
  * `sensor.idrac_<server>_inlet_temp` (°C), `sensor.idrac_<server>_exhaust_temp` (°C)
  * `sensor.idrac_<server>_fan_<name>_rpm` (RPM) — One per detected tachometer.
  * `sensor.idrac_<server>_power` (W)
  * `sensor.idrac_<server>_target_fan_speed` (%)
* **Disk Health Entities (when SNMP is enabled):**
  * `binary_sensor.idrac_<server>_disks_problem` (`problem`) — `ON` if any disk has a problem. Attributes: `summary` (`All N disks OK` or `<k> problem(s)`), `count`, `problem_count`, `problems` (plus `stale` and `last_success` when stale). Goes `unavailable` when stale and last known healthy.
  * `sensor.idrac_<server>_disk_<bay>_health` — Per-disk state (`OK`, `Warning`, `Predicted failure`, `Critical`, `Failed`, or `Offline`) with hardware attributes (`model`, `serial`, `firmware`, `size_gb`, `media_type`, `raid_state`, `component_status`, `smart_alert`, `manufactured`, `endurance_remaining`, `slot`).
  * `binary_sensor.idrac_<server>_disk_monitoring_failing` (`problem`, diagnostic) — `ON` when disk polling is stale or has 3+ consecutive failures, with `last_success`, `last_error`, and `consecutive_failures`.
  * `sensor.idrac_<server>_disks_last_checked` (`timestamp`, diagnostic) — UTC timestamp of the last successful SNMP poll.

---

## Disk Health Monitoring (SNMP)

The add-on autodetects all physical disks connected to your server's storage/RAID controller via SNMP (`IDRAC-MIB::physicalDiskTable` OID `1.3.6.1.4.1.674.10892.5.5.1.20.130.4.1`) and cleans up discovery if a disk is removed.

### How to Enable SNMP on iDRAC
1. Log in to your iDRAC web interface and go to **iDRAC Settings** > **Network** > **Services** (or **SNMP Agent**).
2. Enable **SNMP Agent**, set an **SNMP Community Name** (e.g. `public`), and keep port `161`.
3. Enable **Disk Health Monitoring** in the server's Edit page in the add-on Web UI.
*(Requires an iDRAC Express or Enterprise licence; basic BMC does not include the SNMP agent. SNMPv2c sends the community string unencrypted on your network.)*

### What Is and Is Not Available
* **Available:** Operational state, component status, SMART predictive-failure alert, model, serial, firmware, capacity (GB), media type (HDD/SSD), bay slot, manufacture date, and SSD endurance remaining (when reported).
* **Not Available:** Raw SMART diagnostic attributes (individual disk temperatures, reallocated sector counts) are not exposed via this MIB.

### Staleness Safety Behaviour
If 3 consecutive SNMP polls fail, or if no successful poll has completed within `max(3 * disk_poll_minutes, 30 minutes)` (including if the poller thread dies, in which case it is restarted once):
* If the last known state was healthy, per-disk sensors and `disks_problem` go `unavailable` so stale `"OK"` states are never kept alive.
* If a disk problem was already active, `disks_problem` stays `ON` and affected disks keep their problem state with `stale: true` and `last_success`.
* `disk_monitoring_failing` turns `ON` with the error details.

### Home Assistant Automation Example

```yaml
alias: "Alert: Server Physical Disk Issue"
description: "Notify when a physical disk problem is detected on the iDRAC server"
trigger:
  - platform: state
    entity_id: binary_sensor.idrac_r720_main_disks_problem
    to: "on"
action:
  - service: notify.persistent_notification
    data:
      title: "Storage Alert: {{ state_attr('binary_sensor.idrac_r720_main_disks_problem', 'problems') | join(', ') }}"
      message: >
        Server storage alert: {{ state_attr('binary_sensor.idrac_r720_main_disks_problem', 'summary') }}.
        Affected disks: {{ state_attr('binary_sensor.idrac_r720_main_disks_problem', 'problems') | join(', ') }}.
mode: single
```

## Does My Server Work?

* **Supported Generations:** Dell PowerEdge servers with **iDRAC 7** (12G) and **iDRAC 8** (13G) equipped with a storage or PERC RAID controller.
* **Verified Hardware:** Verified on iDRAC 7 (Dell PowerEdge R720, firmware 2.41, PERC Integrated RAID Controller 1, 8 SAS HDDs).

## Troubleshooting

* **Fan control refused (`0xc1` / `0xd4` / `0xd5`):** Either the server is iDRAC 9/10 (which rejects the 12G/13G Dell OEM raw fan commands with `0xc1`/`0xd5`) or the IPMI account privilege is below **Administrator** (`0xd4`). The add-on remembers the refusal and continues monitoring temperatures/fans without sending fan commands.
* **SNMP not responding:** Verify that your server has an iDRAC Express or Enterprise licence, that **SNMP Agent** is enabled in iDRAC network services, that UDP port `161` is reachable from Home Assistant, and that the community string matches. Use **Test disk SNMP** on the Edit Server page to check raw output.
* **Sensors show `Unknown` right after start:** MQTT discovery runs on the first poll cycle (and disk discovery on the first SNMP walk). Wait one poll interval, or check the add-on log if MQTT connection failed.
* **Log levels & PID debug fields:** Set `log_level` to `debug` in the HA Configuration tab (then restart) to log each PID cycle: `samples` is the count of learned steady-state windows, `theta=[a, b]` are the fitted thermal-resistance parameters, and `feedforward` is the model's baseline fan speed (%) before integral `trim` and proportional correction.
