## 0.1.0-dev.15 - 2026-10-18

* 🐛 **Bug Fix**: Inlet/exhaust sensors are matched at the start of the sensor
  name, so `PSU1 Inlet Temp` can no longer replace the chassis inlet reading
  (same failure mode reported upstream in tigerblue77's controller).
* 🐛 **Bug Fix**: If the manual-mode command lands but the fan-speed command
  fails, the fans are handed straight back to Dell auto instead of being left
  pinned at the previous manual speed.
* 🐛 **Bug Fix**: Temperature readings outside -20..125 °C are ignored.
* 🛡 **Safety**: A server that refuses fan control with completion code
  0xc1/0xd4/0xd5 (iDRAC 9/10, or an account below Administrator) is detected
  once, fan control is disabled, and monitoring continues without repeating the
  failing command every cycle.
* ✨ **New option**: per-server "Third-party PCIe cooling" to disable Dell's
  default cooling response for non-Dell cards using the 12G/13G OEM command
  (`0x30 0xce ... 0x01 0x00 0x00`); restored to Dell default on shutdown.
* ✨ **Diagnostics**: iDRAC firmware version and server serial number are read
  at startup and published as the MQTT device's `sw_version`/`serial_number`.
* 🔧 **Validation**: contradictory thresholds (critical <= low, high < base)
  are corrected with a warning instead of silently degrading the profile.
* 🩺 **Health**: Supervisor watchdog now probes `/health`, which fails only when
  every server worker has died — an unreachable iDRAC or MQTT broker does not
  trigger restart loops.
* ✅ **Tests**: 25 tests now cover the new refusal, cooling-response, firmware,
  serial and validation paths.

## 0.1.0-dev.14 - 2026-10-18

* 🐛 **Bug Fix (13G/iDRAC 8)**: CPU temperatures are now detected by IPMI
  processor entity (`3.x`), so `CPU1 Temp`/`CPU2 Temp` on T430/R630/R730 are
  found. Previously only a sensor literally named `Temp` (12G) matched and fan
  control never engaged on 13G servers.
* 🐛 **Bug Fix (13G/iDRAC 8)**: PSU status now uses `PSn Status` presence and
  `PSn PG Fail` instead of a `Voltage n` sensor that iDRAC 8 does not expose.
* 🐛 **Bug Fix**: 2 °C hysteresis on the base/high and critical thresholds; the
  fans no longer flip between manual and Dell auto every cycle.
* 🐛 **Bug Fix**: Fan speeds below 5 % are clamped to 5 %.
* 🐛 **Bug Fix**: MQTT connection failures no longer stop a server worker
  permanently; reconnects re-subscribe and re-publish discovery.
* 🔒 **Security**: iDRAC password is passed via `ipmitool -E`/`IPMI_PASSWORD`
  instead of the command line.
* 🔧 **Change**: Removed the unused `master_encryption_key` option and the
  unused `temperature_unit` option (the add-on reports °C).
* 🔧 **Change**: Enabled the Supervisor watchdog.
* ✅ **Tests**: Added pytest coverage for SDR parsing (12G/13G), fan RPM,
  power, PSU status and the hysteresis logic, plus a GitHub Actions workflow.

## 0.1.0-dev.13 - 2025-09-24

* ✨ **New Feature**: Added a "monitor-only" mode to disable fan control on a per-server basis.
* 🐛 **Bug Fix**: The addon now correctly parses power consumption from older iDRAC 6 servers using the "System Level" sensor.
* 🎨 **UI Improvement**: Added a "Fan Control" dropdown to the Add and Edit Server pages.

## 0.1.0-dev.12 - 2025-09-20

* Previous updated release.
