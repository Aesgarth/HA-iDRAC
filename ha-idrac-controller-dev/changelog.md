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
