# HA-iDRAC — iDRAC 7/8 (12G/13G) fork

Home Assistant add-ons for monitoring and controlling Dell PowerEdge servers
through their iDRAC via IPMI. This is a fork of
[Aesgarth/HA-iDRAC](https://github.com/Aesgarth/HA-iDRAC) with fixes for
13th-generation servers (iDRAC 8, e.g. **T430**, R430, R630, R730) and for a
number of robustness problems in the original add-ons.

## What this fork changes

- **13G CPU temperature sensors are found.** iDRAC 8 reports `CPU1 Temp` /
  `CPU2 Temp` (processor entities `3.x`); the original stable add-on only
  matched a sensor literally named `Temp` and therefore never controlled fans
  on a T430/R730. Parsing now uses the entity ID and falls back to the
  12G-style name, so both generations work.
- **PSU status is no longer wrong on 13G.** The old check required a
  `Voltage 1` sensor that iDRAC 8 does not expose; presence plus `PSn PG Fail`
  is used instead.
- **Fan control no longer flaps.** A 2 °C hysteresis band is applied to the
  base/high thresholds and to the hand-back-to-iDRAC critical threshold.
- **Safe fan floor.** 0–4 % requests are clamped to 5 %; the web UI and add-on
  schema reject values below 5 %.
- **MQTT outages no longer kill monitoring.** The BETA add-on retries the
  broker instead of stopping the worker, and re-subscribes/re-publishes
  discovery after a reconnect.
- **Passwords are no longer on the `ipmitool` command line.** `-E` +
  `IPMI_PASSWORD` is used, so the iDRAC password does not appear in `ps`.
- **Watchdog enabled** so Supervisor restarts an add-on that exits unexpectedly.
- **Removed the unused `master_encryption_key` option.** It was never used by
  the code; iDRAC credentials live in the add-on's private `/data` volume.
- **Tests + CI** for the SDR parsers and the fan hysteresis logic.
- The stale `multi-server` add-on was removed; the BETA add-on already
  supports multiple servers and is the only one still maintained.

## Add-ons

1. **HA iDRAC Controller** (`ha_idrac_controller`, version 0.1.3, "stable")
   - Single server, configured from the add-on Configuration tab.
   - Simple 3-tier fan control with hysteresis, monitoring, MQTT discovery.

2. **HA iDRAC Controller BETA** (`ha_idrac_controller_development`, version 0.1.0-dev.14)
   - Multi-server, per-server fan modes (simple / curve / target temperature),
     monitor-only mode, PSU + connectivity sensors, graceful shutdown button.
   - Recommended for T430/iDRAC 8 because it is the actively developed one.

> Both add-ons use the same fan-control commands as the original project:
> `raw 0x30 0x30 0x01 0x00` (manual) and `raw 0x30 0x30 0x02 0xff <pct>`.
> These are the standard 12G/13G Dell sequence. Do not use this fork on
> iDRAC 9 (14G+) without checking the raw commands for your firmware.

## Installation

1. In Home Assistant go to **Settings → Add-ons → Add-on Store**.
2. Open the three-dot menu → **Repositories**.
3. Paste this URL (no markdown, no brackets):

   ```
   https://github.com/sam2kb/HA-iDRAC
   ```

4. Install **HA iDRAC Controller BETA** for a T430.
5. Enable **IPMI over LAN** on the iDRAC (iDRAC Settings → Network →
   IPMI Settings), set the privilege limit to Administrator, and make sure HA
   can reach UDP 623 on the iDRAC.
6. Start Mosquitto (or another MQTT broker) **before** starting the add-on,
   fill in the MQTT settings, save, start the add-on, then add the server in
   the web UI (Manage Servers) and restart the add-on once.

The original repository also defined a third add-on, `multi-server`, which was
an older snapshot of the BETA code. It has been removed here. If you previously
added the upstream repository, remove it before adding this one — the add-on
slugs are the same, so keep only one of the two repositories in the store.

## Configuration notes

- Fan values are percentages of the iDRAC's maximum fan speed, not RPM.
  Start with `base_fan_speed_percent: 20`, `low_temp_threshold: 45`,
  `high_temp_fan_speed_percent: 50`, `critical_temp_threshold: 65` and watch
  the temperatures for a day before lowering anything.
- The BETA add-on stores servers in `/data/servers_config.json` (root-only on
  HA OS) with the iDRAC password in plain text. Do not reuse an administrator
  password from another system.
- On stop the BETA add-on hands the fans back to Dell's automatic profile. If
  the container is killed without running its shutdown handler, the iDRAC
  stays in manual mode at the last speed — enable the Supervisor watchdog
  (done in `config.yaml`) and keep an eye on the server after host crashes.
- Sensor naming differs between firmware versions. If the log shows
  "No CPU temperature sensor found", set `log_level: debug`, check the raw
  SDR output in the log, and open an issue with the `sdr type temperature`
  lines for your model.

## Known limitations

- Third-party PCIe cards can make the iDRAC ramp the fans even in manual
  mode on some firmware; upstream issue #4 asks for the OEM command that
  disables that response and it is deliberately not implemented here (the
  command bytes differ between firmware generations and a wrong write is
  worse than a noisy fan). Use the iDRAC's own "Third-Party PCIe Card
  Cooling Response" setting if your firmware has it.
- iDRAC 9 / 14th generation and newer are not a target of this fork. The raw
  fan commands and sensor layout differ.
- A `SIGKILL`, OOM kill or host power loss cannot run the shutdown handler, so
  the iDRAC can stay in manual mode. The Supervisor watchdog restarts the
  add-on in that case, which re-applies the configured fan profile.
- The 13G parsing is covered by unit tests and a fake-IPMI run, not by a
  physical T430. The first start on real hardware should be monitored.

## Development


```sh
python -m pytest -q
```

The tests run without an iDRAC and without Flask/paho installed; they cover
12G/13G SDR parsing, fan RPM/power/PSU parsing and the hysteresis logic.
GitHub Actions runs the same suite on every push.

## License

MIT, same as the upstream project. Original work by
[Aesgarth](https://github.com/Aesgarth/HA-iDRAC).
