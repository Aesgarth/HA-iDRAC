# Home Assistant Automation Examples

These examples use the placeholder server alias `my-server` (for example `binary_sensor.idrac_my_server_fan_fault`). Replace `my_server` with the sanitized alias you configured for your server in the add-on Web UI.

## 1. Fan Fault Alert

Triggers when a fan fault persists for 2 minutes and includes the fault description from the `status` attribute (such as `"Fan4: stopped"`).

```yaml
alias: "iDRAC: Fan Fault"
description: "Alert when a server fan fault persists for 2 minutes"
triggers:
  - trigger: state
    entity_id: binary_sensor.idrac_my_server_fan_fault
    to: "on"
    for:
      minutes: 2
conditions: []
actions:
  - action: notify.notify
    data:
      title: "Server Fan Fault (my-server)"
      message: >
        Fan fault detected: {{ state_attr('binary_sensor.idrac_my_server_fan_fault', 'status') }}.
mode: single
```

## 2. Disk Health Problem

Triggers when any physical disk reports a problem and includes the `summary` and `problems` attributes.

```yaml
alias: "iDRAC: Disk Problem"
description: "Alert when SNMP disk monitoring reports a physical disk problem"
triggers:
  - trigger: state
    entity_id: binary_sensor.idrac_my_server_disks_problem
    to: "on"
conditions: []
actions:
  - action: notify.notify
    data:
      title: "Server Disk Alert (my-server)"
      message: >
        {{ state_attr('binary_sensor.idrac_my_server_disks_problem', 'summary') }}.
        Affected disks: {{ state_attr('binary_sensor.idrac_my_server_disks_problem', 'problems') | join(', ') }}.
mode: single
```

## 3. Power Supply (PSU) Problem

Triggers when either PSU problem binary sensor stays `on` for 1 minute.

```yaml
alias: "iDRAC: PSU Problem"
description: "Alert when a power supply fault lasts for 1 minute"
triggers:
  - trigger: state
    entity_id:
      - binary_sensor.idrac_my_server_psu1
      - binary_sensor.idrac_my_server_psu2
    to: "on"
    for:
      minutes: 1
conditions: []
actions:
  - action: notify.notify
    data:
      title: "Server PSU Fault (my-server)"
      message: >
        {{ trigger.to_state.name }} is reporting a problem.
mode: single
```

## 4. iDRAC Unreachable

Triggers when iDRAC stops responding for 3 minutes. Note that while iDRAC is unreachable, fan control is not possible and the add-on is repeatedly trying to hand fans back to Dell auto.

```yaml
alias: "iDRAC: Unreachable"
description: "Alert when iDRAC stops responding; fan control is paused and hand-back to Dell auto is being retried"
triggers:
  - trigger: state
    entity_id: binary_sensor.idrac_my_server_idrac_reachable
    to: "off"
    for:
      minutes: 3
conditions: []
actions:
  - action: notify.notify
    data:
      title: "iDRAC Unreachable (my-server)"
      message: >
        Cannot reach iDRAC for my-server. Fan control is unavailable and the add-on is retrying hand-back to Dell auto.
mode: single
```

## 5. CPU Too Hot

Triggers when the hottest CPU temperature stays above 70 °C for 5 minutes (adjust `above: 70` to match your CPU's thermal limits).

```yaml
alias: "iDRAC: CPU Too Hot"
description: "Alert when the hottest CPU stays above 70 C for 5 minutes"
triggers:
  - trigger: numeric_state
    entity_id: sensor.idrac_my_server_hottest_cpu_temp
    above: 70
    for:
      minutes: 5
conditions: []
actions:
  - action: notify.notify
    data:
      title: "Server CPU Overheating (my-server)"
      message: >
        Hottest CPU is {{ states('sensor.idrac_my_server_hottest_cpu_temp') }} °C.
mode: single
```

## 6. Fans Pinned High

Triggers when the target fan speed stays above 80% for 30 minutes, which can indicate a hot room, a blocked air filter, or a stuck workload.

```yaml
alias: "iDRAC: Fans Pinned High"
description: "Alert when target fan speed stays above 80% for 30 minutes"
triggers:
  - trigger: numeric_state
    entity_id: sensor.idrac_my_server_target_fan_speed
    above: 80
    for:
      minutes: 30
conditions: []
actions:
  - action: notify.notify
    data:
      title: "Server Fans Running High (my-server)"
      message: >
        Target fan speed has been {{ states('sensor.idrac_my_server_target_fan_speed') }}% for 30 minutes. Check room temperature, filters, or server load.
mode: single
```

## 7. Add-on / Server Monitoring Offline

Triggers when entities go `unavailable` for 5 minutes (for example, if the add-on itself stops publishing or loses MQTT connectivity).

```yaml
alias: "iDRAC: Monitoring Offline"
description: "Alert when the add-on stops publishing MQTT availability"
triggers:
  - trigger: state
    entity_id: sensor.idrac_my_server_hottest_cpu_temp
    to: "unavailable"
    for:
      minutes: 5
conditions: []
actions:
  - action: notify.notify
    data:
      title: "iDRAC Monitoring Offline (my-server)"
      message: >
        iDRAC sensors for my-server have been unavailable for 5 minutes. Check the add-on and MQTT broker.
mode: single
```

## Energy Use Tip

To track server energy consumption in Home Assistant's **Energy** dashboard, create an **Integral sensor** helper (**Settings > Devices & services > Helpers > Create Helper > Integral sensor**) using `sensor.idrac_my_server_power` as the input sensor, with **Integration method** set to `Trapezoidal`, **Metric prefix** `k`, and **Time unit** `Hours` to produce `kWh`.
