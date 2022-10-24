# ROS topics: CPU Monitor

## <u>CPU Temperature</u>

/diagnostics/cpu_monitor: CPU Temperature

<b>[summary]</b>

| level | message |
| ----- | ------- |
| OK    | OK      |

<b>[values]</b>

| key (example)                               | value (example) |
| ------------------------------------------- | --------------- |
| Package id 0, Core [0-9], thermal_zone[0-9] | 50.0 DegC       |

\*key: thermal_zone[0-9] for ARM architecture.

## <u>CPU Usage</u>

/diagnostics/cpu_monitor: CPU Usage

<b>[summary]</b>

| level | message        |
| ----- | -------------- |
| OK    | OK             |
| WARN  | high load      |
| ERROR | very high load |

<b>[values]</b>

| key                   | value (example)                 |
| --------------------- | ------------------------------- |
| CPU [all,0-9]: status | OK / high load / very high load |
| CPU [all,0-9]: usr    | 2.00%                           |
| CPU [all,0-9]: nice   | 0.00%                           |
| CPU [all,0-9]: sys    | 1.00%                           |
| CPU [all,0-9]: idle   | 97.00%                          |

## <u>CPU Load Average</u>

/diagnostics/cpu_monitor: CPU Load Average

<b>[summary]</b>

| level | message   |
| ----- | --------- |
| OK    | OK        |
| WARN  | high load |

<b>[values]</b>

| key   | value (example) |
| ----- | --------------- |
| 1min  | 14.50%          |
| 5min  | 14.55%          |
| 15min | 9.67%           |

## <u>CPU Thermal Throttling</u>

> Intel and raspi platform only.<br>
> Tegra platform not supported.

/diagnostics/cpu_monitor: CPU Thermal Throttling

<b>[summary]</b>

| level | message    |
| ----- | ---------- |
| OK    | OK         |
| ERROR | throttling |

<b>[values for intel platform]</b>

| key                           | value (example) |
| ----------------------------- | --------------- |
| CPU [0-9]: Pkg Thermal Status | OK / throttling |

<b>[values for raspi platform]</b>

| key    | value (example)                                                 |
| ------ | --------------------------------------------------------------- |
| status | All clear / Currently throttled / Soft temperature limit active |

## <u>CPU Frequency</u>

/diagnostics/cpu_monitor: CPU Frequency

<b>[summary]</b>

| level | message |
| ----- | ------- |
| OK    | OK      |

<b>[values]</b>

| key              | value (example) |
| ---------------- | --------------- |
| CPU [0-9]: clock | 2879MHz         |
