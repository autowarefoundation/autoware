# ROS topics: GPU Monitor

> Intel and tegra platform only.<br>
> Raspi platform not supported.

## <u>GPU Temperature</u>

/diagnostics/gpu_monitor: GPU Temperature

<b>[summary]</b>

| level | message |
| ----- | ------- |
| OK    | OK      |
| WARN  | warm    |
| ERROR | hot     |

<b>[values]</b>

| key (example)                       | value (example) |
| ----------------------------------- | --------------- |
| GeForce GTX 1650, thermal_zone[0-9] | 46.0 DegC       |

\*key: thermal_zone[0-9] for ARM architecture.

## <u>GPU Usage</u>

/diagnostics/gpu_monitor: GPU Usage

<b>[summary]</b>

| level | message        |
| ----- | -------------- |
| OK    | OK             |
| WARN  | high load      |
| ERROR | very high load |

<b>[values]</b>

| key               | value (example)                 |
| ----------------- | ------------------------------- |
| GPU [0-9]: status | OK / high load / very high load |
| GPU [0-9]: name   | GeForce GTX 1650, gpu.[0-9]     |
| GPU [0-9]: usage  | 19.0%                           |

\*key: gpu.[0-9] for ARM architecture.

## <u>GPU Memory Usage</u>

> Intel platform only.<br>
> There is no separate gpu memory in tegra. Both cpu and gpu uses cpu memory.

/diagnostics/gpu_monitor: GPU Memory Usage

<b>[summary]</b>

| level | message        |
| ----- | -------------- |
| OK    | OK             |
| WARN  | high load      |
| ERROR | very high load |

<b>[values]</b>

| key               | value (example)                 |
| ----------------- | ------------------------------- |
| GPU [0-9]: status | OK / high load / very high load |
| GPU [0-9]: name   | GeForce GTX 1650                |
| GPU [0-9]: usage  | 13.0%                           |
| GPU [0-9]: total  | 3G                              |
| GPU [0-9]: used   | 1G                              |
| GPU [0-9]: free   | 2G                              |

## <u>GPU Thermal Throttling</u>

> Intel platform only.<br>
> Tegra platform not supported.

/diagnostics/gpu_monitor: GPU Thermal Throttling

<b>[summary]</b>

| level | message    |
| ----- | ---------- |
| OK    | OK         |
| ERROR | throttling |

<b>[values]</b>

| key                       | value (example)                  |
| ------------------------- | -------------------------------- |
| GPU [0-9]: status         | OK / throttling                  |
| GPU [0-9]: name           | GeForce GTX 1650                 |
| GPU [0-9]: graphics clock | 1020 MHz                         |
| GPU [0-9]: reasons        | GpuIdle / SwThermalSlowdown etc. |

## <u>GPU Frequency</u>

/diagnostics/gpu_monitor: GPU Frequency

### Intel platform

<b>[summary]</b>

| level | message           |
| ----- | ----------------- |
| OK    | OK                |
| WARN  | unsupported clock |

<b>[values]</b>

| key                       | value (example)        |
| ------------------------- | ---------------------- |
| GPU [0-9]: status         | OK / unsupported clock |
| GPU [0-9]: name           | GeForce GTX 1650       |
| GPU [0-9]: graphics clock | 1020 MHz               |

### Tegra platform

<b>[summary]</b>

| level | message |
| ----- | ------- |
| OK    | OK      |

<b>[values]</b>

| key (example)             | value (example) |
| ------------------------- | --------------- |
| GPU 17000000.gv11b: clock | 318 MHz         |
