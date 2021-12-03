# velodyne_monitor

## Purpose

This node monitors the status of Velodyne LiDARs.
The result of the status is published as diagnostics.

## Inner-workings / Algorithms

The status of Velodyne LiDAR can be retrieved from `http://[ip_address]/cgi/{info, settings, status, diag}.json`.

The types of abnormal status and corresponding diagnostics status are following.

| Abnormal status                                     | Diagnostic status |
| --------------------------------------------------- | ----------------- |
| No abnormality                                      | OK                |
| Top board temperature is too cold                   | ERROR             |
| Top board temperature is cold                       | WARN              |
| Top board temperature is too hot                    | ERROR             |
| Top board temperature is hot                        | WARN              |
| Bottom board temperature is too cold                | ERROR             |
| Bottom board temperature is cold                    | WARN              |
| Bottom board temperature is too hot                 | ERROR             |
| Bottom board temperature is hot                     | WARN              |
| Rpm(Rotations per minute) of the motor is too low   | ERROR             |
| Rpm(Rotations per minute) of the motor is low       | WARN              |
| Connection error (cannot get Velodyne LiDAR status) | ERROR             |

## Inputs / Outputs

### Input

None

### Output

| Name           | Type                              | Description         |
| -------------- | --------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Diagnostics outputs |

## Parameters

### Node Parameters

| Name      | Type   | Default Value | Description                                               |
| --------- | ------ | ------------- | --------------------------------------------------------- |
| `timeout` | double | 0.5           | Timeout for HTTP request to get Velodyne LiDAR status [s] |

### Core Parameters

| Name              | Type   | Default Value   | Description                                                                                                               |
| ----------------- | ------ | --------------- | ------------------------------------------------------------------------------------------------------------------------- |
| `ip_address`      | string | "192.168.1.201" | IP address of target Velodyne LiDAR                                                                                       |
| `temp_cold_warn`  | double | -5.0            | If the temperature of Velodyne LiDAR is lower than this value, the diagnostics status becomes WARN [°C]                   |
| `temp_cold_error` | double | -10.0           | If the temperature of Velodyne LiDAR is lower than this value, the diagnostics status becomes ERROR [°C]                  |
| `temp_hot_warn`   | double | 75.0            | If the temperature of Velodyne LiDAR is higher than this value, the diagnostics status becomes WARN [°C]                  |
| `temp_hot_error`  | double | 80.0            | If the temperature of Velodyne LiDAR is higher than this value, the diagnostics status becomes ERROR [°C]                 |
| `rpm_ratio_warn`  | double | 0.80            | If the rpm rate of the motor (= current rpm / default rpm) is lower than this value, the diagnostics status becomes WARN  |
| `rpm_ratio_error` | double | 0.70            | If the rpm rate of the motor (= current rpm / default rpm) is lower than this value, the diagnostics status becomes ERROR |

## Assumptions / Known limits

TBD.
