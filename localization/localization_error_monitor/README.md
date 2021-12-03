# localization_error_monitor

## Purpose

localization_error_monitor is a package for diagnosing localization errors by monitoring uncertainty of the localization results.
The package monitors the following two values:

- size of long radius of confidence ellipse
- size of confidence ellipse along lateral direction (body-frame)

## Inputs / Outputs

### Input

| Name                  | Type                                            | Description         |
| --------------------- | ----------------------------------------------- | ------------------- |
| `input/pose_with_cov` | `geometry_msgs::msg::PoseWithCovarianceStamped` | localization result |

### Output

| Name                   | Type                                    | Description         |
| ---------------------- | --------------------------------------- | ------------------- |
| `debug/ellipse_marker` | `visualization_msgs::msg::Marker`       | ellipse marker      |
| `diagnostics`          | `diagnostic_msgs::msg::DiagnosticArray` | diagnostics outputs |

## Parameters

| Name                                   | Type   | Description                                                                                 |
| -------------------------------------- | ------ | ------------------------------------------------------------------------------------------- |
| `scale`                                | double | scale factor for monitored values (default: 3.0)                                            |
| `error_ellipse_size`                   | double | error threshold for long radius of confidence ellipse [m] (default: 1.0)                    |
| `warn_ellipse_size`                    | double | warning threshold for long radius of confidence ellipse [m] (default: 0.8)                  |
| `error_ellipse_size_lateral_direction` | double | error threshold for size of confidence ellipse along lateral direction [m] (default: 0.3)   |
| `warn_ellipse_size_lateral_direction`  | double | warning threshold for size of confidence ellipse along lateral direction [m] (default: 0.2) |
