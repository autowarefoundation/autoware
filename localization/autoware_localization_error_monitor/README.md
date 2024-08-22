# autoware_localization_error_monitor

## Purpose

<p align="center">
<img src="./media/diagnostics.png" width="400">
</p>

autoware_localization_error_monitor is a package for diagnosing localization errors by monitoring uncertainty of the localization results.
The package monitors the following two values:

- size of long radius of confidence ellipse
- size of confidence ellipse along lateral direction (body-frame)

## Inputs / Outputs

### Input

| Name         | Type                      | Description         |
| ------------ | ------------------------- | ------------------- |
| `input/odom` | `nav_msgs::msg::Odometry` | localization result |

### Output

| Name                   | Type                                    | Description         |
| ---------------------- | --------------------------------------- | ------------------- |
| `debug/ellipse_marker` | `visualization_msgs::msg::Marker`       | ellipse marker      |
| `diagnostics`          | `diagnostic_msgs::msg::DiagnosticArray` | diagnostics outputs |

## Parameters

{{ json_to_markdown("localization/autoware_localization_error_monitor/schema/localization_error_monitor.schema.json") }}
