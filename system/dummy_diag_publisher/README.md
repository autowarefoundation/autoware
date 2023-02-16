# dummy_diag_publisher

## Purpose

This package outputs a dummy diagnostic data for debugging and developing.

## Inputs / Outputs

### Outputs

| Name           | Type                                     | Description         |
| -------------- | ---------------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs::msgs::DiagnosticArray` | Diagnostics outputs |

## Parameters

### Node Parameters

The parameter `DIAGNOSTIC_NAME` must be a name that exists in the parameter YAML file. If the parameter `status` is given from a command line, the parameter `is_active` is automatically set to `true`.

| Name                        | Type   | Default Value | Explanation                             | Reconfigurable |
| --------------------------- | ------ | ------------- | --------------------------------------- | -------------- |
| `update_rate`               | int    | `10`          | Timer callback period [Hz]              | false          |
| `DIAGNOSTIC_NAME.is_active` | bool   | `true`        | Force update or not                     | true           |
| `DIAGNOSTIC_NAME.status`    | string | `"OK"`        | diag status set by dummy diag publisher | true           |

### YAML format for dummy_diag_publisher

If the value is `default`, the default value will be set.

| Key                                        | Type   | Default Value | Explanation                             |
| ------------------------------------------ | ------ | ------------- | --------------------------------------- |
| `required_diags.DIAGNOSTIC_NAME.is_active` | bool   | `true`        | Force update or not                     |
| `required_diags.DIAGNOSTIC_NAME.status`    | string | `"OK"`        | diag status set by dummy diag publisher |

## Assumptions / Known limits

TBD.

## Usage

### launch

```sh
ros2 launch dummy_diag_publisher dummy_diag_publisher.launch.xml
```

### reconfigure

```sh
ros2 param set /dummy_diag_publisher velodyne_connection.status "Warn"
ros2 param set /dummy_diag_publisher velodyne_connection.is_active true
```
