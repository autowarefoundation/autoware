# yabloc_monitor

YabLoc monitor is a node that monitors the status of the YabLoc localization system. It is a wrapper node that monitors the status of the YabLoc localization system and publishes the status as diagnostics.

## Feature

### Availability

The node monitors the final output pose of YabLoc to verify the availability of YabLoc.

### Others

To be added,

## Interfaces

### Input

| Name                  | Type                        | Description                     |
| --------------------- | --------------------------- | ------------------------------- |
| `~/input/yabloc_pose` | `geometry_msgs/PoseStamped` | The final output pose of YabLoc |

### Output

| Name           | Type                              | Description         |
| -------------- | --------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Diagnostics outputs |

### Parameters

{{ json_to_markdown("localization/yabloc/yabloc_monitor/schema/yabloc_monitor.schema.json") }}
