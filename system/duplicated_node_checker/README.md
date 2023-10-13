# Duplicated Node Checker

## Purpose

This node monitors the ROS 2 environments and detect duplication of node names in the environment.
The result is published as diagnostics.

### Standalone Startup

```bash
ros2 launch duplicated_node_checker duplicated_node_checker.launch.xml
```

## Inner-workings / Algorithms

The types of topic status and corresponding diagnostic status are following.

| Duplication status    | Diagnostic status | Description                |
| --------------------- | ----------------- | -------------------------- |
| `OK`                  | OK                | No duplication is detected |
| `Duplicated Detected` | ERROR             | Duplication is detected    |

## Inputs / Outputs

### Output

| Name           | Type                              | Description         |
| -------------- | --------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Diagnostics outputs |

## Parameters

{{ json_to_markdown("system/duplicated_node_checker/schema/duplicated_node_checker.schema.json") }}

## Assumptions / Known limits

TBD.
