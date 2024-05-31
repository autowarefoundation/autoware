# Dump tool

This tool displays `/diagnostics_graph` in table format.

## Usage

```bash
ros2 run diagnostic_graph_utils dump_node
```

## Examples

Terminal 1:

```bash
ros2 launch diagnostic_graph_aggregator example-main.launch.xml
```

Terminal 2:

```bash
ros2 run diagnostic_graph_utils dump_node
```

Output:

```txt
|----|-------|----------------------------------|------|----------|
| No | Level | Path                             | Type | Children |
|----|-------|----------------------------------|------|----------|
|  1 | OK    | /autoware/modes/stop             | ok   |          |
|  2 | ERROR | /autoware/modes/autonomous       | and  | 8 9      |
|  3 | OK    | /autoware/modes/local            | and  | 13       |
|  4 | OK    | /autoware/modes/remote           | and  | 14       |
|  5 | OK    | /autoware/modes/emergency_stop   | ok   |          |
|  6 | OK    | /autoware/modes/comfortable_stop | and  | 9        |
|  7 | ERROR | /autoware/modes/pull_over        | and  | 8 9      |
|  8 | ERROR | /functions/pose_estimation       | and  | 10       |
|  9 | OK    | /functions/obstacle_detection    | or   | 11 12    |
| 10 | ERROR | /sensing/lidars/top              | diag |          |
| 11 | OK    | /sensing/lidars/front            | diag |          |
| 12 | OK    | /sensing/radars/front            | diag |          |
| 13 | OK    | /external/joystick_command       | diag |          |
| 14 | OK    | /external/remote_command         | diag |          |

```
