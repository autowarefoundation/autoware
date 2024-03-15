# Dump tool

This tool displays `/diagnostics_graph` in table format.

## Usage

```bash
ros2 run diagnostic_graph_aggregator dump
```

## Examples

```bash
ros2 launch diagnostic_graph_aggregator example-main.launch.xml
ros2 run diagnostic_graph_aggregator dump
```

```txt
| ----- | ----- | -------------------------------- | ----- |
| index | level | name                             | links |
| ----- | ----- | -------------------------------- | ----- |
| 0     | OK    | /sensing/radars/front            |       |
| 1     | OK    | /sensing/lidars/front            |       |
| 2     | ERROR | /sensing/lidars/top              |       |
| 3     | OK    | /functions/obstacle_detection    | 1 0   |
| 4     | ERROR | /functions/pose_estimation       | 2     |
| 5     | OK    | /external/remote_command         |       |
| 6     | OK    | /external/joystick_command       |       |
| 7     | ERROR | /autoware/modes/pull_over        | 4 3   |
| 8     | OK    | /autoware/modes/comfortable_stop | 3     |
| 9     | OK    | /autoware/modes/emergency_stop   |       |
| 10    | OK    | /autoware/modes/remote           | 5     |
| 11    | OK    | /autoware/modes/local            | 6     |
| 12    | ERROR | /autoware/modes/autonomous       | 4 3   |
| 13    | OK    | /autoware/modes/stop             |       |
```
