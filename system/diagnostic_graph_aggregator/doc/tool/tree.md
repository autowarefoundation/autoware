# Tree tool

This tool displays the graph structure of the configuration file in tree format.

## Usage

```bash
ros2 run diagnostic_graph_aggregator tree <graph-config-path>
```

## Examples

```bash
ros2 run diagnostic_graph_aggregator tree system/diagnostic_graph_aggregator/example/graph/main.yaml
```

```txt
===== root nodes =================================
- /autoware/modes/local (and)
    - /external/joystick_command (diag)
- /autoware/modes/comfortable_stop (and)
    - /functions/obstacle_detection (or)
- /autoware/modes/pull_over (and)
    - /functions/pose_estimation (and)
    - /functions/obstacle_detection (or)
- /autoware/modes/autonomous (and)
    - /functions/pose_estimation (and)
    - /functions/obstacle_detection (or)
- /autoware/modes/remote (and)
    - /external/remote_command (diag)
===== intermediate nodes =========================
- /functions/obstacle_detection (or)
    - /sensing/lidars/front (diag)
    - /sensing/radars/front (diag)
- /functions/pose_estimation (and)
    - /sensing/lidars/top (diag)
===== isolated nodes =============================
- /autoware/modes/stop (const)
- /autoware/modes/emergency_stop (const)
```
