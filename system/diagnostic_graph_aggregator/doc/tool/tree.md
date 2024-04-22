# Tree tool

This tool displays the graph structure of the configuration file in tree format.

## Usage

```bash
ros2 run diagnostic_graph_aggregator tree <graph-config-path>
```

## Examples

```bash
ros2 run diagnostic_graph_aggregator tree '$(find-pkg-share diagnostic_graph_aggregator)/example/graph/main.yaml'
```

```txt
===== Top-level trees ============================
- /autoware/modes/autonomous (and)
    - /functions/pose_estimation (and)
    - /functions/obstacle_detection (or)
- /autoware/modes/local (and)
    - /external/joystick_command (diag)
- /autoware/modes/remote (and)
    - /external/remote_command (diag)
- /autoware/modes/comfortable_stop (and)
    - /functions/obstacle_detection (or)
- /autoware/modes/pull_over (and)
    - /functions/pose_estimation (and)
    - /functions/obstacle_detection (or)
===== Subtrees ===================================
- /functions/pose_estimation (and)
    - /sensing/lidars/top (diag)
- /functions/obstacle_detection (or)
    - /sensing/lidars/front (diag)
    - /sensing/radars/front (diag)
===== Isolated units =============================
- /autoware/modes/stop (ok)
- /autoware/modes/emergency_stop (ok)
```
