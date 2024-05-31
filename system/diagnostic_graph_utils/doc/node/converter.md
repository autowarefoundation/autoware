# Converter tool

This tool converts `/diagnostics_graph` to `/diagnostics_array` so it can be read by tools such as `rqt_runtime_monitor`.

## Usage

```bash
ros2 run diagnostic_graph_utils converter_node
```

## Examples

Terminal 1:

```bash
ros2 launch diagnostic_graph_aggregator example-main.launch.xml
```

Terminal 2:

```bash
ros2 run diagnostic_graph_utils converter_node
```

Terminal 3:

```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor --ros-args -r diagnostics:=diagnostics_array
```

![rqt_runtime_monitor](./images/rqt_runtime_monitor.png)
