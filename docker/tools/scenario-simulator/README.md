# Autoware Tools - Scenario Simulator

Runs Scenario Simulator in a container.

## Standalone Run

```bash
docker run --rm --name scenario-simulator ghcr.io/autowarefoundation/autoware-tools:scenario-simulator
```

## Settings

The following environment variables can be passed to the `scenario-simulator` container:

| Variable              | Default Value                                                                                                                                                   | Possible Values             | Description                                                                                                                          |
| --------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `SCENARIO`            | [`scenario_test_runner/scenario/sample.yaml`](https://github.com/tier4/scenario_simulator_v2/blob/master/test_runner/scenario_test_runner/scenario/sample.yaml) | Any valid path              | The full path to the scenario file inside the container                                                                              |
| `ARCHITECTURE_TYPE`   | `awf/universe/20240605`                                                                                                                                         | Any valid architecture type | The architecture type to use for the scenario simulation                                                                             |
| `SENSOR_MODEL`        | `sample_sensor_kit`                                                                                                                                             | Any valid sensor model      | The sensor model to use for the scenario simulation                                                                                  |
| `VEHICLE_MODEL`       | `sample_vehicle`                                                                                                                                                | Any valid vehicle model     | The vehicle model to use for the scenario simulation                                                                                 |
| `LAUNCH_AUTOWARE`     | `false`                                                                                                                                                         | `true`, `false`             | Set to `true` to launch Autoware for standalone scenario simulation. By default, the scenario simulator expects an Autoware running. |
| `LAUNCH_RVIZ`         | `true`                                                                                                                                                          | `true`, `false`             | Set to `true` to launch RViz for the scenario simulation                                                                             |
| `INITIALIZE_DURATION` | `90`                                                                                                                                                            | Any positive integer        | The duration to initialize the scenario simulation                                                                                   |
| `GLOBAL_FRAME_RATE`   | `30`                                                                                                                                                            | Any positive integer        | The frame rate of the scenario simulation                                                                                            |
| `GLOBAL_TIMEOUT`      | `120`                                                                                                                                                           | Any positive integer        | The timeout of the scenario simulation                                                                                               |
| `OUTPUT_DIRECTORY`    | `/autoware/scenario-sim/output`                                                                                                                                 | Any valid path              | The directory to save the simulation results                                                                                         |
| `USE_SIM_TIME`        | `false`                                                                                                                                                         | `true`, `false`             | Whether to use simulation time instead of system time                                                                                |
| `RECORD`              | `false`                                                                                                                                                         | `true`, `false`             | Whether to record the scenario simulation rosbag                                                                                     |
