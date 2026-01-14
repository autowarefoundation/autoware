# Autoware Tools - Visualizer

Opens a remote RViz display for Autoware.

## Standalone Run

```bash
docker run --rm --name autoware-visualizer --net=host ghcr.io/autowarefoundation/autoware-tools:visualizer
```

## Settings

The following environment variables can be configured with `-e` while launching the visualizer container:

| Variable          | Default Value                  | Possible Values                       | Description                                                                                                                                    |
| ----------------- | ------------------------------ | ------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `PASSWORD`         | `openadkit`                    | Any string without special characters | Password for remote display                                                                             |
| `RVIZ_CONFIG_PATH` | `/autoware/rviz/autoware.rviz` | Any valid path                        | The full path to the RViz configuration file inside the container                                                                              |
| `USE_SIM_TIME`    | `false`                        | `true`, `false`                       | Whether to use simulation time                                                                                                               |
| `VEHICLE_MODEL`   | `sample_vehicle`               | Valid vehicle model name that has vehicle description          | The vehicle model to use for the simulation in RViz                                                                                            |
