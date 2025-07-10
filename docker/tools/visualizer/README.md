# Open AD Kit - Visualizer

Opens a remote RViz display for Autoware.

## Standalone Run

```bash
docker run --rm --name visualizer -p 6080:6080 ghcr.io/autowarefoundation/openadkit:visualizer
```

## Settings

The following environment variables can be passed to the `visualizer` container:

| Variable          | Default Value                  | Possible Values                       | Description                                                                                                                              |
| ----------------- | ------------------------------ | ------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `RVIZ_CONFIG`     | `/autoware/rviz/autoware.rviz` | Any valid path                        | The full path to the RViz configuration file inside the container                                                                        |
| `REMOTE_DISPLAY`  | `true`                         | `true`, `false`                       | **(Recommended)** Light-weight and browser-based RViz display, accessible from any device.                                               |
| `REMOTE_PASSWORD` | `openadkit`                    | Any string without special characters | Password for remote display (only used when `REMOTE_DISPLAY=true`).                                                                      |
| `LOCAL_DISPLAY`   | `false`                        | `true`, `false`                       | Local RViz display, useful for debugging and development. `/tmp/.X11-unix:/tmp/.X11-unix` volume mount required to enable local display. |
