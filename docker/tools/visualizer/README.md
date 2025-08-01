# Autoware Tools - Visualizer

Opens a remote RViz display for Autoware.

## Standalone Run

```bash
docker run --rm --name visualizer -p 6080:6080 ghcr.io/autowarefoundation/autoware-tools:visualizer
```

## Settings

The following environment variables can be configured with `-e` while launching the visualizer container:

| Variable          | Default Value                  | Possible Values | Description                                                                                                                              |
| ----------------- | ------------------------------ | --------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `RVIZ_CONFIG`     | `/autoware/rviz/autoware.rviz` | Any valid path  | The full path to the RViz configuration file inside the container                                                                        |
| `REMOTE_DISPLAY`  | `true`                         | `true`, `false` | By default, the rviz2 is redirected to a URL to enable browser-based visualization. Set this to `false` to launch a local rviz2 display. |
| `REMOTE_PASSWORD` | `openadkit`                    | Any string      | Password for remote display (only used when `REMOTE_DISPLAY=true`)                                                                       |
