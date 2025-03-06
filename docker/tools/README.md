# Open AD Kit Tools

This directory offers tools for [Open AD Kit](https://autoware.org/open-ad-kit/) containers to make deployment and simulation easier.

- `visualizer`: a container that can visualize the Autoware and scenario simulation on RViz.

## Visualizer

### Run

```bash
docker run --rm --name openadkit-visualizer -p 6080:6080 -p 5900:5900 ghcr.io/autowarefoundation/autoware-tools:visualizer
```

### Settings

The following environment variables can be configured for the visualizer container:

| Variable       | Default Value                  | Possible Values | Description                                                       |
| -------------- | ------------------------------ | --------------- | ----------------------------------------------------------------- |
| `RVIZ_CONFIG`  | `/autoware/rviz/autoware.rviz` | Any valid path  | The full path to the RViz configuration file inside the container |
| `WEB_ENABLED`  | `false`                        | `true`, `false` | Enable visualization through a web browser                        |
| `WEB_PASSWORD` | -                              | Any string      | Password for web visualization (required when `WEB_ENABLED=true`) |
