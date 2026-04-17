# planning-simulator

Launches the planning simulator with the sample map, vehicle, and sensor kit using the `universe-jazzy` image. The `docker-compose.yaml` here runs the full launch command automatically on container start, then drops to `bash` so you can inspect state after the sim exits.

Planning sim itself doesn't need a GPU — perception is dummy. The only reason to use GPU passthrough here is to get hardware-accelerated rviz rendering. See the NVIDIA section below.

## Prerequisites

- Sample map extracted to `~/autoware_map/sample-map-planning`
- Perception model data under `~/autoware_data` (mounted but not used by this demo)
- Docker Compose v2

[Download the sample map](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/README.md) and unpack it to `~/autoware_map/sample-map-planning` before running.

## Run (no GPU, software rendering)

Works on any host. rviz will use Mesa's `llvmpipe` software rasterizer — fine for light views, laggy on dense point clouds.

```bash
xhost +local:docker

cd docker-new/examples/demos/planning-simulator
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose run --rm planning-simulator
```

## Run (Intel / AMD hosts, hardware accel)

Merge the `docker-compose.dri.yaml` overlay to pass `/dev/dri` into the container. Works with Mesa's open drivers (iris, radeonsi, nouveau). Silently falls back to `llvmpipe` on NVIDIA + proprietary driver, so use the NVIDIA overlay there instead.

```bash
xhost +local:docker

cd docker-new/examples/demos/planning-simulator
HOST_UID=$(id -u) HOST_GID=$(id -g) \
  docker compose -f docker-compose.yaml -f docker-compose.dri.yaml \
  run --rm planning-simulator
```

## Run (NVIDIA hosts, hardware accel)

Merge the `docker-compose.nvidia.yaml` overlay to add `runtime: nvidia`, NVIDIA env, and the `deploy.devices` block. Requires the NVIDIA proprietary driver and `nvidia-container-toolkit`.

```bash
xhost +local:docker

cd docker-new/examples/demos/planning-simulator
HOST_UID=$(id -u) HOST_GID=$(id -g) \
  docker compose -f docker-compose.yaml -f docker-compose.nvidia.yaml \
  run --rm planning-simulator
```

## After the launch exits

The `command:` block ends with `exec bash`, so when `ros2 launch` exits the container drops into an interactive shell instead of disappearing. `Ctrl+D` exits and removes the container (`--rm`).

The compose file defaults `HOST_UID`/`HOST_GID` to `1000`; drop the prefix if your host UID/GID are 1000.

## Customizing

Edit the `command:` block in `docker-compose.yaml` to change launch arguments (different map, vehicle, sensor kit). Swap the `image:` tag to target a different ROS distro (e.g. `universe-humble`).

To enable ROS 2 discovery with other hosts on the network, add `network_mode: host` to the service.

## Launch command reference

```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/home/aw/autoware_map/sample-map-planning \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

`map_path` points inside the container; it maps to `~/autoware_map/sample-map-planning` on the host via the `volumes:` mount.
