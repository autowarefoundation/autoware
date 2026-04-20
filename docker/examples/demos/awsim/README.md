# awsim

Bridges autoware to the [AWSIM](https://tier4.github.io/AWSIM/) Unity simulator over `network_mode: host` and launches `e2e_simulator.launch.xml`. AWSIM itself runs on the host (or elsewhere on the LAN); this container is the autoware side of the bridge.

## Prerequisites

- NVIDIA GPU with the NVIDIA Container Toolkit installed
- Shinjuku map extracted to `~/autoware_map/Shinjuku-Map/map`
- Perception model data under `~/autoware_data`
- AWSIM running on the host and reachable on the ROS 2 graph
- Docker Compose v2

## Run

```bash
xhost +local:docker

cd docker-new/examples/demos/awsim
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose run --rm awsim
```

When the launch exits the container drops into `bash` (from `exec bash` at the end of `command`) so you can inspect state. `Ctrl+D` exits and removes the container (`--rm`).

The compose file defaults `HOST_UID`/`HOST_GID` to `1000`; drop the prefix if your host UID/GID are 1000.

## Customizing

Edit the `command:` block in `docker-compose.yaml` to change launch arguments (vehicle model, sensor kit, map path). The service uses `universe-cuda-jazzy`; swap the `image:` tag to target a different ROS distro (e.g. `universe-cuda-humble`) or point at a locally built image.

`network_mode: host` is required so AWSIM's DDS traffic reaches the container; removing it will break the connection.

## Launch command reference

The image's entrypoint sources `/opt/autoware/setup.bash` (via `AUTOWARE_RUNTIME=1`), then the service launches:

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  vehicle_model:=sample_vehicle \
  sensor_model:=awsim_sensor_kit \
  map_path:=/home/aw/autoware_map/Shinjuku-Map/map
```

`map_path` points inside the container; it maps to `~/autoware_map/Shinjuku-Map/map` on the host via the `volumes:` mount.
