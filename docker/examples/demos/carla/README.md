# CARLA

Runs the [CARLA](https://carla.readthedocs.io/en/0.9.16/) simulator and the autoware-side bridge in a single Docker Compose stack, split into four services:

- **`carla-simulator`** — `carlasim/carla:0.9.16`, runs `CarlaUE4.sh` and listens on `localhost:2000`.
- **`carla-interface`** — `autoware:universe-cuda-jazzy`, runs `autoware_carla_interface.launch.xml` standalone. This is the bridge that connects to CARLA on port 2000 and republishes sensor data onto the autoware ROS graph. Its healthcheck passes once the ego vehicle exists in the current CARLA episode.
- **`spectator-follow`** — `autoware:universe-cuda-jazzy`, runs `ros2 run autoware_carla_interface spectator_follow` so the CARLA spectator camera chases the ego vehicle instead of staying where the simulator window started.
- **`autoware`** — `autoware:universe-cuda-jazzy`, launches `e2e_simulator.launch.xml` with `simulator_type:=carla` but `launch_simulator_interface:=false`, since the interface runs in its own container.

All four services use `network_mode: host` so the interface can reach the CARLA server on `localhost:2000` and DDS traffic flows between the interface and autoware nodes.

The autoware image does not ship the CARLA Python API, so the two services that talk to the CARLA server directly (`carla-interface` and `spectator-follow`) fetch the matching wheel on first run. That snippet lives in the `x-carla-python-api` YAML anchor at the top of the compose file, which is where the CARLA version is pinned.

Splitting the interface out makes it easier to restart just the bridge (e.g. when CARLA changes maps) without tearing down the autoware stack.

## Prerequisites

- NVIDIA GPU with the NVIDIA Container Toolkit installed
- CARLA Lanelet2 map (e.g. `Town01`) extracted to `~/autoware_data/maps/Town01` — see the [autoware_carla_interface README](https://autowarefoundation.github.io/autoware_universe/main/simulator/autoware_carla_interface/#map-setup) for the expected layout (`lanelet2_map.osm`, `pointcloud_map.pcd`, `map_projector_info.yaml`)
- Perception model data under `~/autoware_data/ml_models`
- Docker Compose v2

## Run

```bash
xhost +local:docker

cd docker/examples/demos/carla
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose up
```

Or, to drop into a shell on the autoware side after the launch exits:

```bash
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose run --rm autoware
```

`docker compose run` starts the full dependency chain automatically: `carla-simulator` (healthcheck: a TCP probe of port `2000`), then `carla-interface` (healthcheck: the ego vehicle exists in the episode), so autoware launches only after the bridge is healthy. Note that `docker compose run autoware` only pulls in autoware's own dependencies — add `spectator-follow` explicitly (`docker compose up -d spectator-follow`) if you want the chase camera in that flow.

To run everything except the chase camera and keep manual control of the CARLA spectator:

```bash
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose up --scale spectator-follow=0
```

To stop everything:

```bash
docker compose down
```

The compose file defaults `HOST_UID`/`HOST_GID` to `1000`; drop the prefix if your host UID/GID are 1000.

Once both sides are up: set the initial pose (Init by GNSS) in RViz, set a goal, wait for planning, then engage.

## Customizing

Edit the `command:` block in `docker-compose.yaml` to change launch arguments.

**`carla-simulator`**:

- **Headless / no rendering**: append `-RenderOffScreen` to the `command:` list to disable the spectator window.
- **Higher quality**: change `-quality-level=Low` to `-quality-level=Epic` (uses more GPU).

**`carla-interface`**:

- **Map**: change `map_path` (e.g. `Town10HD`) — the basename is forwarded to CARLA as the world to load. Keep this in sync with the `map_path` on the `autoware` service.
- **Light-weight sensors**: set `use_light_weight_sensor_mapping:=true` to use a single front camera at lower frequencies (lower GPU load).
- **Timeout**: `timeout:=60` is the CARLA client connect timeout in seconds.

**`spectator-follow`**:

- **Top-down view**: use `--distance 0 --height 30 --pitch -90`.
- **Chase distance / height / pitch**: `--distance 8.0`, `--height 4.0`, `--pitch -15.0` position the camera behind and above the ego.
- **Ego actor**: `--role ego_vehicle` must match `ego_vehicle_role_name` on the interface.
- **Update rate**: `--rate 30.0` Hz. Lower it if the extra RPC traffic bothers the server.

It waits for `carla-interface` to report healthy before connecting.

The camera has no effect if CARLA runs headless (`-RenderOffScreen`).

**`autoware`**:

- **Map**: change `map_path` to match the interface.
- **E2E planning (VAD)**: add `use_e2e_planning:=true`.

The autoware, interface, and spectator services all use `universe-cuda-jazzy`; swap the `image:` tag to target a different ROS distro (e.g. `universe-cuda-humble`) or point at a locally built image. Keep them in sync — they need to be ABI-compatible on the ROS graph.

`network_mode: host` is required on all four services; removing it will break the bridge.

## Launch command reference

The autoware image's entrypoint sources `/opt/autoware/setup.bash` (via `AUTOWARE_RUNTIME=1`).

`carla-interface` runs:

```bash
ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml \
  map_path:=/home/aw/autoware_data/maps/Town01 \
  port:=2000 \
  timeout:=60 \
  use_light_weight_sensor_mapping:=true
```

`spectator-follow` runs:

```bash
ros2 run autoware_carla_interface spectator_follow \
  --host localhost \
  --port 2000 \
  --timeout 60 \
  --role ego_vehicle \
  --distance 8.0 \
  --height 4.0 \
  --pitch -15.0 \
  --rate 30.0
```

See the [autoware_carla_interface README](https://autowarefoundation.github.io/autoware_universe/main/simulator/autoware_carla_interface/#following-the-ego-vehicle-with-the-carla-spectator-camera) for the full flag list.

`autoware` runs:

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  simulator_type:=carla \
  launch_simulator_interface:=false \
  map_path:=/home/aw/autoware_data/maps/Town01
```

`launch_simulator_interface:=false` prevents the e2e launch from also starting `autoware_carla_interface` — that runs in its own container now.

`map_path` points inside the container; it maps to `~/autoware_data/maps/Town01` on the host via the `volumes:` mount. The map directory name (`Town01`) is parsed out and used as the CARLA world name, so it must match an existing CARLA map.

`carla-simulator` runs:

```bash
./CarlaUE4.sh -prefernvidia -quality-level=Low -nosound
```
