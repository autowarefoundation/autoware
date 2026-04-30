# scenario-simulator

Runs a `scenario_simulator_v2` scenario against a live Autoware planning stack. Two services are defined in `docker-compose.yaml`:

- `autoware` — the `ghcr.io/autowarefoundation/autoware:universe-humble` image launching `planning_simulator.launch.xml` with `scenario_simulation:=true` so it pairs with an external scenario runner.
- `scenario-simulator` — the `scenario_test_runner` from `ghcr.io/tier4/scenario_simulator_v2:humble-25.0.17-runtime`, with `launch_autoware:=false` so it only drives the simulator engine.

Both services use `network_mode: host` and share a generated `cyclonedds.xml` (via the `cyclonedds-config` named volume) so ROS 2 discovery works between them. The `scenario-simulator` service waits for the `autoware` service's healthcheck (`/api/operation_mode/state` topic) to pass before launching.

## Prerequisites

- Docker Compose v2
- Sample map extracted to `~/autoware_map/sample-map-planning`
- Sample scenarios cloned to `~/autoware_data/scenarios`

[Download the sample map](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/#download-the-sample-map) and unpack it to `~/autoware_map/sample-map-planning`.

Clone the sample scenarios repository:

```bash
mkdir -p ~/autoware_data/scenarios/
cd ~/autoware_data/scenarios/
git clone https://github.com/autowarefoundation/autoware_sample_scenarios.git
```

## Run

```bash
xhost +local:docker
cd docker/examples/demos/scenario-simulator
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose up
```

`docker compose up` starts both services; the `scenario-simulator` service blocks on the `autoware` healthcheck and then launches the scenario. Logs are interleaved.

To run them in separate terminals (each gets its own stdout/stdin):

```bash
xhost +local:docker

# terminal 1 — Autoware planning stack (rviz needs X11)
cd docker/examples/demos/scenario-simulator
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose run --rm autoware

# terminal 2 — scenario runner
# --no-deps stops compose from starting a second `autoware` container
# (scenario-simulator declares depends_on: autoware); wait until the
# container in terminal 1 is healthy before running this.
cd docker/examples/demos/scenario-simulator
docker compose run --rm --no-deps scenario-simulator
```

`HOST_UID`/`HOST_GID` default to `1000`; drop the prefix if your host UID/GID are 1000.

## After the launch exits

Each `command:` block ends with `exec bash`, so when `ros2 launch` exits the container drops into an interactive shell instead of disappearing. `Ctrl+D` exits and removes the container (`--rm`).

## Customizing

Edit the `command:` block in `docker-compose.yaml` to change launch arguments — for example, point `scenario:=` at a different YAML under `~/autoware_data/scenarios`, or flip `launch_rviz:=true` on the scenario-simulator side to watch the run.

## Launch command reference

`autoware`:

```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/home/aw/autoware_map/sample-map-planning \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  scenario_simulation:=true
```

`scenario-simulator`:

```bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py \
  architecture_type:=awf/universe/20250130 \
  record:=false \
  scenario:=/home/aw/autoware_data/scenarios/autoware_sample_scenarios/sample-scenario.yaml \
  use_custom_centerline:=true \
  launch_rviz:=false \
  launch_autoware:=false
```

The `cyclonedds-config` named volume is populated by the `autoware` service on startup (a copy of `/home/aw/cyclonedds.xml`) and mounted into the scenario-simulator at `/shared-config`, which `CYCLONEDDS_URI` points at.
