# basic

Three flavors of "drop me into a shell" container. All three use the `universe-devel-*` images (build tooling plus `/opt/autoware` built from source), mount `~/autoware_map` and `~/autoware_data`, forward `$DISPLAY`, and differ only in how the GPU is exposed.

| Host GPU / driver            | Use          | How                                              |
| ---------------------------- | ------------ | ------------------------------------------------ |
| NVIDIA + proprietary driver  | `dev-nvidia` | `runtime: nvidia` injects proprietary GLX / CUDA |
| NVIDIA + Nouveau open driver | `dev-dri`    | `/dev/dri` passthrough with Mesa nouveau         |
| Intel / AMD                  | `dev-dri`    | `/dev/dri` passthrough with Mesa iris / radeonsi |
| No GPU / headless CI         | `dev-cpu`    | forces Mesa `llvmpipe` software rendering        |

`dev-nvidia` uses `universe-devel-cuda-jazzy`; the other two use `universe-devel-jazzy`.

On NVIDIA hosts running the proprietary driver, `dev-dri` silently falls back to `llvmpipe` (no `nvidia-drm` loader in Mesa) — use `dev-nvidia` instead. Software rendering is fine for text-heavy rviz but laggy on dense point clouds.

Verify you actually got hardware acceleration inside the container:

```bash
glxinfo -B | grep -E "OpenGL (vendor|renderer)"
```

You want to see your GPU (`NVIDIA Corporation` / `AMD Radeon...` / `Mesa Intel...`), not `llvmpipe`.

## Run

```bash
# Allow the container to connect to the host X server
xhost +local:docker

# Pick one
COMPOSE=docker/examples/basic/dev-cpu.compose.yaml
COMPOSE=docker/examples/basic/dev-dri.compose.yaml
COMPOSE=docker/examples/basic/dev-nvidia.compose.yaml

HOST_UID=$(id -u) HOST_GID=$(id -g) \
  docker compose -f "$COMPOSE" run --rm autoware
```

`HOST_UID` / `HOST_GID` are forwarded so the entrypoint can remap the in-container `aw` user to match your host UID/GID; without them, files created in mounted volumes would be owned by the wrong user. The compose files default both to `1000` — you can drop the prefix if your host UID/GID are 1000.

Inside the container, `/opt/autoware` holds the prebuilt install space. The devel images don't source it automatically — you do that yourself before running ROS 2 commands:

```bash
source /opt/autoware/setup.bash
ros2 doctor   # quick smoke test: RMW, topics, node discovery
```

See `demos/` for pre-configured launches (planning-simulator, awsim).

## Attaching a second terminal

`docker compose run` creates a one-off container that disappears when you exit. To open more shells in the same container, start it detached with `up` instead:

```bash
# Terminal 1 — start the container in the background
HOST_UID=$(id -u) HOST_GID=$(id -g) \
  docker compose -f "$COMPOSE" up -d

# Terminal 2, 3, ... — attach a new bash shell
docker compose -f "$COMPOSE" exec autoware /docker-entrypoint.sh bash

# Any terminal — tear it down when you're done
docker compose -f "$COMPOSE" down
```

Running `bash` through `/docker-entrypoint.sh` is what `docker compose up` does for you on the first shell. It drops to the `aw` user and sources `/opt/ros/$ROS_DISTRO/setup.bash` before handing you the prompt. Without it, `docker exec` lands you as `root` with no ROS environment.
