# Run Autoware in Docker

## Build

```bash
cd autoware

# Just the base
docker build \
  --build-arg ROS_DISTRO=jazzy \
  --build-arg RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  --target core-base \
  -t autoware-core-base:jazzy \
  -f docker-new/core.Dockerfile \
  .

# Base + rosdep
docker build \
  --build-arg ROS_DISTRO=jazzy \
  --target core-rosdep \
  -t autoware-core-rosdep:jazzy \
  -f docker-new/core.Dockerfile \
  .
```

## Usage

```bash
docker run --rm -it \
  --net host \
  -e HOST_UID=$(id -u) \
  -e HOST_GID=$(id -g) \
  -v $HOME/projects/autoware:/home/aw/autoware \
  -w /home/aw/autoware \
  autoware-core-rosdep:jazzy
```

Or run without volume mounting. But beware, the `autoware` folder is not present in the container.

```bash
docker run --rm -it \
  --net host \
  autoware-core-base:jazzy
```
