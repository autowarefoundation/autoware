# Run Autoware in Docker

## Build

```bash
cd autoware

docker build \
  --build-arg ROS_DISTRO=jazzy \
  --build-arg RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -t autoware-core-base:jazzy \
  -f docker-new/core-base.Dockerfile \
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
  autoware-core-base:jazzy
```

Or run without volume mounting. But beware, the `autoware` folder is not present in the container.

```bash
docker run --rm -it \
  --net host \
  autoware-core-base:jazzy
```
