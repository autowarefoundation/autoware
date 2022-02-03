# pre_commit

This role installs Docker environment following [this page](https://docs.docker.com/engine/install/ubuntu/) and sets up rootless execution following [this page](https://docs.docker.com/engine/install/linux-postinstall/).

Also, it installs some additional tools:

- [Docker Compose](https://github.com/docker/compose)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
- [rocker](https://github.com/osrf/rocker)

## Inputs

| Name                   | Required | Description                    |
| ---------------------- | -------- | ------------------------------ |
| docker_compose_version | false    | The version of Docker Compose. |
