# pre_commit

This role installs Docker environment following [this page](https://docs.docker.com/engine/install/ubuntu/) and sets up rootless execution following [this page](https://docs.docker.com/engine/install/linux-postinstall/).

Also, it installs [rocker](https://github.com/osrf/rocker) for easy NVIDIA support.

## Inputs

| Name                   | Required | Description                    |
| ---------------------- | -------- | ------------------------------ |
| docker_compose_version | false    | The version of Docker Compose. |
