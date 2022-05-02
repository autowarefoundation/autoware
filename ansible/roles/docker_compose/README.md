# docker_compose

This role installs Docker environment following [this page](https://docs.docker.com/engine/install/ubuntu/) and sets up rootless execution following [this page](https://docs.docker.com/engine/install/linux-postinstall/).

Also, it installs some additional tools:

- [Docker Compose](https://github.com/docker/compose) following the [installation guide](https://github.com/docker/compose#linux).

## Inputs

| Name                   | Required | Description                    |
| ---------------------- | -------- | ------------------------------ |
| docker_compose_version | false    | The version of Docker Compose. |

## Manual Installation

Install Docker Compose:

The `docker_compose_version` can also be found in:
[./defaults/main.yaml](./defaults/main.yaml)

```bash
# Modified from: https://docs.docker.com/compose/cli-command/#install-on-linux

# Run this command to download the Docker Compose:
docker_compose_version=v2.5.0
sudo curl -SL https://github.com/docker/compose/releases/download/${docker_compose_version}/docker-compose-$(uname -s)-$(uname -m) -o /usr/local/bin/docker-compose

# Apply executable permissions to the binary:
sudo chmod +x /usr/local/bin/docker-compose

# Test the installation.
docker-compose --version
```
