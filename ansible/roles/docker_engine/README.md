# docker_engine

This role installs [Docker Engine](https://docs.docker.com/engine/) following the [Official Installation Guide](https://docs.docker.com/engine/install/ubuntu/).
Then it sets up execution from non-root users following the [Official Post Installation Steps](https://docs.docker.com/engine/install/linux-postinstall).

## Inputs

None.

## Manual installation (Recommended)

> ℹ️ The steps below may differ from the role implementation.  
> They reflect the **most up-to-date and preferred procedure** for manual installation.  
> The role will be updated to align with these steps.

### Install Docker Engine

```bash
# Taken from: https://docs.docker.com/engine/install/ubuntu/
# And: https://docs.docker.com/engine/install/linux-postinstall

# Uninstall old versions
sudo apt remove $(dpkg --get-selections docker.io docker-compose docker-compose-v2 docker-doc podman-docker containerd runc | cut -f1)

# Add Docker's official GPG key:
sudo apt update
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Verify that Docker Engine is installed correctly by running the hello-world image.
sudo docker run hello-world
# Note: This command downloads a test image and runs it in a container. When the container runs, it prints a message and exits.
```

### Post-Installation setup (non-root usage)

```bash
# Post-installation steps for Linux

# Create the docker group.
sudo groupadd docker

# Add your user to the docker group.
sudo usermod -aG docker $USER

# Log out and log back in so that your group membership is re-evaluated.

# Verify that you can run docker commands without sudo
docker run hello-world
# Note: This command downloads a test image and runs it in a container. When the container runs, it prints a message and exits.
```
