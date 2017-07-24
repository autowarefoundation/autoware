# Autoware Docker

## Install docker
```
sudo apt-get install apt-transport-https ca-certificates
sudo apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys 58118E89F3A912897C070ADBF76221572C52609D

# For Ubuntu 14.04
Repo = "deb https://apt.dockerproject.org/repo ubuntu-trusty main"
# For Ubuntu 16.04
Repo = "deb https://apt.dockerproject.org/repo ubuntu-xenial main"

echo $Repo | sudo tee /etc/apt/sources.list.d/docker.list
sudo apt-get update
sudo apt-get install docker-engine
```

## Install NVIDIA Docker
```
wget -P /tmp https://github.com/NVIDIA/nvidia-docker/releases/download/v1.0.0/nvidia-docker_1.0.0-1_amd64.deb
sudo dpkg -i /tmp/nvidia-docker*.deb && rm /tmp/nvidia-docker*.deb
```

## How to build
```
sh build.sh
```

## How to run
```
sh run.sh
```
