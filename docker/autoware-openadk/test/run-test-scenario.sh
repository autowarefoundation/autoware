#!/usr/bin/env bash
set -e

# download test artifacts from google drive
if [ ! -d ~/autoware_artifacts/test-artifacts ]; then
    echo "Downloading test artifacts from google drive..."
    gdown -O ~/autoware_artifacts/ 'https://drive.google.com/uc?export=download&id=1gTCWcye4JxqR7rSxl2b-iiLqgya6ShBR'
    unzip -d ~/autoware_artifacts ~/autoware_artifacts/test-artifacts.zip
    rm ~/autoware_artifacts/test-artifacts.zip
else
    echo "Test artifacts are already downloaded."
fi

# enable xhost
xhost +

# start containers
echo "Starting containers..."
docker compose -f test-scenario.docker-compose.yaml up
