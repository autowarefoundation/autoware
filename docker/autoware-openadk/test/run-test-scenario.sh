#!/usr/bin/env bash

set -e

xhost +
docker compose -f docker-compose.yaml up
