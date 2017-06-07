#!/bin/sh

# Build Docker Image
nvidia-docker build --no-cache -t autoware-image .
