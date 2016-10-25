#!/bin/sh

nvidia_version=$(cat /proc/driver/nvidia/version | head -n 1 | awk '{ print $8 }')

# We must use the same driver in the image as on the host
if test ! -f nvidia-driver.run; then
  echo ${nvidia_version}
  nvidia_driver_uri=http://us.download.nvidia.com/XFree86/Linux-x86_64/${nvidia_version}/NVIDIA-Linux-x86_64-${nvidia_version}.run
  wget -O nvidia-driver.run $nvidia_driver_uri
fi

# Build Docker Image
docker build -t autoware-image .
