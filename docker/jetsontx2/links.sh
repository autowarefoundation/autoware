#!/bin/bash

# Add links for GPU usage
sudo ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvidia-ptxjitcompiler.so.28.2.1 /usr/lib/aarch64-linux-gnu/tegra/libnvidia-ptxjitcompiler.so
sudo ln -s /usr/lib/aarch64-linux-gnu/libcuda.so /usr/lib/aarch64-linux-gnu/libcuda.so.1
sudo ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvll.so /usr/lib/aarch64-linux-gnu/
sudo ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvrm.so /usr/lib/aarch64-linux-gnu/
sudo ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvrm_graphics.so /usr/lib/aarch64-linux-gnu/
sudo ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvdc.so /usr/lib/aarch64-linux-gnu/
sudo ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvos.so /usr/lib/aarch64-linux-gnu/
sudo ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvimp.so /usr/lib/aarch64-linux-gnu/
sudo ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvrm_gpu.so /usr/lib/aarch64-linux-gnu/
sudo ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvidia-fatbinaryloader.so.28.2.1 /usr/lib/aarch64-linux-gnu/
sudo ln -sf /usr/lib/aarch64-linux-gnu/tegra/libGL.so /usr/lib/aarch64-linux-gnu/libGL.so
sudo ln -s /usr/lib/aarch64-linux-gnu/tegra/libGLX.so.0 /usr/lib/aarch64-linux-gnu/libGLX.so.0
sudo ln -s /usr/lib/aarch64-linux-gnu/tegra/libGLdispatch.so.0 /usr/lib/aarch64-linux-gnu/libGLdispatch.so.0
sudo ln -s /usr/lib/aarch64-linux-gnu/tegra/libGL.so.1 /usr/lib/aarch64-linux-gnu/libGL.so.1
