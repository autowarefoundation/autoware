# Setting up a new PC for the Mach-E

These steps reproduce the lab Mach-E PC (`rancslab-MIC-770V3W`) as of 2026-10-19. Every repo in `repositories/autoware.repos` is pinned to the exact commit that PC runs, and everything that used to live only on that PC is now in this folder:

| In this folder | Installed to | What it is |
|---|---|---|
| `bringup/` | `~/utils/mach_e_bringup` (symlink) | `run_mach_e.sh` (starts ADMA + Autoware), RViz GPU guard, drive verification watchers, calibration tools |
| `env/` | `~/utils/autoware_env_setup` (symlink) | `aw.sh` (ROS + CycloneDDS + acados environment) and `cyclonedds.xml` |
| `patches/autoware_core.patch` + `apply_patches.sh` | `src/core/autoware_core` | **EKF MRM-flicker fix** (and route_handler `yaw_threshold`). Upstream `autoware_core` has no lab fork, so the fix is kept as a patch |
| `bringup/display/nvidia-application-profiles-rc` | `~/.nv/` | **RViz GPU leak fix**. Without it RViz fills the GPU in ~5 min and the lidar pipeline dies |
| `system/10-cyclone-max.conf` | `/etc/sysctl.d/` | CycloneDDS socket buffers |
| `system/lo-multicast.service` | `/etc/systemd/system/` | Multicast on `lo` (DDS is bound to `lo`) |
| `system/ptp4l-ouster.{conf,service}` | `/etc/linuxptp/`, `/etc/systemd/system/` | PTP time master for the Ouster (software timestamps) |
| `system/setup_network.sh` | NetworkManager | The two fixed-IP sensor network profiles |
| `docs/autoware_mach_e_install.md` | | The detailed install guide, with troubleshooting |

Allow about 2 hours: about 40 min for the build, plus the driver install and a reboot.

---

## 1. Requirements

- Ubuntu 22.04, an NVIDIA GPU (the lab PC has an RTX A4000 on driver 615), about 60 GB of free disk space
- **Two Ethernet ports**: one to the lidar/ADMA switch, one cabled straight to the LUCID camera
- A GitHub account in the `rancslab` org with an SSH key or a PAT. `rancslab/dbw_ros` is **private**, see `docs/autoware_mach_e_install.md` Section 3
- The repo **must** be cloned to `~/autoware.mach_e` (`aw.sh` and `run_mach_e.sh` use that path)

## 2. Clone and install ROS 2, CUDA and TensorRT

```bash
sudo apt update && sudo apt install -y git python3-vcs2l
git clone https://github.com/rancslab/autoware.mach_e.git ~/autoware.mach_e
cd ~/autoware.mach_e
./setup-dev-env.sh --download-artifacts      # ROS 2 Humble, CUDA 12.8, TensorRT, ML models -> ~/autoware_data
sudo reboot                                  # then: nvidia-smi shows driver >= 570
```
If the script fails at `Install cuda-drivers` (Ubuntu's own NVIDIA driver conflicts), see `docs/autoware_mach_e_install.md` Section 5.

## 3. Import the source repos and apply the lab patch

One import, with no overlay file. `autoware.repos` already points at the rancslab forks.
```bash
cd ~/autoware.mach_e
mkdir -p src && vcs import src < repositories/autoware.repos
mach_e_setup/apply_patches.sh                      # REQUIRED: EKF fix into src/core/autoware_core
vcs export --exact src | grep -B2 -A1 rancslab     # 6 repos must come from github.com/rancslab
```
⚠️ **Don't skip `apply_patches.sh`.** Without it the build still succeeds, but the MRM emergency stop flickers on the car again. Re-run it after any `vcs import` that re-checks out `autoware_core`; it skips a patch that is already applied.

The lab forks used are `autoware_universe`, `autoware_launch`, `dbw_ros`, `nebula`, `lucid_vision_driver` and `adma_ros_driver`.

## 4. Install the LUCID ArenaSDK (camera driver dependency)

Follow `docs/autoware_mach_e_install.md` Section 7. The lab copy of the SDK is on the lab PC at `~/autoware_logs/ArenaSDK_AVMP_v_1.0.8.60_Linux_x64.tar.gz`. Do this **before** building.

## 5. Dependencies and build

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/opt/acados:$CMAKE_PREFIX_PATH ACADOS_SOURCE_DIR=/opt/acados LD_LIBRARY_PATH=/opt/acados/lib:$LD_LIBRARY_PATH
cd ~/autoware.mach_e
sudo rosdep init 2>/dev/null; rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro humble \
  --skip-keys "sample_sensor_kit_description sample_vehicle_description sample_sensor_kit_launch"
MAKEFLAGS=-j4 colcon build --symlink-install --parallel-workers 12 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 6. Install the PC configuration

```bash
cd ~/autoware.mach_e/mach_e_setup
ip -br link                                   # find the two Ethernet interface names
SENSOR_IF=<lidar/ADMA NIC> ./install.sh --system
system/setup_network.sh <lidar/ADMA NIC> <camera NIC>
```
`install.sh` sets up the `~/utils` symlinks, the `aw` alias, the NVIDIA profile, sysctl, `lo` multicast and PTP. It is safe to re-run. Without `--system` it only does the user-level parts (no sudo).

| Device | Address | Notes |
|---|---|---|
| This PC, sensor NIC | 192.168.88.100/24 + 169.254.6.1/16, MTU 9000 | **Must be .100**: the ADMA sends to 192.168.88.100:4444 |
| This PC, camera NIC | 192.168.88.101/24, metric 200, MTU 9000, rx ring 4096 | Host routes to the camera (.107, old .126) |
| Ouster OS1-128 | 192.168.88.37 | PTP time from this PC |
| GeneSys ADMA | 192.168.88.30 | |
| LUCID camera, SN 262700247 | 192.168.88.107 | The driver finds it by serial number |
| Dataspeed USB CAN tool | USB | DBW2 |

## 7. Copy the maps

Maps are not in git. Copy them from the lab PC, for example:
```bash
rsync -a rancslab@<lab-pc>:autoware_map/PD2LAB ~/autoware_map/     # 362 MB; same for USF_map etc.
```
Each map folder needs `lanelet2_map.osm`, `pointcloud_map.pcd` and `map_projector_info.yaml`.

## 8. Check and run

```bash
sysctl net.core.rmem_max                       # 2147483647
systemctl is-active ptp4l-ouster lo-multicast  # active active
ls ~/.nv/nvidia-application-profiles-rc

~/utils/mach_e_bringup/run_mach_e.sh PD2LAB    # starts ADMA + Autoware + RViz
~/utils/mach_e_bringup/check_topics.sh         # second terminal: every sensor topic should have a rate
```
- **The first launch builds the TensorRT engines (~15 min).** They are cached in `~/autoware_data/ml_models` and are specific to the GPU.
- Nothing moves until someone presses **Engage**.
- Lidar/IMU extrinsics and the vehicle description come from the pinned `autoware_launch` commit. Re-check the calibration if the sensors were remounted.
- Planning simulator smoke test without hardware: `docs/autoware_mach_e_install.md` Section 12.1.

---

## Keeping the pins up to date

The pins are exact commits, so a fresh install never picks up untested changes. After pushing a change to one of the lab forks:
```bash
cd ~/autoware.mach_e
vcs export --exact src > /tmp/exact.repos      # copy the new hash for that repo into repositories/autoware.repos
```
Then commit `autoware.repos`, keeping the `# branch ...` comment.

To change the `autoware_core` fix, edit it under `src/core/autoware_core`, then regenerate the patch with `git -C src/core/autoware_core diff > mach_e_setup/patches/autoware_core.patch`. If `autoware_core` moves to a new version in `autoware.repos`, check that `apply_patches.sh` still applies it.

To edit the scripts on the lab PC, edit them in place: `~/utils/mach_e_bringup` is a symlink into this repo. Then commit as usual.

Not needed for Autoware: `~/ros2_ws` (ouster-ros, LIO-SAM) is only for building maps.
