# Installing Autoware (RANCS Lab Mach-E) on a Lab PC

This guide installs the RANCS Lab Autoware stack for the Ford Mach-E. It builds natively from source on ROS 2 Humble and uses every RANCS Lab Autoware fork from <https://github.com/rancslab>. The folder layout follows the lab file standard: [rancs-file-standard-linux.md](https://gist.github.com/SoupyzInc/718b28ae588b04216867dea531e019f9).

> Written 2026-10-06 against `rancslab/autoware.mach_e@main` (based on Autoware Universe 0.51.0).

> **Update 2026-10-19:** for a new PC, follow [`../NEW_PC_SETUP.md`](../NEW_PC_SETUP.md) first. `repositories/autoware.repos` now points at every rancslab fork with exact commits, and the EKF fix for upstream `autoware_core` is applied by `mach_e_setup/apply_patches.sh` after the import, so the `rancslab.repos` overlay in Section 6 is no longer needed. The branch table in Section 0 is out of date: the PC runs `autoware_launch@dev`, `lucid_vision_driver@dev` and `adma_ros_driver@dev`. Sections 10 and 12.2 are replaced by `mach_e_setup/install.sh` and `run_mach_e.sh`. Sections 5, 7, 8, 9 and the troubleshooting appendix still apply.
> If the forks have moved on since, re-check the branch table in [Section 0](#0-which-repos-get-installed).

---

## Contents

0. [Which repos get installed](#0-which-repos-get-installed)
1. [System requirements](#1-system-requirements)
2. [Create the lab directory layout](#2-create-the-lab-directory-layout)
3. [Prerequisites and GitHub access](#3-prerequisites-and-github-access)
4. [Clone the Autoware meta-repo](#4-clone-the-autoware-meta-repo)
5. [Install the development environment (ROS 2, CUDA, TensorRT)](#5-install-the-development-environment-ros-2-cuda-tensorrt)
6. [Import the source repos (RANCS Lab forks first)](#6-import-the-source-repos-rancs-lab-forks-first)
7. [Install the LUCID ArenaSDK (camera)](#7-install-the-lucid-arenasdk-camera)
8. [Install ROS dependencies](#8-install-ros-dependencies)
9. [Build](#9-build)
10. [Environment script (`~/utils/autoware_env_setup`)](#10-environment-script-utilsautoware_env_setup)
11. [Maps and ROS bags](#11-maps-and-ros-bags)
12. [Run Autoware](#12-run-autoware)
13. [Updating](#13-updating)
14. [Appendix A: all RANCS Lab repos](#appendix-a-all-rancs-lab-repos)
15. [Appendix B: troubleshooting](#appendix-b-troubleshooting)

---

## 0. Which repos get installed

Upstream Autoware is split across many repos. The RANCS Lab forks below **replace** their upstream counterparts:

| RANCS Lab repo | Forked from | Branch | What it provides |
|---|---|---|---|
| [`autoware.mach_e`](https://github.com/rancslab/autoware.mach_e) | autowarefoundation/autoware | `main` | Meta-repo: ansible setup scripts, `repositories/autoware.repos`, `fake_vehicle_status.py` |
| [`autoware_universe`](https://github.com/rancslab/autoware_universe) | autowarefoundation/autoware_universe | **`mach-e`** | Universe 0.51.0 plus Mach-E perception changes: centerpoint on `/ouster/points`, bytetrack camera_info relay, ring outlier filter fix |
| [`autoware_launch`](https://github.com/rancslab/autoware_launch) | autowarefoundation/autoware_launch | `main` | Launch files and configs, **`mach_e_sensor_kit_launch`**, **`mach_e_vehicle_launch`**, localization and NDT tuning, LUCID camera and camera-LiDAR fusion |
| [`nebula`](https://github.com/rancslab/nebula) | ub-cavas/nebula | **`add-ouster-support`** | LiDAR driver with Ouster OS-64 support, plus lab fixes for host-time stamping and the y-axis flip |
| [`lucid_vision_driver`](https://github.com/rancslab/lucid_vision_driver) | autowarefoundation/lucid_vision_driver | `main` | LUCID Triton GigE camera driver (camera0). Requires the ArenaSDK |
| [`adma_ros_driver`](https://github.com/rancslab/adma_ros_driver) | GeneSysElektronik/adma_ros_driver | `ros2_master` | GeneSys ADMA GNSS/INS driver. The sensor kit reads `/genesys/adma/fix` and `/genesys/adma/imu` |
| `dbw_ros` (`rancslab/dbw_ros`) | Dataspeed dbw_ros | `ros2` | Dataspeed DBW2 drive-by-wire: `ds_dbw_autoware` vehicle interface and `ds_dbw_can`. **Private repo, so you need lab GitHub access** |

Everything else (core messages, `autoware_core`, `autoware_utils` and so on) comes from the upstream Autoware Foundation versions pinned in `autoware.mach_e/repositories/autoware.repos`.

> **Why an overlay file?** `autoware.mach_e/repositories/autoware.repos` still points at upstream `autowarefoundation/autoware_universe@0.51.0` and `ub-cavas/nebula`, and it doesn't list `adma_ros_driver`. [Section 6](#6-import-the-source-repos-rancs-lab-forks-first) uses a small `rancslab.repos` overlay so that **every** forked package comes from `github.com/rancslab`. The long-term fix is to commit those URL changes to `autoware.mach_e`'s `autoware.repos`.

---

## 1. System requirements

| | Required / recommended | This PC (`rancslab-MIC-770V3W`) |
|---|---|---|
| OS | Ubuntu 22.04 (ROS 2 Humble) | Ubuntu 22.04.5 LTS ✅ |
| CPU / RAM | 8+ cores, 32 GB+ RAM | 24 cores, 62 GB ✅ |
| Disk | ~60 GB free (sources, build, CUDA, ML models) | 123 GB free ✅ |
| GPU | NVIDIA, driver **≥ 570** for CUDA 12.8 | RTX A4000, driver **550** ⚠️ (Section 5 upgrades it) |

---

## 2. Create the lab directory layout

The lab standard requires these folders in `$HOME`:

```bash
mkdir -p ~/autoware_map ~/documentation ~/source ~/rosbags ~/utils
```

| Folder | Contents |
|---|---|
| `~/autoware.mach_e/` | Autoware itself: sources, build and install. Created in Section 4 |
| `~/autoware_map/` | One folder per map |
| `~/documentation/` | Docs like this file |
| `~/source/` | Non-Autoware projects, each in its own folder (e.g. the ArenaSDK, LIO-SAM) |
| `~/rosbags/` | Bags, organised by project, each project folder with a `README.md` |
| `~/utils/` | Scripts and environment configs, each in its own folder with a `README.md` |

---

## 3. Prerequisites and GitHub access

```bash
sudo apt update
sudo apt install -y git curl python3-pip

# vcs (multi-repo import tool). Autoware now uses vcs2l; vcstool works the same way.
sudo apt install -y python3-vcs2l || pip3 install --user vcstool
```

### Access to the private `dbw_ros` repo

`rancslab/dbw_ros` is private. You need a GitHub account that is a member of the `rancslab` organisation, plus **one** of these:

- **HTTPS + personal access token (PAT):** create a token at GitHub → Settings → Developer settings → Tokens with `repo` scope. Then make git remember it:
  ```bash
  git config --global credential.helper store   # stores the token in ~/.git-credentials
  git ls-remote https://github.com/rancslab/dbw_ros.git   # enter username + PAT once
  ```
- **SSH key:** add `~/.ssh/id_ed25519.pub` to your GitHub account, then change the `dbw_ros` URL in the overlay (Section 6) to `git@github.com:rancslab/dbw_ros.git`.

---

## 4. Clone the Autoware meta-repo

The lab standard names this folder `autoware.your_vehicle`, so for the Mach-E it is `~/autoware.mach_e`.

```bash
git clone https://github.com/rancslab/autoware.mach_e.git ~/autoware.mach_e
cd ~/autoware.mach_e
```

---

## 5. Install the development environment (ROS 2, CUDA, TensorRT)

Autoware's ansible-based setup script installs:
- ROS 2 Humble desktop
- colcon and rosdep
- RMW (CycloneDDS) settings
- the NVIDIA driver, CUDA 12.8, cuDNN and TensorRT 10.8
- `gdown`, `geographiclib`, the `acados`/`spconv` dependencies and so on
- the perception ML model artifacts, downloaded into `~/autoware_data/ml_models`

```bash
cd ~/autoware.mach_e
./setup-dev-env.sh --download-artifacts
```

> **Unattended install (for example by Claude Code):** `./setup-dev-env.sh -y --download-artifacts` only skips its sudo password prompt if sudo doesn't ask for one. Grant passwordless sudo first. It has **no expiry**, so revoke it when the install is done:
> ```bash
> sudo bash ~/utils/temp_nopasswd_sudo/temp_nopasswd_sudo.sh            # grant until revoked
> sudo bash ~/utils/temp_nopasswd_sudo/temp_nopasswd_sudo.sh --revoke
> ```
> See `~/utils/temp_nopasswd_sudo/README.md`.

- The script asks for your sudo password (ansible's "BECOME" prompt) and asks you to confirm the NVIDIA install. Answer **yes**.
- ⚠️ **If Ubuntu's own NVIDIA driver is installed** (`dpkg -l | grep nvidia-driver-5` shows something like `nvidia-driver-550`), the script **fails** at `cuda : Install cuda-drivers` with `pkgProblemResolver::Resolve generated breaks`. The cause: NVIDIA's `cuda-drivers` / `nvidia-open` packages conflict with Ubuntu's `libnvidia-*-550` packages. On this PC, the fix was to swap the drivers in one apt transaction and then rerun the script:
  ```bash
  R550=$(dpkg -l | awk '/^ii/ && $2 ~ /550/ {sub(/:amd64|:i386/,"",$2); print $2"-"}' | sort -u | tr '\n' ' ')
  apt-get -s install cuda-drivers $R550          # simulate first; it should only remove *-550 + nvidia-prime
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y cuda-drivers $R550
  ./setup-dev-env.sh -y --download-artifacts     # rerun; finished tasks are skipped
  ```
  This installs NVIDIA driver **615 (open kernel modules)** through DKMS. Check `mokutil --sb-state`: if Secure Boot is enabled, the reboot will ask you to enroll the DKMS signing key (MOK) at the console.
- **NVIDIA driver:** this PC had driver 550, but CUDA 12.8 needs ≥ 570. Let the script (or the swap above) upgrade it, then **reboot**. Until you reboot, `nvidia-smi` prints `Driver/library version mismatch`. That is harmless for building.
  ```bash
  sudo reboot
  nvidia-smi        # after reboot: driver ≥ 570, "CUDA Version: 12.8" or newer
  ```
- Useful flags: `-y` (non-interactive), `--no-nvidia` (skip CUDA/TensorRT, for a CPU-only machine), `--no-cuda-drivers` (install CUDA but keep the current driver, which **won't work here** because 550 is too old).
- If the download stage fails, rerun it with `./setup-dev-env.sh --download-artifacts` (it is idempotent).

---

## 6. Import the source repos (RANCS Lab forks first)

### 6.1 Create the RANCS Lab overlay

Save this as `~/autoware.mach_e/repositories/rancslab.repos`. The paths match the ones in `autoware.repos`, so these clones take the place of the upstream ones.

```yaml
repositories:
  # universe (RANCS Lab Mach-E branch, based on 0.51.0)
  universe/autoware_universe:
    type: git
    url: https://github.com/rancslab/autoware_universe.git
    version: mach-e

  # launcher (contains mach_e_sensor_kit_launch + mach_e_vehicle_launch)
  launcher/autoware_launch:
    type: git
    url: https://github.com/rancslab/autoware_launch.git
    version: main

  # vehicle: Dataspeed DBW2 (PRIVATE - needs rancslab GitHub access)
  vehicle/external/dbw_ros:
    type: git
    url: https://github.com/rancslab/dbw_ros.git
    version: ros2

  # sensors
  sensor_component/external/nebula:
    type: git
    url: https://github.com/rancslab/nebula.git    # lab copy (NOT ub-cavas/nebula)
    version: add-ouster-support
  sensor_component/external/lucid_vision_driver:
    type: git
    url: https://github.com/rancslab/lucid_vision_driver.git
    version: main
  sensor_component/external/adma_ros_driver:
    type: git
    url: https://github.com/rancslab/adma_ros_driver.git
    version: ros2_master
```

### 6.2 Import: overlay first, then everything else

Import the overlay **first**. Then import the rest of `autoware.repos` with `--skip-existing`. That way the RANCS Lab forks aren't overwritten by upstream URLs (`autoware_universe` and `nebula`), and nothing gets cloned twice.

```bash
cd ~/autoware.mach_e
mkdir -p src
vcs import src < repositories/rancslab.repos
vcs import --skip-existing src < repositories/autoware.repos
```

### 6.3 Check that the forks came from rancslab

```bash
cd ~/autoware.mach_e
for d in universe/autoware_universe launcher/autoware_launch vehicle/external/dbw_ros \
         sensor_component/external/nebula sensor_component/external/lucid_vision_driver \
         sensor_component/external/adma_ros_driver; do
  printf '%-48s %s @ %s\n' "$d" "$(git -C src/$d remote get-url origin)" "$(git -C src/$d rev-parse --abbrev-ref HEAD)"
done
```

All six lines should show `github.com/rancslab/...` on the branch from the table in Section 0. If one shows `autowarefoundation` or `ub-cavas`, delete that folder under `src/` and rerun `vcs import src < repositories/rancslab.repos`.

> **No GitHub access to `dbw_ros` yet?** Both `.repos` files list it, so import copies with that entry removed. Set `GIT_TERMINAL_PROMPT=0` so that no clone hangs waiting for a password. This is what was done on this PC, see `~/autoware_logs/autoware_install_log.md` Step 5:
> ```bash
> for f in rancslab autoware; do
>   python3 -c "import yaml,sys; d=yaml.safe_load(open(sys.argv[1])); d['repositories'].pop('vehicle/external/dbw_ros',None); yaml.safe_dump(d,open(sys.argv[2],'w'),sort_keys=False)" \
>     repositories/$f.repos /tmp/$f.nodbw.repos
> done
> export GIT_TERMINAL_PROMPT=0
> vcs import src < /tmp/rancslab.nodbw.repos && vcs import --skip-existing src < /tmp/autoware.nodbw.repos
> ```
> Without `dbw_ros`, run Autoware with `launch_vehicle_interface:=false` (and use `fake_vehicle_status.py` on the bench).

---

## 7. Install the LUCID ArenaSDK (camera)

`lucid_vision_driver` links against LUCID's proprietary ArenaSDK. Its CMake looks the SDK up through `/etc/ld.so.conf.d/Arena_SDK.conf`, so the SDK must be installed **before** building.

The lab copy of the SDK is already on this PC: **`~/autoware_logs/ArenaSDK_AVMP_v_1.0.8.60_Linux_x64.tar.gz`** (ArenaSDK / ArenaView MP 1.0.8.60, Linux x64, about 1.8 GB). On another PC, download "ArenaSDK for Linux x64" from <https://thinklucid.com/downloads-hub/> (needs a free LUCID account). `ArenaSDK_v1.0.80.56.exe` in the same folder is the **Windows** installer and isn't used here.

1. The SDK is supplementary source, so per the lab standard it goes in `~/source/`:
   ```bash
   mkdir -p ~/source/arena_sdk
   tar -xzf ~/autoware_logs/ArenaSDK_AVMP_v_1.0.8.60_Linux_x64.tar.gz -C ~/source/arena_sdk
   # -> ~/source/arena_sdk/ArenaSDK_Linux_x64/  (+ README_AVMP)
   ```
2. Run the SDK's config script **from inside the SDK folder**. It uses `$PWD` to write the library paths, so running it from anywhere else registers the wrong directory:
   ```bash
   cd ~/source/arena_sdk/ArenaSDK_Linux_x64
   echo N | sudo sh Arena_SDK_Linux_x64_AVMP.conf     # the piped N declines Miniconda
   ```
   - When it asks **"Would you like to start the Jupyter and Miniconda installation? [y/N]"**, answer **N**. ROS doesn't need it.
   - Run it **after** `setup-dev-env.sh` has finished. It runs `apt` itself and would collide with ansible's apt lock. It also pulls in Ubuntu point updates of the gcc-12 runtime (`libstdc++6` and so on), which is harmless.
   - The script writes `/etc/ld.so.conf.d/Arena_SDK.conf` (`lib64`, `GenICam/library/lib/Linux64_x64`, `ffmpeg`, `Metavision/lib`). `lucid_vision_driver`'s `cmake/FindARENA.cmake` reads this file to locate the SDK. It also apt-installs `g++`, `make`, `libibverbs-dev`, `librdmacm-dev` and the xcb/GL dev packages, then runs `ldconfig`.
   - Don't pass `-cti`. It is only for GenTL/.cti consumers, not the ROS driver.
3. ⚠️ **Fix the socket buffer setting the script appended.** It appends `net.core.rmem_max=33554432` (32 MB) to `/etc/sysctl.conf`. `/etc/sysctl.conf` is applied *after* `/etc/sysctl.d/*`, so this line silently overrides Autoware's 2 GB CycloneDDS value from Section 10. Remove that one line and keep `rmem_default`:
   ```bash
   sudo sed -i '/^net.core.rmem_max=33554432$/d' /etc/sysctl.conf
   sudo sysctl --system
   ```
4. Reboot, as the SDK README asks. Then confirm the libraries resolve:
   ```bash
   cat /etc/ld.so.conf.d/Arena_SDK.conf      # first line must be .../ArenaSDK_Linux_x64/lib64
   ldconfig -p | grep -E 'libarena\.so|libGCBase'
   ```
5. Add a short `~/source/arena_sdk/README.md` that records the version (1.0.8.60 AVMP), where the tarball came from, and that the uninstall script is `sudo sh Uninstall_Arena_SDK_Linux_x64_AVMP.sh`.
6. Camera network: the Triton cameras use link-local addressing. Set the NIC the camera is plugged into to `169.254.0.1/16`. Then put the camera's serial number in `src/sensor_component/external/lucid_vision_driver/param/camera_0.param.yaml`. The SDK's `precompiledExamples/IpConfigUtility` can list cameras and set their IPs.

> No SDK yet? You can still build everything else by skipping the camera package (see Section 9).

---

## 8. Install ROS dependencies

```bash
source /opt/ros/humble/setup.bash
cd ~/autoware.mach_e

sudo rosdep init 2>/dev/null || true     # only needed the first time
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
  --skip-keys "sample_sensor_kit_description sample_vehicle_description sample_sensor_kit_launch"
```

The `--skip-keys` are needed because the lab's `autoware_launch` renamed the sample vehicle and sensor kit to `mach_e_*`. Three packages still list the old names as `exec_depend`s, but those packages no longer exist:

| Package | Missing key(s) |
|---|---|
| `mach_e_sensor_kit_launch` | `sample_sensor_kit_description` |
| `awsim_sensor_kit_description` | `sample_sensor_kit_description` |
| `autoware_sample_designs` | `sample_vehicle_description`, `sample_sensor_kit_description`, `sample_sensor_kit_launch` |

Find any others with a dry run: `rosdep install -s ...`. If rosdep reports another unresolvable key (for example from `dbw_ros`), add it to `--skip-keys` the same way.

---

## 9. Build

Build from a **fresh terminal**, so that `~/.bashrc` has been read. The ansible `acados` role puts `CMAKE_PREFIX_PATH`, `ACADOS_SOURCE_DIR` and `LD_LIBRARY_PATH=/opt/acados/lib` there. Or export them yourself, as below:

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/opt/acados:$CMAKE_PREFIX_PATH ACADOS_SOURCE_DIR=/opt/acados \
       LD_LIBRARY_PATH=/opt/acados/lib:$LD_LIBRARY_PATH
cd ~/autoware.mach_e
MAKEFLAGS=-j4 colcon build --symlink-install --parallel-workers 12 --continue-on-error \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- **The first build took 40 minutes** on this PC (489 packages) with the settings above.
- **Without the acados variables,** `autoware_trajectory_optimizer` fails to link with `libacados.so: undefined reference to 'blasfeo_…' / 'd_ocp_qp_…' / 'QProblem_…'`. The reason: `libacados.so` has no RUNPATH. To fix it, export the variables above and run `colcon build --symlink-install --packages-above autoware_trajectory_optimizer --cmake-args -DCMAKE_BUILD_TYPE=Release`.
- **ArenaSDK not installed yet?** Add `--packages-skip lucid_vision_driver`.
- **Out of memory or the PC freezes?** Limit parallelism:
  ```bash
  MAKEFLAGS="-j8" colcon build --symlink-install --parallel-workers 8 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```
- **Rebuild one package after editing it:** `colcon build --symlink-install --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=Release`
- **A failure in one package:** `colcon build ... --packages-start <pkg>` resumes from that package. The logs are in `~/autoware.mach_e/log/latest_build/<pkg>/`.

---

## 10. Environment script (`~/utils/autoware_env_setup`)

Per the lab standard, environment config goes in `~/utils/`, one folder per tool, each with a README.

```bash
mkdir -p ~/utils/autoware_env_setup
cp ~/autoware.mach_e/docker/files/cyclonedds.xml ~/utils/autoware_env_setup/cyclonedds.xml

cat > ~/utils/autoware_env_setup/aw.sh << 'EOF'
#!/usr/bin/env bash
# Source the RANCS Lab Mach-E Autoware environment.
source /opt/ros/humble/setup.bash
source "$HOME/autoware.mach_e/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/utils/autoware_env_setup/cyclonedds.xml"
# acados (MPC solver used by autoware_trajectory_optimizer). libacados.so has no RUNPATH,
# so its deps (hpipm, blasfeo, qpOASES) must be found via LD_LIBRARY_PATH at build and run time.
case ":${LD_LIBRARY_PATH:-}:" in *":/opt/acados/lib:"*) ;; *) export LD_LIBRARY_PATH="/opt/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;; esac
export ACADOS_SOURCE_DIR=/opt/acados
EOF
chmod +x ~/utils/autoware_env_setup/aw.sh

cat > ~/utils/autoware_env_setup/README.md << 'EOF'
# autoware_env_setup
`source ~/utils/autoware_env_setup/aw.sh` sets up a shell for the Mach-E Autoware build in
~/autoware.mach_e (ROS 2 Humble + CycloneDDS). `cyclonedds.xml` is copied from
autoware.mach_e/docker/files and binds DDS to `lo` (single-PC setup); change the
NetworkInterface if Autoware has to talk to another computer.
See ~/documentation/autoware_mach_e_install.md.
EOF

# Optional shortcut: type `aw` in any new terminal
echo "alias aw='source ~/utils/autoware_env_setup/aw.sh'" >> ~/.bashrc
```

### DDS / network tuning (once per PC)

These are Autoware's recommended kernel buffer settings for CycloneDDS:

```bash
sudo tee /etc/sysctl.d/10-cyclone-max.conf << 'EOF'
net.core.rmem_max=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
sudo sysctl --system

# cyclonedds.xml binds to `lo`, which needs multicast. Make that persistent with a oneshot unit:
sudo tee /etc/systemd/system/lo-multicast.service << 'EOF'
[Unit]
Description=Enable multicast on loopback (CycloneDDS bound to lo for Autoware)
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/sbin/ip link set lo multicast on
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload && sudo systemctl enable --now lo-multicast.service
```

Check that it worked: `sysctl net.core.rmem_max` should print `2147483647`, and `ip link show lo` should list `MULTICAST`. If `rmem_max` shows `33554432`, the ArenaSDK script's line in `/etc/sysctl.conf` is overriding it (Section 7, step 3).

---

## 11. Maps and ROS bags

### Maps: `~/autoware_map/<map_name>/`

One folder per map, named `map_<place>_<size>` (for example `map_alumni_small`):

```
~/autoware_map/map_busyard_large/
├── pointcloud_map.pcd            # single file OR a folder of tiles
│   ├── pointcloud_map_xxx_yyy.pcd    (if tiled)
├── pointcloud_map_metadata.yaml  # only for tiled maps
├── map_projector_info.yaml
└── lanelet2_map.osm
```

### ROS bags: `~/rosbags/<project>/`

Every project folder needs a `README.md` written for someone who has never met you. Bag folders keep the default timestamped name:

```bash
mkdir -p ~/rosbags/my_project && cd ~/rosbags/my_project
ros2 bag record -o rosbag2_$(date +%Y_%m_%d-%H_%M_%S) /ouster/points /genesys/adma/fix /genesys/adma/imu ...
```

---

## 12. Run Autoware

Run `aw` (or `source ~/utils/autoware_env_setup/aw.sh`) in every terminal first.

> **Model names:** the lab's `autoware_launch` replaced Autoware's `sample_vehicle`/`sample_sensor_kit` with the Mach-E packages, so **always** pass
> `vehicle_model:=mach_e_vehicle sensor_model:=mach_e_sensor_kit`.
> The launch file defaults (`sample_vehicle`, `sample_sensor_kit`) don't exist in this build.

### 12.1 Smoke test: planning simulator (no hardware needed)

```bash
# Download Autoware's sample map (gdown was installed by setup-dev-env.sh)
gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
unzip -d ~/autoware_map ~/autoware_map/sample-map-planning.zip
mv ~/autoware_map/sample-map-planning ~/autoware_map/map_sample_planning
rm ~/autoware_map/sample-map-planning.zip

aw
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/map_sample_planning \
  vehicle_model:=mach_e_vehicle sensor_model:=mach_e_sensor_kit
```

In RViz:
1. Set **2D Pose Estimate**.
2. Set **2D Goal Pose**.
3. Click **Auto** in the OperationMode panel.

The ego vehicle should drive the route. If it does, the install works.

**Headless version (no RViz, scriptable):** run the launch above with `rviz:=false`, wait about 45 s, then run `python3 ~/utils/planning_sim_drive_test/drive_test.py`. It sets the pose, sets a route 75 m down a straight lane, engages AUTONOMOUS, and exits with 0 when the car arrives. During the first install on this PC it reported `arrived=True distance_driven=75.0 m max_speed=8.18 m/s` in 21 s.

Ignore the `component_container_mt ... process has died [exit code -11]` messages (`mission_planner_container`, `motion_planning_container`) that appear **after Ctrl-C**. They are a crash during shutdown, not while running.

### 12.2 On the vehicle

```bash
aw
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=$HOME/autoware_map/<map_name> \
  vehicle_model:=mach_e_vehicle sensor_model:=mach_e_sensor_kit
```

Useful switches from the lab's `autoware.launch.xml`:

| Argument | Default | Use |
|---|---|---|
| `perception_mode` | `camera_lidar_fusion` | Set it to `lidar` to run without the LUCID camera |
| `launch_vehicle_interface` | `true` | Set it to `false` to run without the Dataspeed DBW |
| `launch_sensing_driver` | `true` | Set it to `false` when replaying a rosbag |
| `launch_dummy_perception` | `false` | Adds dummy objects, for planning tests |

The sensors behind this setup:
- **Ouster OS-64:** driven by nebula, publishes `/ouster/points`.
- **ADMA GNSS/IMU:** launch `adma_ros2_driver` separately; it publishes `/genesys/adma/fix` and `/genesys/adma/imu`.
- **LUCID camera0.**
- **DBW2:** the `ds_dbw_autoware` vehicle interface.

### 12.3 Bench test without the DBW (`fake_vehicle_status.py`)

`autoware.mach_e` ships `fake_vehicle_status.py`. It publishes velocity, steering, gear and control-mode reports, and answers `/control/control_mode_request`, so Autoware can engage without the real vehicle. Keep it in `~/utils/` per the standard:

```bash
mkdir -p ~/utils/fake_vehicle_status
cp ~/autoware.mach_e/fake_vehicle_status.py ~/utils/fake_vehicle_status/
echo "# fake_vehicle_status
Stands in for the Dataspeed DBW vehicle interface on the bench (publishes /vehicle/status/*,
serves /control/control_mode_request). Source: rancslab/autoware.mach_e/fake_vehicle_status.py.
Run: aw && python3 ~/utils/fake_vehicle_status/fake_vehicle_status.py
Launch Autoware with launch_vehicle_interface:=false." > ~/utils/fake_vehicle_status/README.md

aw && python3 ~/utils/fake_vehicle_status/fake_vehicle_status.py
# then, in another terminal, launch autoware.launch.xml with launch_vehicle_interface:=false
```

---

## 13. Updating

```bash
cd ~/autoware.mach_e
git pull                                                   # meta-repo (ansible, autoware.repos)
vcs import src < repositories/rancslab.repos               # re-pin the RANCS forks/branches
vcs import --skip-existing src < repositories/autoware.repos   # add any newly listed repos
vcs pull src                                               # pull every repo on its current branch
```

- If `autoware.repos` changes the **version** of an upstream repo that's already cloned, run `vcs import src < repositories/autoware.repos` **without** `--skip-existing` to check out the new pins. Then re-run the `rancslab.repos` import so the forks win.
- After updating, re-run the rosdep command (Section 8) and the build (Section 9). If the Autoware version jumped, delete `build/ install/ log/` first.

---

## Appendix A: all RANCS Lab repos

I checked all 19 public repos in `github.com/rancslab` (as of 2026-10-06), plus `dbw_ros`, which is private.

**Used by this Autoware install**

| Repo | Fork of | Branch |
|---|---|---|
| autoware.mach_e | autowarefoundation/autoware | main |
| autoware_universe | autowarefoundation/autoware_universe | mach-e |
| autoware_launch | autowarefoundation/autoware_launch | main |
| nebula | ub-cavas/nebula (← tier4/nebula) | add-ouster-support |
| lucid_vision_driver | autowarefoundation/lucid_vision_driver | main |
| adma_ros_driver | GeneSysElektronik/adma_ros_driver | ros2_master |
| dbw_ros (private) | Dataspeed dbw_ros | ros2 |

**Not part of the Autoware install.** If you need one, clone it into `~/source/<name>/`.

| Repo | Why it's excluded |
|---|---|
| carma-platform, carma-platform-archive, carma-platform-stable, carma-config, carma-config-archive | USDOT CARMA platform, a separate AV stack and not Autoware |
| novatel_oem7_driver, novatel_gps_driver | NovAtel GNSS drivers (CARMA forks). The Mach-E sensor kit uses the GeneSys ADMA instead |
| object_detection | YOLOv5 Ouster-LiDAR experiment (PyTorch), not a ROS 2 package |
| e-stop, cohda-txrx, CANBatteryDashboard | Lab hardware and tooling projects that aren't Autoware packages |

---

## Appendix B: troubleshooting

| Symptom | Fix |
|---|---|
| `setup-dev-env.sh` fails at `Install cuda-drivers`: `pkgProblemResolver::Resolve generated breaks`, with `libnvidia-compute-550 : Conflicts: libnvidia-compute` | Ubuntu's own `nvidia-driver-550` conflicts with NVIDIA's `cuda-drivers`. Swap them in one transaction, then rerun the script (Section 5) |
| `nvidia-smi`: `Failed to initialize NVML: Driver/library version mismatch` | The new driver is installed but the old kernel module is still loaded. Reboot |
| `autoware_trajectory_optimizer` link error: `libacados.so: undefined reference to 'blasfeo_…'` | The acados env vars are missing from the build shell. Use a fresh terminal or `aw`, then run `--packages-above autoware_trajectory_optimizer` (Section 9) |
| AD API `set_route_points` returns `"The route is already set."` or `"Route API is not ready"` right after initializing the pose | `mission_planner` hasn't received odometry yet. Wait a few seconds and retry |
| `CUDA driver version is insufficient` / TensorRT errors | `nvidia-smi` must show driver ≥ 570. Rerun `./setup-dev-env.sh` (without `--no-cuda-drivers`) and reboot |
| `Could NOT find ARENA` / `arena_sdk_*` undefined when building `lucid_vision_driver` | ArenaSDK isn't installed or `/etc/ld.so.conf.d/Arena_SDK.conf` is missing (Section 7). Or build with `--packages-skip lucid_vision_driver` |
| `vcs import` prompts for a password or fails on `dbw_ros` | No rancslab access. Set up a PAT or SSH key (Section 3), or ask a lab admin to add you to the org |
| `Path already exists and contains a different repository` | A folder was cloned from upstream first. Delete it under `src/` and rerun `vcs import src < repositories/rancslab.repos` |
| rosdep: `Cannot locate rosdep definition for [sample_sensor_kit_description]` (or `sample_vehicle_description`, `sample_sensor_kit_launch`) | Use `--skip-keys` (Section 8) |
| `package 'sample_vehicle_description' not found` at launch | Pass `vehicle_model:=mach_e_vehicle sensor_model:=mach_e_sensor_kit` |
| Build freezes the PC / `c++: fatal error: Killed` | Out of memory. Use `--parallel-workers 8` and `MAKEFLAGS=-j8` (or lower) |
| `Could not resolve host: github.com` during a CMake FetchContent or clone | DNS/network problem (this happened on this PC before, see `~/ouster fetch build fail`). Check `ping github.com` and `resolvectl status`, then fix the network and rerun |
| `sysctl net.core.rmem_max` shows `33554432` instead of `2147483647` | The ArenaSDK config script appended it to `/etc/sysctl.conf`, which overrides `/etc/sysctl.d/`. Remove that line (Section 7, step 3) and run `sudo sysctl --system` |
| Nodes can't see each other or topics are missing | Make sure every terminal sourced `aw.sh` (same `RMW_IMPLEMENTATION`/`CYCLONEDDS_URI`) and that `lo` multicast is on (Section 10) |
