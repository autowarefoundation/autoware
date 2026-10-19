# Mach-E Autoware drive, 2026-10-15: issues, causes, fixes

Two drives on map PD2LAB (18:48-19:00 and 19:11-19:35).
- Logs: `~/.ros/log/2026-10-15-18-48-15-*` for the first drive and `~/autoware_logs/runs/2026-10-15_191105_PD2LAB/` for the second.
- Bags (planning, control, MRM and diagnostics topics): `/mnt/mapdata/autoware_runs/20261015_185315_PD2LAB_drive` and `/mnt/mapdata/autoware_runs/2026-10-15_191105_PD2LAB_planning`.

| # | Issue | Status |
|---|---|---|
| 1 | MRM emergency stop switches on/off several times a second; Engage sometimes refused | **Fixed** (EKF code); not yet verified on the car |
| 2 | Car re-engages by itself after a pedal/steering override (10-13 stop-line overrun) | **Kept as designed** (fix reverted at the team's request) |
| 3 | RViz fills the GPU, then the lidar preprocessing crashes (out of memory) | **Fixed**: NVIDIA driver setting (root cause found); stock RViz is the default again |
| 4 | Stop line 3083 logs "No stop point found" | Not a bug |
| 5 | Map RE 2878 is tagged as a traffic light but points at a keep-right sign | Open (map fix) |
| 6 | 10-13: car sped up approaching stop line 3047 although Autoware commanded braking | Open (needs a braking test) |

---

## 1. MRM emergency-stop flicker (FIXED)

**Symptom**
- While driving autonomously, `/system/fail_safe/mrm_state` jumped between NORMAL and EMERGENCY_STOP about 5 times per second.
- First drive: 367 episodes in about 200 s, 60 s in total. Second drive: 23% of the time.
- Each time, `vehicle_cmd_gate` logged "Emergency!" and replaced the controller command with the emergency-stop command (−2.4 m/s²).
- Engage was sometimes refused with "The target mode is not available".
- The same pattern is in the 2026-10-13 logs.

**Cause:** a bug in the EKF's diagnostic reporting, not a real localization problem.
- The EKF (`ekf_localizer`) runs at 50 Hz, but NDT poses arrive at 10 Hz. So 4 of every 5 EKF cycles have no new pose.
- At the start of every cycle, `initialize_diagnostic_info()` sets `is_passed_delay_gate` and `is_passed_mahalanobis_gate` to false. They're only set back to true when a pose is processed.
- So every cycle without a pose published WARN "pose topic is delay; mahalanobis distance of pose topic is large" with delay = NaN and queue size = 0.
- In this build, the autonomous mode counts as available only if every diagnostic is OK, so a WARN makes it "unavailable". The MRM handler then starts EMERGENCY_STOP, and cancels it on the next cycle that has a pose.
- The real NDT pose age was only about 180 ms (max ~420 ms), against a 1.0 s limit.

**Fix**
- File: `~/autoware.mach_e/src/core/autoware_core/localization/autoware_ekf_localizer/src/ekf_localizer.cpp`.
- In the timer, the pose and twist gate flags are now set to true *before* the `if (!queue.empty())` blocks instead of inside them. A cycle with no new measurement therefore reports "passed".
- A pose or twist that really is late or rejected still sets the flag to false (in `ekf_module.cpp`) and still raises the WARN.
- `initialize_diagnostic_info()` is unchanged, so when the EKF is not activated or has no initial pose, the gates still report failed. A unit test checks this.
- Rebuilt with `colcon build --symlink-install --packages-select autoware_ekf_localizer --cmake-args -DCMAKE_BUILD_TYPE=Release`. `colcon test` gives 0 failures.

**Check on the next drive**
- `/system/fail_safe/mrm_state` stays at state 1 / behavior 1 (NORMAL / NONE) while engaged.
- No "Emergency!" lines in `autoware.log`.
- Engage works on the first press.

Still open: `[EKF] Twist queue size (3) is exceeding max_queue_size (2)`. Fix: `max_twist_queue_size: 2` → `4` in `autoware_launch/config/localization/ekf_localizer.param.yaml`. The file's own comment says it must be larger than `twist_smoothing_steps` (2).

## 2. Automatic re-engage after a driver override (KEPT AS DESIGNED)

**Behaviour**
- After a pedal or steering override, `vehicle_interface_node.py` keeps sending `/vehicle/enable`. So Autoware takes over again as soon as the driver lets go: about 0.16 s after the override on 2026-10-13.
- This is intentional: the safety driver can step in briefly and Autoware continues on its own afterwards.

**What happened on 10-13 (stop line 3047)**
- An accelerator override about 0.1 s before the stop point was followed by an immediate re-engage, with the car already past the stop point.
- The stop-line module only stops before the line. Once past the stop point it gives up, and with `enable_overshoot_emergency: false` nothing brakes, so the car continued through.

**For the safety driver:** if you step in near a stop line and let go after passing its stop point, Autoware will **not** stop at that line. Brake to a stop yourself in that case.

**History:** a fix that kept the car in MANUAL after an override until Engage was pressed again was applied on 2026-10-15, then reverted at the team's request. The file is back to the committed version.

## 3. RViz GPU memory leak, and the lidar crash it caused (FIXED, root cause found)

**Root cause:** the NVIDIA driver (615.71.09) turns on its **GL threaded optimizations** for rviz2 automatically. Nothing in our setup asked for it. With that on, every point-cloud vertex buffer RViz frees is never returned: one full buffer lost per message, ~3 GB/min with the full lidar.

**Fix:** `~/.nv/nvidia-application-profiles-rc`, a per-user NVIDIA application profile that sets `GLThreadedOptimizations = false` for processes named `rviz2`.
- It applies however RViz is started: `run_mach_e.sh`, a plain `ros2 launch autoware_launch autoware.launch.xml ...`, or standalone.
- A copy is in `display/nvidia-application-profiles-rc`. Install it on another PC or user account with `cp display/nvidia-application-profiles-rc ~/.nv/`.
- `__GL_THREADED_OPTIMIZATIONS=0` in the environment does the same thing for one shell.

**Verified** with replayed OS1-128 data (10 Hz, ~262k points) in an isolated test RViz:
| RViz setup | Driver default | With the fix |
|---|---|---|
| Autoware lidar display only | +2,520 to +2,942 MiB/min (two runs) | **+4 MiB/min** (flat) |
| Complete stock `autoware.rviz` | ~+3,000 MiB/min | **+10 MiB/min** (flat) |
RViz still renders the full cloud (~33% CPU).

**Launcher:** `run_mach_e.sh` again starts the **stock** Autoware RViz: full map, full lidar, full obstacle cloud. `rviz_gpu_guard.sh` stays as a backstop and restarts RViz if total GPU memory passes 13 GB, for example if the profile is missing. The light config (below) is opt-in with `RVIZ_LIGHT=1`.

### History (workarounds used before the root cause was found)
**Symptom**
- First drive: RViz grew to 11.8 GB of the RTX A4000's 16 GB in 12 minutes.
- At 18:59:30 the CUDA lidar preprocessing in `pointcloud_container` died with `thrust::bad_alloc: cudaErrorMemoryAllocation: out of memory`.
- After that there was no concatenated or obstacle pointcloud and no NDT: localization was dead-reckoning only and perception had no lidar.

**Cause**
- In this rviz2 (Humble) setup, every live PointCloud2 display leaks GPU memory. It allocates new buffers for each message and never frees the old ones.
- Measured with a test RViz per display group, 30 s each:
  - **Sensing** (lidar cloud): about 800 MB/min
  - **Perception** (NoGroundPointCloud): about 500 MB/min
  - System, Map, Localization, Planning and Control: no growth
- The occupancy-grid Map display was also ruled out.
- The total of about 1.3–1.5 GB/min matches the growth seen during both drives.
- Showing the full 23.6M-point map was not the leak, only a large one-time load.

**What was done**
- `run_mach_e.sh` now starts a **light RViz** by default; use `RVIZ_FULL=1` for the stock RViz. The files are in `display/`:
  - `display_map_pub.py`: a 0.5 m voxel copy of the map (2.6M points), published once on `/display/pointcloud_map`.
  - A random subsample of the lidar (`RVIZ_LIDAR_POINTS`, default 40,000 points) on `/display/lidar`.
  - `make_light_rviz.py`, which generates `autoware_light.rviz` from the stock config. It turns off the NDT debug clouds and the occupancy-grid display.
- Two RViz traps found along the way:
  - Rewriting the `.rviz` file through a YAML parser breaks it. So does disabling the camera/Image displays. Both make RViz fail with "Could not load display config: Invalid argument", leaving it with no Displays panel and no 3D view. The generator therefore edits the text line by line and leaves the Image/Camera displays alone.
  - The backup of the old script is `run_mach_e.sh.bak_20261015`.
- **During the second drive, RViz was restarted on purpose** whenever it passed 7 GB, by a temporary watchdog. Autoware respawns RViz automatically (`rviz_respawn:=true`). See "Why RViz kept restarting" below.

**Fix: throttled lidar view, full-rate obstacle cloud, GPU guard** (all started by `run_mach_e.sh`)
- `display/display_cloud_throttle.py` gives RViz its own copies of the clouds. Autoware itself still uses the full clouds.
  - **General lidar view:** `/sensing/lidar/concatenated/pointcloud` → `/display/lidar`, at 2 Hz with at most 20,000 random points. Set with `RVIZ_CLOUD_RATE` and `RVIZ_LIDAR_POINTS`.
  - **Obstacle cloud:** `/perception/obstacle_segmentation/pointcloud` → `/display/obstacle_pointcloud`, **every message at full rate** (10 Hz in = 10 Hz out, tested). Capped at 50,000 points (`RVIZ_OBSTACLE_POINTS`). It is not throttled because the safety driver must see obstacles without delay.
  - In `autoware_light.rviz` the ConcatenatePointCloud and NoGroundPointCloud displays use these topics, with Best Effort QoS.
- `display/rviz_gpu_guard.sh` is a safety net. It restarts RViz (Autoware respawns it, ~10 s without a view) if **total** GPU memory passes `RVIZ_GPU_LIMIT` (default 13,000 MiB), so the lidar pipeline can never be starved again. Each restart is logged in `rviz_gpu_guard.log` in the run folder. It also runs with `RVIZ_FULL=1`.

**Measured** with replayed OS1-128 data (Parking_lot_coast bag) in an isolated test RViz:
| Cloud drawn by RViz | Style | GPU growth |
|---|---|---|
| full cloud, 10 Hz, ~260k points | Points | +2,600 to +3,200 MiB/min |
| full cloud | Flat Squares | +14,000 MiB/min |
| full cloud | Squares | +16,000 MiB/min |
| throttled, 2 Hz, 20k points | Points | +72 MiB/min |

- The leak is roughly **2–2.5 MiB/min per 1,000 points/s drawn**. The render style doesn't remove it, and **Points** (what we use) is the least bad.
- Expected on the car: lidar view ~70 MiB/min, plus the obstacle cloud at about (its points × 10 Hz) × 2.3 MiB per 1,000 points/s. For example, 20k obstacle points gives ~460 MiB/min.
- With large obstacle clouds the GPU guard will restart RViz now and then. The run watcher reports the real numbers. If restarts are too frequent, lower `RVIZ_OBSTACLE_POINTS` (e.g. 15000), which still shows obstacles clearly at 10 Hz.
- Obstacles are also shown as boxes by the PredictedObjects display, at full rate, and that display doesn't leak.
- The leak itself is inside rviz2 / OGRE. A newer rviz2 or NVIDIA driver might remove it (not tested).


**Foxglove comparison (2026-10-15, 21:08-21:12)**
- Same replayed full cloud (10 Hz, ~262k points), Foxglove Studio 3.3.0 with `foxglove_bridge` (send_buffer_limit 300 MB, localhost only, isolated ROS domain 88).
- GPU memory of the Foxglove processes: 225 → 194 MiB, then **flat for 3 minutes**. No leak.
- CPU: Foxglove ~1.4 cores (GPU process + renderer), bridge ~0.1 core, out of 24.
- Test layout: `display/foxglove_leaktest_layout.json`. Its colour settings are rejected (red "!"), so points draw black; set Color by in the topic settings.
- Possible next step: a hybrid setup. RViz keeps operating (Engage, goal pose, objects, path) with its PointCloud2 displays off, which removes the leak there; Foxglove shows the obstacle and lidar clouds at full rate.

### Why RViz kept restarting
- Every restart in the second drive had exit code −9, meaning it was killed on purpose and respawned by Autoware. None were crashes:
  1. Twice to load a corrected `autoware_light.rviz` (the "Could not load display config" problem above).
  2. Once by hand at 9.4 GB, when it was growing about 1.8 GB/min and about 1–2 minutes from starving the lidar pipeline again.
  3. Once to measure the leak without the occupancy-grid display.
  4. By the watchdog at 7.1 GB, about every 4–5 minutes.
- Without these restarts, the leak fills the GPU in about 10 minutes and the lidar preprocessing crashes, as it did in the first drive.
- From now on the GPU guard does this automatically, and only when total GPU memory passes 13 GB (see above).

## 4. Stop line 3083 "No stop point found" (NOT A BUG)
- The goal pose was at (−175.6, 81.2) on lanelet 41, about 74 m before stop line 3083 (y ≈ 7).
- The path ends at the goal and never crosses the line, so the stop-line module has nothing to stop for. It still logs a warning because lanelet 41 holds the stop-sign regulatory element.

Stop-line results today: stop sign 3047 was handled correctly twice (APPROACH → STOPPED → START) and 3087 once. No overruns.

## 5. Map: RE 2878 (OPEN)
- `lanelet2_map.osm` regulatory element 2878 has `subtype=traffic_light`, but its `refers` way 2874 is a traffic sign (`usR4-7`, keep right).
- With no recognised signal, Autoware treats line 2877 as an unknown light and stops there. On 10-13 the driver pushed through it.
- Fix: retag it, or remove it in the map editor.
- Stop lines 800 and 801 are not referenced by any regulatory element, so Autoware ignores them.

## 6. The car did not follow the braking command on 10-13 (OPEN)
- Approaching stop line 3047, Autoware commanded −1.1 to −2.0 m/s², but the car accelerated from 4.3 to 5.15 m/s.
- The DBW logged "Brake degraded: Insufficient control performance".
- The cause is either a foot on the accelerator below the override threshold, or ULC accel-mode braking not delivering. The second is untested while moving since commit 83fdc58.
- **Next step:** a closed-course rolling test. Command −1 and −2 m/s² at 4–5 m/s and compare `/vehicle/ulc/report` accel_ref with accel_meas.
- Until then, keep your foot off the accelerator while engaged.

## Other notes
- **ADMA:** `run_mach_e.sh PD2LAB` starts the ADMA driver itself. A plain `ros2 launch autoware_launch autoware.launch.xml ...` does **not**, so don't start both.
- **Engage refused while rolling** ("enable_engage_on_driving is false, and the vehicle is not stationary") is by design: engage from standstill.
- **Known startup errors, harmless:**
  - "Please publish TF base_link to imu_link" (only at startup)
  - costmap "map and base_link not connected" (before localization is initialized)
  - net_monitor socket error
  - "Map was not split" (voxel_based_compare_map_filter)
