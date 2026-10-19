# mach_e_bringup
Run Autoware on the real Mach-E. This starts the Ouster OS1-128 lidar, the LUCID camera, the ADMA GNSS/INS and the Dataspeed DBW2 vehicle interface, with map PD2LAB by default.

```bash
~/utils/mach_e_bringup/run_mach_e.sh              # PD2LAB, with RViz
~/utils/mach_e_bringup/run_mach_e.sh USF_map      # any map folder under ~/autoware_map
~/utils/mach_e_bringup/check_topics.sh            # in a second terminal: rate of every sensor/vehicle topic
```

`run_mach_e.sh` does two things:
- It starts the ADMA driver (`adma_ros2_driver`) in the background. The sensor kit doesn't launch the ADMA driver, so this is the only thing that starts it.
- It then runs `autoware.launch.xml` with `vehicle_model:=mach_e_vehicle sensor_model:=mach_e_sensor_kit`.

Logs for each run go in `~/autoware_logs/runs/<date>_<map>/`. Arguments after the map name are passed to `ros2 launch`, e.g. `rviz:=false`. `ADMA_CONFIG=<yaml>` selects a different ADMA config (for the bus).

**Nothing moves until someone engages AUTONOMOUS** (RViz "Engage", or the AD API). Until then, `ds_dbw_autoware` publishes `/vehicle/disable` every cycle and sends no motion commands.

Network (two NM profiles):
- `mach-e-sensors` on `eno1`: 192.168.88.100/24 + 169.254.6.1/16, MTU 9000. Connects through the switch to the lidar and ADMA.
- `mach-e-camera` on `enp4s0`: 192.168.88.101/24 (route metric 200) with host routes 192.168.88.107/32 (the camera) and 192.168.88.126/32 (its old address). MTU 9000, rx ring 4096. The camera is cabled straight to this port, so it doesn't share the switch with the lidar. On a shared 1 Gb/s link, camera ~481 Mbps + lidar ~254 Mbps gave dropped camera packets (horizontal banding in ArenaView).
  - Both profiles are on 192.168.88.0/24 and the `eno1` route wins (metric 101), so a camera address without a host route here is sent out `eno1` and never answers. ArenaView then can't connect. If the camera's IP changes, add its host route: `nmcli con modify mach-e-camera +ipv4.routes "<ip>/32 src=192.168.88.101" && nmcli device reapply enp4s0`. The ROS driver finds the camera by serial number, so only the route has to follow the IP.

| Device | Address | Topic(s) |
|---|---|---|
| Ouster OS1-128 | 192.168.88.37 (UDP 7502/7503) | `/sensing/lidar/top/pointcloud_raw_ex` 10 Hz |
| LUCID camera, SN 262700247 | 192.168.88.107 (direct on enp4s0; was .126 until 2026-10-09) | `/sensing/camera/camera0/image_raw` ~25-30 Hz |
| GeneSys ADMA | 192.168.88.30, sends to 192.168.88.100:4444, protocol v3.3.4 | `/genesys/adma/{fix,imu}` 200 Hz |
| Dataspeed USB CAN Tool | USB, CAN ch3 500 kbps | `/vehicle/status/*` |

Notes from the first bring-up on 2026-10-08 (vehicle parked indoors):
- All sensor and vehicle topics are up, and perception publishes objects.
- The ADMA reported `warn_gnss_no_solution` (no sky view), so there's no `/sensing/gnss/pose`. Initialize localization with RViz "2D Pose Estimate".
- Nothing publishes `/autoware_orientation` (the GNSS heading input). That was also true on the previous PC, so GNSS-only initialization gets a wrong heading.
- The first launch builds the TensorRT engines (~15 min); they are cached in `~/autoware_data/ml_models`. On this PC they are built for the RTX A4000.

See `~/autoware_logs/autoware_install_log.md` (2026-10-08).

2026-10-15 drive: issues, causes and fixes (EKF MRM flicker fix, override re-engage behaviour, RViz GPU leak, light RViz) are in `ISSUES_AND_FIXES_2026-10-15.md`.
