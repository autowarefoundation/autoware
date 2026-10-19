#!/usr/bin/env bash
# Source the RANCS Lab Mach-E Autoware environment.
source /opt/ros/humble/setup.bash
source "$HOME/autoware.mach_e/install/setup.bash"
# Standalone sensor drivers (ouster_ros). local_setup adds only this workspace's packages.
[ -f "$HOME/ros2_ws/install/local_setup.bash" ] && source "$HOME/ros2_ws/install/local_setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/utils/autoware_env_setup/cyclonedds.xml"
# acados (MPC solver used by autoware_trajectory_optimizer). libacados.so has no RUNPATH,
# so its deps (hpipm, blasfeo, qpOASES) must be found via LD_LIBRARY_PATH at build and run time.
case ":${LD_LIBRARY_PATH:-}:" in *":/opt/acados/lib:"*) ;; *) export LD_LIBRARY_PATH="/opt/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;; esac
export ACADOS_SOURCE_DIR=/opt/acados
