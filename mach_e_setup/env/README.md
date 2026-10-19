# autoware_env_setup
`source ~/utils/autoware_env_setup/aw.sh` (or the `aw` alias in ~/.bashrc) sets up a shell for the
Mach-E Autoware build in ~/autoware.mach_e (ROS 2 Humble + CycloneDDS), plus the standalone drivers in
~/ros2_ws (`ouster_ros`) if that workspace is built. The two workspaces share no packages. `cyclonedds.xml` is copied from
autoware.mach_e/docker/files and binds DDS to `lo` (single-PC setup); change the NetworkInterface if
Autoware has to talk to another computer. Lives in ~/autoware.mach_e/mach_e_setup/env; ~/utils/autoware_env_setup is a symlink to it; see ../NEW_PC_SETUP.md.

`aw.sh` also exports the acados paths (`LD_LIBRARY_PATH=/opt/acados/lib`, `ACADOS_SOURCE_DIR`), because
`libacados.so` has no RUNPATH. Without them, building or running `autoware_trajectory_optimizer` fails with
undefined `blasfeo_*`/`d_ocp_qp_*`/`QProblem_*` symbols.
