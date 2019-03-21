^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_driveworks_gmsl_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------
* [fix] Install commands for all the packages (`#1861 <https://github.com/CPFL/Autoware/issues/1861>`_)
  * Initial fixes to detection, sensing, semantics and utils
  * fixing wrong filename on install command
  * Fixes to install commands
  * Hokuyo fix name
  * Fix obj db
  * Obj db include fixes
  * End of final cleaning sweep
  * Incorrect command order in runtime manager
  * Param tempfile not required by runtime_manager
  * * Fixes to runtime manager install commands
  * Remove devel directory from catkin, if any
  * Updated launch files for robosense
  * Updated robosense
  * Fix/add missing install (`#1977 <https://github.com/CPFL/Autoware/issues/1977>`_)
  * Added launch install to lidar_kf_contour_track
  * Added install to op_global_planner
  * Added install to way_planner
  * Added install to op_local_planner
  * Added install to op_simulation_package
  * Added install to op_utilities
  * Added install to sync
  * * Improved installation script for pointgrey packages
  * Fixed nodelet error for gmsl cameras
  * USe install space in catkin as well
  * add install to catkin
  * Fix install directives (`#1990 <https://github.com/CPFL/Autoware/issues/1990>`_)
  * Fixed installation path
  * Fixed params installation path
  * Fixed cfg installation path
  * Delete cache on colcon_release
* Contributors: Abraham Monrroy Cano

1.10.0 (2019-01-17)
-------------------
* [fix] CMake error & warning fixes on develop (`#1808 <https://github.com/CPFL/Autoware/issues/1808>`_)
  * CMake fixes
  * CMake updated to remove unnecessary dependencies when the package is not built
  * added autoware flags
* Feature/gmsl multiple (v2) (`#1683 <https://github.com/CPFL/Autoware/issues/1683>`_)
  Added driver for multiple GMSL cameras for the Drive PX2.
* Contributors: Abraham Monrroy Cano, Esteve Fernandez
