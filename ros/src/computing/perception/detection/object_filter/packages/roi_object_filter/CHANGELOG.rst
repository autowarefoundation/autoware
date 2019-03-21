^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roi_object_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Updated license to Apache
* * Fix install commands
  * Fix license
* Updated License notice (`#1850 <https://github.com/CPFL/Autoware/issues/1850>`_)
* Added CheckPointInGrid Test to RoiFilter
* Adaptation of Object Filter with new perception workflow
* feature/test_object_sanitizer (`#1713 <https://github.com/CPFL/Autoware/issues/1713>`_)
  * Added unit test base
  * Added friend class and sample test
  * Added comment on test
* Updated readme
* Initial release of object filter
* Contributors: Abraham Monrroy, Abraham Monrroy Cano, Servando, amc-nu
