^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kitti_player
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2017-12-11)
------------------
* Merge branch 'develop' into feature/OpenPlanner
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* merge develop and fixed slow switching map
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Merge branch 'develop' into feature/ndt_pcl_gpu
  Conflicts:
  ros/src/computing/perception/localization/packages/ndt_localizer/CMakeLists.txt
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_mapping/ndt_mapping.cpp
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_matching/ndt_matching.cpp
* Merge branch 'master' into feature/decision
* Feature/kitti player update for kinetic, autoware_msgs (`#865 <https://github.com/CPFL/Autoware/issues/865>`_)
  * Added support for Autoware Messages
  * Changed argument
  * Added support for Kinetic
  * Updated README
  * Added fps argument to launch file
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* merge develop
* Contributors: Abraham Monrroy, Yamato ANDO, Yuki Iida, Yusuke FUJII, anhnv-3991, hatem-darweesh, yukikitsukawa

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------

1.4.0 (2017-08-04)
------------------

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------

1.2.0 (2017-06-07)
------------------
* Fix compile error regarding kitty_player
  * Change compile flag from `-std=c++0x` to `-std=c++11` as `c++0x` seems to be deprecated later than GCC ver4.7
  * Fix `ROS_ERROR_STREAM` usages as `<<` operator in `ROS_ERROR_STREAM` doesn't seem to be able to use with ifstream directly
* hotfix make issue
* KITTI Dataset ROS Publisher, terminal mode only
* Contributors: AMC, Manato Hirabayashi, Yusuke Fujii

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
