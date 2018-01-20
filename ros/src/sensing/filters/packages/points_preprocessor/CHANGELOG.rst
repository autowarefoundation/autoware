^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package points_preprocessor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2018-01-20)
------------------

1.6.0 (2017-12-11)
------------------
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge branch 'feature/decision' of github.com:cpfl/autoware into feature/decision
* Merge branch 'feature/OpenPlanner' into develop
* Merge branch 'develop' into feature/OpenPlanner
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Merge branch 'develop' of github.com:CPFL/Autoware into feature/decision
* decoupled ray ground filter into lib and exe, added unit test (`#932 <https://github.com/CPFL/Autoware/issues/932>`_)
* - Add new Node for object polygon representation and tracking (kf_contour_tracker)
  - Add launch file and tune tracking parameters
  - Test with Moriyama rosbag
* merge develop and fixed slow switching map
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Feature/ring_ground_filter parameter description (`#884 <https://github.com/CPFL/Autoware/issues/884>`_)
  * Added a README file for ground_filter tuning
  * Moved and formatted Patipon instructions on ring_ground_filter
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Merge branch 'develop' into feature/ndt_pcl_gpu
  Conflicts:
  ros/src/computing/perception/localization/packages/ndt_localizer/CMakeLists.txt
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_mapping/ndt_mapping.cpp
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_matching/ndt_matching.cpp
* Merge branch 'master' into feature/decision
* Feature/fusion_filter - fusion multiple lidar (`#842 <https://github.com/CPFL/Autoware/issues/842>`_)
  * Add fusion_filter to merge multiple lidar pointclouds
  * Refactor fusion_filter
  * Apply clang-format and rebase develop
  * Add fusion_filter launch and runtime_manager config
  * Fix names, fusion_filter -> points_concat_filter
  * Fix build error in ros-indigo
  * Fix some default message/frame names
  * Refactor code and apply clang-format
  * Add configrations for runtime_manager
  * Fix CMake
* Feature/cloud transformer (`#860 <https://github.com/CPFL/Autoware/issues/860>`_)
  * Added Cloud transformer node
  transforms pointcloud to a target frame
  * Added support for XYZIR point type
  * Added error checks when transformation unavailable
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* Merge pull request `#826 <https://github.com/CPFL/Autoware/issues/826>`_ from CPFL/feature/ray_ground_filter
  Feature/ray ground filter
* Solved conflicts by ring filter config message naming change
* Merge branch 'develop' into feature/ray_ground_filter
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/sensing.yaml
* Add ground_filter config for runtime_manager (`#828 <https://github.com/CPFL/Autoware/issues/828>`_)
* Added Compilation fix for Kinect
* Added descriptions to the params in launch file
* Ray Ground Filter Initial Commit
* merge develop
* Contributors: AMC, Abraham Monrroy, Akihito Ohsato, Yamato ANDO, Yuki Iida, Yuki Kitsukawa, Yusuke FUJII, anhnv-3991, christopherho-ApexAI, hatem-darweesh, yukikitsukawa

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------
* [hotfix] fixes to lidar_tracker package(`#787 <https://github.com/cpfl/autoware/issues/787>`_)
  -Fixed a typo in the ground_filter launch file from points_preprocessor
  -Fixed ID duplication in kf_lidar_tracker
  Tested on Ubuntu 14.04 and 16.04
* Contributors: Abraham Monrroy

1.4.0 (2017-08-04)
------------------

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------

1.2.0 (2017-06-07)
------------------

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
* Rename package name.
  data_filter -> filters
  points_filter -> points_downsample
* Contributors: yukikitsukawa
