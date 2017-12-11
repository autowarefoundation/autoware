^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package points2image
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2017-12-11)
------------------
* Merge pull request `#967 <https://github.com/CPFL/Autoware/issues/967>`_ from CPFL/fix/qt_autouic
  [bugfix]use qt5_wrap_ui instead of autouic
* use qt5_wrap_ui instead of autouic
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge branch 'develop' into feature/OpenPlanner
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* merge develop and fixed slow switching map
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Merge pull request `#888 <https://github.com/CPFL/Autoware/issues/888>`_ from CPFL/feature/cmake_refactor
  refactor CMakeLists.txt. use automoc, autouic and autorcc
* Merge branch 'feature/cmake_refactor' of github.com:cpfl/autoware into feature/decision
* Fix feature/points2image bug multicam support (`#886 <https://github.com/CPFL/Autoware/issues/886>`_)
  * pointgrey
  * Added New Calibration node
  * Added parameters, plane fitting
  * added mirror node, etc
  * Points2Image
  Calibration Publisher
  now works with multiple cameras using ros namespaces
  * Including only points2image
  * Added Launch file for points2 image specific for the ladybug camera
* refactor CMakeLists.txt. use automoc, autouic and autorcc
* Merge branch 'master' into feature/decision
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* merge develop
* Contributors: Abraham Monrroy, Yamato ANDO, YamatoAndo, Yuki Iida, Yusuke FUJII, hatem-darweesh, yukikitsukawa

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
* convert to autoware_msgs
* Contributors: YamatoAndo

1.2.0 (2017-06-07)
------------------
* fix circular-dependency
* Fixes
* Contributors: AMC, Shohei Fujii

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
* Add module graph tool
* Add missing dependencies
* Remove needless compiling flags
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* Merge branch 'master' of ssh://localhost:8443/CPFL/Autoware into apply_multi_camera
  WARNING: This commit contains unresolved conflict.
  Unresolved conflicts are contained in...:
  *
  ros/src/util/packages/runtime_manager/scripts/runtime_manager_dialog.py
  * ros/src/util/packages/runtime_manager/scripts/computing.yaml
  * ros/src/util/packages/runtime_manager/scripts/sensing.yaml
  Conflicts:
  ros/src/computing/perception/detection/packages/cv_tracker/launch/dpm_ocv.launch
  ros/src/computing/perception/detection/packages/cv_tracker/launch/dpm_ttic.launch
  ros/src/computing/perception/detection/packages/cv_tracker/launch/kf_tracking.launch
  ros/src/computing/perception/detection/packages/cv_tracker/launch/reprojection.launch
  ros/src/computing/perception/detection/packages/cv_tracker/nodes/obj_reproj/obj_reproj.cpp
  ros/src/computing/perception/detection/packages/viewers/nodes/scan_image_viewer/scan_image_viewer.cpp
  ros/src/sensing/fusion/packages/calibration_camera_lidar/CalibrationToolkit/calibrationtoolkit.cpp
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
  ros/src/util/packages/runtime_manager/scripts/points2image.launch
  ros/src/util/packages/runtime_manager/scripts/rtmgr.py
  ros/src/util/packages/runtime_manager/scripts/rtmgr.wxg
  ros/src/util/packages/runtime_manager/scripts/runtime_manager_dialog.py
  ros/src/util/packages/runtime_manager/scripts/sensing.yaml
  ros/src/util/packages/runtime_manager/scripts/vscan.launch
* Change a subscribing topic  in points2image and vscan2points when synchronization is enabled
* Correct calibration_camera_lidar dependnecy about message header
* Correct dependency name
* Delete image size fixing
* Ros-parameterize all defined value in points2vscan
  Now we can specify parameters value from launch file
* Update threshold
  So that vscan can work for close-range obstacles when velodyne 32 is
  used
* Make projection matrix source selectable
  I modified nodes that subscribe /projection_matrix
  so that we can specify the topic name from launch file
* Make camera_info source selectable
  I modified nodes that subscribe /camera/camera_info
  so that we can specify the topic name from launch file
* Clean up Qt5 configuration
  Use pkg-config as possible instead of absolute pathes.
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Updated point2image to set minh in the message
* updated fusion to optionally read from any points to image projected topic via argument points_node.
  default  topic vscan_image (not changed)
  updated points2image topic to optionally project any pointcloud2 topic via argu
  ment point_node.
  default topic: points_raw (not changed)
* Initial commit for public release
* Contributors: AMC, Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi, Yukihiro Saito
