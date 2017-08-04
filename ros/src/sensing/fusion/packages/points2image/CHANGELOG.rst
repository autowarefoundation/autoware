^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package points2image
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
