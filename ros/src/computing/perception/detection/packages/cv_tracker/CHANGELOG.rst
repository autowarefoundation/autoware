^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cv_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2017-12-11)
------------------
* Merge pull request `#949 <https://github.com/CPFL/Autoware/issues/949>`_ from CPFL/fix/obj_reproj_multi_cam_support
  Support for multi cameras in obj_reproj node
* Support for multi cameras in pbj_reproj node
* merge develop and fixed slow switching map
* Merge branch 'master' into feature/decision
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* merge develop
* Contributors: Yamato ANDO, Yuki Iida, Yusuke FUJII, yukikitsukawa

1.5.1 (2017-09-25)
------------------
* Update README.md (`#813 <https://github.com/cpfl/autoware/issues/813>`_)
  Added instructions to compile compatible version of SSD with Autoware
* Contributors: Abraham Monrroy

1.5.0 (2017-09-21)
------------------

1.4.0 (2017-08-04)
------------------

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* fix build issues due to autoware_msgs
* Resolved merge conflict by new feature
* Merge pull request `#713 <https://github.com/CPFL/Autoware/issues/713>`_ from CPFL/fix/ssd_build_redefinition_std
  fix redefinition error@yolo
* fix redefinition error
* Enable building nodes on the kinetic setup
  -road_wizard
  -klt_tracker
* opencv 3.0 support on ssd node
* convert to autoware_msgs
* Contributors: YamatoAndo, Yusuke FUJII, Yusuke Fujii

1.2.0 (2017-06-07)
------------------
* hotfix a typo
* hotfixed a yolo2 cmake issue
* fix a compile problem due to duplicate declaration of std and opencv3
* Added Readme for yolo2
* updated yolo2 for compatibility with commit 56d69e73aba37283ea7b9726b81afd2f79cd1134 @pjreddie pjreddie committed 4 hours ago
* Added Readme
* Changes to code to adapt to new cv_tracker messages structure
* Merge remote-tracking branch 'origin/develop' into feature/yolo2
* Yolo 2 Node completed
* Yolo OK
* fixed compile issue for ssd node
* cmakelists updated yolo2
* Fixed compatibility issues with indigo
* Initial Yolo2
* clean directories
* fix circular-dependency
* ROS Kinectic Upgrade tested on Ubuntu 16.04 and OpenCV 3.2.0
  Modules not included:
  -orb_localizer
  -dpm_ocv node and lib
  Everything else working
  Added some libraries for Gazebo on src/extras
* Update for kinetic
* Fixed an indication in review.
* Reflected the results of the review.
* Added sys dialog of ssd node.
* Fix rebase result of obj_reproj
  This is just done by following:
  `$git checkout master
  ros/src/computing/perception/detection/packages/cv_tracker/nodes/obj_reproj/obj_reproj.cpp`
* Add visualize function to obj_reproj
* Improve reprojection accuracy
  - Fix unit handling bug
  - Use ros-tf function to convert coordinate system
* Refactor obj_reproj
  - delete needless comment
  - align indentation
  - delete trailing whitespace
  - fix comment
* Get camera intrinsic matrix information from topic
* Fix rebase result of obj_reproj
  This is just done by following:
  `$git checkout master
  ros/src/computing/perception/detection/packages/cv_tracker/nodes/obj_reproj/obj_reproj.cpp`
* Add visualize function to obj_reproj
* Improve reprojection accuracy
  - Fix unit handling bug
  - Use ros-tf function to convert coordinate system
* Refactor obj_reproj
  - delete needless comment
  - align indentation
  - delete trailing whitespace
  - fix comment
* Get camera intrinsic matrix information from topic
* Contributors: AMC, Manato Hirabayashi, Shohei Fujii, Yukihiro Saito, Yusuke FUJII, amc-nu, tange

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------
* Add dummy tracking node
* Contributors: Yukihiro Saito

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
* Added Instructions for SSD node
  How to setup SSD Caffe
* Node Name Change for cv tracker
* Added SSD node to CV Tracker
* Fix obj_reproj as it subscribed out of date topic (current_pose)
  As '/current_pose' is not used, subscribing operation and
  corresponding stuff (callback function, global variables ...etc) are
  deleted by this commit.
  Function and global variables rerated to GNSS data are also deleted as
  they are not required in obj_reproj anymore.
* Remove error message
* Accelerated obj_reproj
* Hotfix in range_fusion
* Add module graph tool
* modify obj_fusion andobj_reproj in order to use tracking ID
* Fixed bug(The klt trancking node publish same distance data)
* Remove needless compiling flags
* Correct typo
* Add frame_id to BoundingBoxArray
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* Add jsk library
  If catkin_make detect "jsk_recognition_msgs" package,
  obj_reproj will publish reprojection result by
  jsk_recognition_msgs::BoundingBoxArray format automatically.
* Modify to get camera ID from runtime manager
  * Make it possible to obtain camera ID in each node to subscribe topics
  under camera name space selectively
  * Get image size from sensor_msgs::Image instead of CameraInfo
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
* Removed *.orig files
* Merge remote-tracking branch 'origin/master' into synchrogazed
* Add sleep command to decrease CPU occupancy
* modify launch files in perception to add a pedestrian mode in the sync packege
* modify correct timestamp and timing to publish
* fixed a bug in obj_reprojection when it occured conflict
* Merge remote-tracking branch 'origin/fix_timestamp' into synchrogazed
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/rtmgr.py
  ros/src/util/packages/runtime_manager/scripts/rtmgr.wxg
* Merge remote-tracking branch 'origin/resolve_conflict' into synchrogazed
  Conflicts:
  ros/src/computing/perception/detection/packages/cv_tracker/launch/ranging.launch
  ros/src/computing/perception/detection/packages/cv_tracker/nodes/obj_reproj/obj_reproj.cpp
  ros/src/util/packages/runtime_manager/scripts/rtmgr.py
  ros/src/util/packages/runtime_manager/scripts/rtmgr.wxg
  ros/src/util/packages/runtime_manager/scripts/runtime_manager_dialog.py
* Correct calibration_camera_lidar dependnecy about message header
* Correct runtime manager dependencies
* Correct cv_tracker building
  - Remove undefined dependencies
  - Set valid dependencies
* Correct dependency name
* Resolve conflict of obj_reproj.cpp between master and synchrogazed
* Runtime Manager Computing tab, add Synchronization button
* Remove nonexistent dependencies
* Fix requirement
* Improve accuracy of obj_reproj
  * Fix distance unit calculation error
  * Get intrinsic matrix from /camera/camera_info topic
  * Use ROS TF function to convert coordinate system
* Fix bug
  Add scale to markers
* Fix ranging.launch
* Publish reprojection result as ROS Marker
* Unify how to receive /image_raw
  In order to use both rgb8 and bayer image format of /image_raw
* Modify launch files to specify source camera
* modified klt to support new sync
* modify subscribed topic name in range fusion
* modify correct timestamp in range fusion
* kf and klt modified as asked.
* Publish obj_label immediately when source topics are subscribed
* Add flags to confirm multiple topics are subscribed
  - When topic's callback is called, corresponding flag is turned true
  - Result topic is published only when all flags are true
* Modify correct timestamps
* Modify correct lisence.
  All codes in the range_fusion was written by Nagoya University
* change publish timing in range fusion
* Better ID tracking using frame count
* Make projection matrix source selectable
  I modified nodes that subscribe /projection_matrix
  so that we can specify the topic name from launch file
* KLT based Multi Tracking
  -Added Launch file access from RTM
  -Modified ImageViewer to show circles instead of rectangles
* klt changes
* changes
* Update to KLT
* KLT updates
* KLT
* Klt Code Re arranging.
* Make image source selectable from launch file
  In order to use multiple camera, I modified
  - all viewers
  - some cv_tracker's packages and a library
  - lane_detector package
  - some road_wizard package
  so that input image topic can be changed easily from launch file
* KLT based Multi Tracking
  -Added Launch file access from RTM
  -Modified ImageViewer to show circles instead of rectangles
* Merge branch 'master' of https://github.com/CPFL/Autoware into klt_continued
* Fix include path and linked library setting issue on Ubuntu 15.04
  Paths of header files and libraries of libhdf5 and CUDA on Ubuntu 15.04 are
  different from Ubuntu 14.04. And those paths are set explicitly at compiling
  time on Ubuntu 15.04.
  And clean up CMake codes by using CMake and pkg-config features instead of
  absolute paths.
* Merge pull request `#81 <https://github.com/CPFL/Autoware/issues/81>`_ from CPFL/rcnn
  Integration of RCNN object detection on Autoware
* Updated to compile rcnn only if caffe and fast rcnn are installed
* klt changes
* changes
* Update to KLT
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* KLT updates
* KLT
* Klt Code Re arranging.
* Removed local references
  added $ENV{HOME} as suggested.
* Added files for RCNN node
* Integration of RCNN object detection on Autoware
  **Added a new library librcnn, which executes the object recognition using the Caffe framework, specifically the fast-rcnn branch.
  git clone --recursive https://github.com/rbgirshick/fast-rcnn.git
  -Requires CUDA for GPU support.
  To take advantage of cuDNN, at least CUDA 7.0 and a GPU with 3.5 compute capability is required.
  -Compile Caffe, located in caffe-fast-rcnn.
  Complete the requisites:http://caffe.berkeleyvision.org/install_apt.html
  -Download the pretrained models:
  http://www.cs.berkeley.edu/~rbg/fast-rcnn-data/voc12_submission.tgz
  -Modify the CMakeFiles and point them to your caffe and models directories.
  **Modified KF to use the new NMS algorithm
  **Modified Range fusion, it will not execute unnecesary fusions.
  **Added Configuration Messages to Runtime manager and RCNN node launch files
* Fix circular dependency
* Fix no-GPU platform issue
  'use_gpu' must not be used on no-GPU platform.
* Fixed topic names to relative ones
* Updated point2image to set minh in the message
* updated fusion to optionally read from any points to image projected topic via argument points_node.
  default  topic vscan_image (not changed)
  updated points2image topic to optionally project any pointcloud2 topic via argu
  ment point_node.
  default topic: points_raw (not changed)
* Fixed spell miss and minor update
* Modified dpm_ocv so that making executing CPU, GPU, car detection and pedestrian detection selectable
* Moved dpm_ocv.launch from runtime_manager/scripts to cv_tracker/launch
* Removed unnecessary files
* Update dpm_ocv
  - support using both GPU and CPU
  - clean up code
* Initial commit for public release
* Contributors: AMC, Abraham Monrroy, Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi, Yukihiro Saito, h_ohta, kondoh, pdsljp
