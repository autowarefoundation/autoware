^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package road_wizard
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2018-01-20)
------------------

1.6.0 (2017-12-11)
------------------
* Merge pull request `#967 <https://github.com/CPFL/Autoware/issues/967>`_ from CPFL/fix/qt_autouic
  [bugfix]use qt5_wrap_ui instead of autouic
* use qt5_wrap_ui instead of autouic
* Merge branch develop into feature/ndt_pcl_gpu
* Merge pull request `#953 <https://github.com/CPFL/Autoware/issues/953>`_ from CPFL/feature/tlr_turn_support
  Feature/tlr turn support, multi traffic signal support
* Changed the publishing time of all the signals
* Added support to publish result of multiple traffic signals according to the lane
  VectorMapServer Support to publish signals on current lane if current_pose and final_waypoints available
* Initial modifications to feat_proj, tlr, context and vector_map loader, server and client to support different types of traffic signals
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge branch 'develop' into feature/OpenPlanner
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Support for multi cameras in feat_proj node (`#930 <https://github.com/CPFL/Autoware/issues/930>`_)
  [fix] feat_proj node multi cam support
* merge develop and fixed slow switching map
* Merge pull request `#888 <https://github.com/CPFL/Autoware/issues/888>`_ from CPFL/feature/cmake_refactor
  refactor CMakeLists.txt. use automoc, autouic and autorcc
* Merge branch 'feature/cmake_refactor' of github.com:cpfl/autoware into feature/decision
* refactor CMakeLists.txt. use automoc, autouic and autorcc
* Merge branch 'master' into feature/decision
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* merge develop
* Merge branch 'develop' of https://github.com/CPFL/Autoware into feature/remote_monitor
* Contributors: AMC, Abraham Monrroy, Yamato ANDO, YamatoAndo, Yuki Iida, Yuki Kitsukawa, Yusuke FUJII, hatem-darweesh, yukikitsukawa

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------
* fixed cmake and package.xml for libvectormap
  moved headers into include/libvectormap since otherwise this otherwise can conflict with other files elsewhere.
* Contributors: Dejan Pangercic

1.4.0 (2017-08-04)
------------------
* Change specifing each msg to catkin_EXPORTED_TARGETS
* R.I.P.
* Contributors: Yusuke FUJII

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* fix build issues due to autoware_msgs
* Resolved merge conflict by new feature
* Add link of trained data for tlr
* Merge branch 'develop' into feature/TLR_by_machine_learning
* fix to free a NULL window
* Merge branch 'develop' into feature/TLR_by_machine_learning
* Enable building nodes on the kinetic setup
  -road_wizard
  -klt_tracker
* Add README for region_tlr_ssd
* Fix for latest specification
* Prepare for merge
  * Fix assumed SSD path in CMakeLists.txt
  * Change default path of trained model into package-internal directory
  * Remove `std::cerr` statements for debug
  * Add UI to boot `traffic_light_recognition_ssd.launch` from runtime-manager
* Update state_transition_matrix
* Fix bug of vector_map_server usage
* Re-implement filter of signal orientation
  Target traffic light should face to camera.
  In this modification, target light will be chosen if it faces to camera +- 50 degree in camera coordinate system
* Apply vector_map_server function
* Fix usage of std::string::substr
* Modify State Transition Matrix of TLR for more precise recognition
  * w.r.t `region_tlr`, just fixed comment
  * w.r.t `region_tlr_ssd`, applied manner of erring on the side of caution
* Add region_tlr_ssd
* Add roi_extractor.launch to make handling parameter easy
  And fixed small bug to get parameter from ROS private parameter server
* Add image size section into annotation xml to fit VOC style
* Add filter not to save similar images
* Modify output format in VOC data one
  * Change the directory structure
  * Output each annotation file separately in XML format
  * Add position coordinate of traffic light in a image into annotation
* Add base function of label_maker
  This version outputs label data into one CSV file.
  Label data only contains file name and its traffic light states.
  We need improve output data label format to apply RCNN easily.
* Add roi_extractor node
* Move context-regarding files as package header and library
* Extract context operations as class method for future shared-use
* convert to autoware_msgs
* Contributors: Manato Hirabayashi, YamatoAndo, Yusuke FUJII

1.2.0 (2017-06-07)
------------------
* Fixed compatibility issues with indigo
* fix circular-dependency
* ROS Kinectic Upgrade tested on Ubuntu 16.04 and OpenCV 3.2.0
  Modules not included:
  -orb_localizer
  -dpm_ocv node and lib
  Everything else working
  Added some libraries for Gazebo on src/extras
* Update for kinetic
* Contributors: Shohei Fujii, Yukihiro Saito, Yusuke FUJII, amc-nu

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
* Merge pull request `#452 <https://github.com/CPFL/Autoware/issues/452>`_ from CPFL/add-vector_map-library
  Add vector map library
* Fix codes to use map_file messages and old vector_map_info topics
* Publish the detection result when it is different from the previous one
* Modify region_tlr so that it publishes superimpose image as topic
  Superimpose result image is now available as topic named
  "/tlr_superimpose_image"
* Uncomment publish statement of region_tlr
  Now region_tlr node publishes "light_color" topic
  as traffic light recognition result which is
  subscribed by traffic_light_viewer
* Runtime Manager, update feat_proj for SIGINT termination
* Uncomment publish statement of region_tlr
  Now region_tlr node publishes "light_color" topic
  as traffic light recognition result which is
  subscribed by traffic_light_viewer
* Add module graph tool
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
* Eigen3 with Fallback on ros-catkin-modules
* Change frame ID according to used camera
* Modify launch files to specify source camera
* Add condition to ignore signals not for cars
* Add ifdef to control showing debug information
* Make camera_info source selectable
  I modified nodes that subscribe /camera/camera_info
  so that we can specify the topic name from launch file
* Make image source selectable from launch file
  In order to use multiple camera, I modified
  - all viewers
  - some cv_tracker's packages and a library
  - lane_detector package
  - some road_wizard package
  so that input image topic can be changed easily from launch file
* Remove sound player execution from traffic_light_recognition.launch
* Clean up Qt5 configuration
  Use pkg-config as possible instead of absolute pathes.
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Update region_tlr
  - Add area-size condition to remove false detection caused by background
  in ROI
  - Update STATE_TRANSITION_MATRIX so that state will be UNDEFINED if
  impossible state change happens
  - minor fix
* Update threshold
  Increased threshold value with regard to update state of detected
  traffic light color
* Fix traffic_light_detection
  I modified traffic_light_detection so that fragmented signal lamps are
  assembled in a context by its poleID
* Modified region_tlr so that we can switch displaying image of superimpose result
  And this commit achieves
  - resizable image window showing superimpose result,
  - making close button on the window usable
* Modified feat_proj so that we can adjust projection error from runtime manger
* Initial commit for public release
* Contributors: Jit Ray Chowdhury, Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA, TomohitoAndo, USUDA Hisashi, kondoh, syouji
