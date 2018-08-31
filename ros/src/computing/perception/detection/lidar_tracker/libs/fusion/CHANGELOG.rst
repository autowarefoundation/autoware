^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fusion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2018-08-31)
------------------
* [Fix] Moved C++11 flag to autoware_build_flags (`#1395 <https://github.com/CPFL/Autoware/pull/1395>`_)
* [Feature] Makes sure that all binaries have their dependencies linked (`#1385 <https://github.com/CPFL/Autoware/pull/1385>`_)
* Contributors: Esteve Fernandez

1.7.0 (2018-05-18)
------------------
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
* [Fix] rename packages (`#1269 <https://github.com/CPFL/Autoware/pull/1269>`_)
  * rename lidar_tracker
  * Modify pf_lidar_track's cmake file
  * Refactor code
  * Rename from euclidean_lidar_tracker to lidar_euclidean_track
  * Rename from kf_contour_track to lidar_kf_contour_track
  * Rename from kf_lidar_track to lidar_kf_track, but need some modification in euclidean cluster(Cluster.h)
  * Rename from pf_lidar_tarck to lidar_pf_track
  * Rename range_fusion
  * Rename obj_reproj
  * Rename euclidean_cluster to lidar_euclidean_cluster_detect
  * Rename svm_lidar_detect to lidar_svm_detect
  * Rename kf_lidar_track to lidar_kf_track
  * Change version 1.6.3 to 1.7.0 in pacakge.xml
  * Modify CMake so that extrenal header would be loaded
  * Remove obj_reproj from cv_tracker
  * Add interface.yaml
  * Rename road_wizard to trafficlight_recognizer
  * create common directory
  * Add lidar_imm_ukf_pda_track
  * create vision_detector and moved cv
  * Modify interface.yaml and package.xml
  * remove dpm_ocv
  * moved directory
  * Delete unnecessary launch file
  * Delete rcnn related file and code
  * separated dummy_track from cv_tracker
  * separated klt_track from cv_tracker
  * Fix a cmake
  * Remove unnecessary dependency of lidar_euclidean_cluster_detect package
  * Rename image_segmenter to vision_segment_enet_detect
  * Remove unnecessary dependency of lidar_svm_detect package
  * separated kf_track and fix a some compiling issue
  * move viewers
  * merge ndt_localizer and icp_localizer, and rename to lidar_localizer
  * Remove unnecessary dependency of lidar_euclidean_track
  * moved image lib
  * add launch
  * lib move under lidar_tracker
  * Rename dpm_ttic to vision_dpm_ttic_detect
  * rename yolo3detector to vision_yolo3_detect
  * Modify cmake and package.xml in vision_dpm_ttic_detect
  * moved sourcefiles into nodes dir
  * moved sourcefiles into nodes dir
  * Move cv_tracker/data folder and delete cv_tracker/model folder
  * fix a package file and cmake
  * Rename yolo2 -> vision_yolo2_detect
  * fix a package file and cmake
  * Fix package name of launch file
  * Rename ssd to vision_ssd_detect
  * fixed cmake and package for decerese dependencies
  * remove top packages dir for detection
  * fixed cmake for cuda
  * Rename lane_detector to vision_lane_detect
  * Modify package.xml in lidar-related packages
  * Remove unnecessary dependencies in lidar_detector and lidar_tracker
  * Modify computing.yaml for dpm_ttic
  * Modify dpm_ttic launch file
  * Remove/Add dependencies to trafficlight_recognizer
  * Update data folder in dpm_ttic
  * Modified CMake and package file in dpm_ttic.
  * Remove src dir in imm_ukf_pda_track
  * removed unnecessary comments
  * rename lidar_tracker
  * Modify pf_lidar_track's cmake file
  * Refactor code
  * Rename from euclidean_lidar_tracker to lidar_euclidean_track
  * Rename from kf_contour_track to lidar_kf_contour_track
  * Rename from kf_lidar_track to lidar_kf_track, but need some modification in euclidean cluster(Cluster.h)
  * Rename from pf_lidar_tarck to lidar_pf_track
  * Rename range_fusion
  * Rename obj_reproj
  * Rename road_wizard to trafficlight_recognizer
  * Rename euclidean_cluster to lidar_euclidean_cluster_detect
  * Rename svm_lidar_detect to lidar_svm_detect
  * Rename kf_lidar_track to lidar_kf_track
  * Change version 1.6.3 to 1.7.0 in pacakge.xml
  * Modify CMake so that extrenal header would be loaded
  * Remove obj_reproj from cv_tracker
  * Add interface.yaml
  * create common directory
  * Add lidar_imm_ukf_pda_track
  * create vision_detector and moved cv
  * Modify interface.yaml and package.xml
  * remove dpm_ocv
  * moved directory
  * Delete unnecessary launch file
  * Delete rcnn related file and code
  * separated dummy_track from cv_tracker
  * separated klt_track from cv_tracker
  * Fix a cmake
  * Remove unnecessary dependency of lidar_euclidean_cluster_detect package
  * Rename image_segmenter to vision_segment_enet_detect
  * Remove unnecessary dependency of lidar_svm_detect package
  * separated kf_track and fix a some compiling issue
  * move viewers
  * merge ndt_localizer and icp_localizer, and rename to lidar_localizer
  * Remove unnecessary dependency of lidar_euclidean_track
  * moved image lib
  * add launch
  * lib move under lidar_tracker
  * Rename dpm_ttic to vision_dpm_ttic_detect
  * rename yolo3detector to vision_yolo3_detect
  * Modify cmake and package.xml in vision_dpm_ttic_detect
  * moved sourcefiles into nodes dir
  * moved sourcefiles into nodes dir
  * Move cv_tracker/data folder and delete cv_tracker/model folder
  * fix a package file and cmake
  * Rename yolo2 -> vision_yolo2_detect
  * fix a package file and cmake
  * Fix package name of launch file
  * Rename ssd to vision_ssd_detect
  * fixed cmake and package for decerese dependencies
  * remove top packages dir for detection
  * fixed cmake for cuda
  * Rename lane_detector to vision_lane_detect
  * Modify package.xml in lidar-related packages
  * Remove unnecessary dependencies in lidar_detector and lidar_tracker
  * Modify computing.yaml for dpm_ttic
  * Modify dpm_ttic launch file
  * Remove/Add dependencies to trafficlight_recognizer
  * Update data folder in dpm_ttic
  * Modified CMake and package file in dpm_ttic.
  * Remove src dir in imm_ukf_pda_track
  * Fix bug for not starting run time manager
  * Remove invalid dependency
* Contributors: Kosuke Murakami

1.6.3 (2018-03-06)
------------------

1.6.2 (2018-02-27)
------------------
* Update CHANGELOG
* Contributors: Yusuke FUJII

1.6.1 (2018-01-20)
------------------
* update CHANGELOG
* Contributors: Yusuke FUJII

1.6.0 (2017-12-11)
------------------

1.5.1 (2017-09-25)
------------------
* Release/1.5.1 (`#816 <https://github.com/cpfl/autoware/issues/816>`_)
  * fix a build error by gcc version
  * fix build error for older indigo version
  * update changelog for v1.5.1
  * 1.5.1
* Contributors: Yusuke FUJII

1.5.0 (2017-09-21)
------------------
* Update changelog
* Contributors: Yusuke FUJII

1.4.0 (2017-08-04)
------------------
* version number must equal current release number so we can start releasing in the future
* added changelogs
* Contributors: Dejan Pangercic

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* convert to autoware_msgs
* Contributors: YamatoAndo

1.2.0 (2017-06-07)
------------------
* fix circular-dependency
* Contributors: Shohei Fujii

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
* Fixed a bud. It caused by reference to out of image size
* Revert "Fixed a bud. It caused by reference to out of image size"
  This reverts commit 9d9568b650ab372883458bf42a93f13bab0be8e3.
* Fixed a bud. It caused by reference to out of image size
* Fix dependency
* Hotfix segmentation fault in range_fusion
* Add missing dependencies
* Add ROS include path to 'include_directories'
* Add new method for distance calculation of fusion
  Add new class to handle several distance calculation methods.
  We can now switch the calculation methos below by this class:
  * Shortest
  * Median
  * Mode (most common value)
  Confine scope of search_distance.h to lib/fusion
  Reconstruct search_distance.cpp
  * use function call instead of class method
  * fix typo
  * start function name with lower case
  * return 0 immediately if argument vector size is empty
  * use const reference as many as possible to avoid data copy
  Do not pass reference to std::sort
* Correct dependency name
* Delete warning cause
  * Use 'ranged-for' to reset std::vector
  * Set nested template parameter to C++11 style
* Change calculation method of ranging
  [Before] minimun distance in a detection rectangle
  [After]  median distance in a detection rectangle
* Delete image size fixing
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
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
* Updated point2image to set minh in the message
* fix-dependency
* Initial commit for public release
* Contributors: AMC, Hiroki Ohta, Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA, Yukihiro Saito, h_ohta
