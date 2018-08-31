^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lidar_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2018-08-31)
------------------
* modified to output log
* add max_scan_range to ConfigNdtMapping/ConfigApproximateNdtMapping
* [Fix] Moved C++11 flag to autoware_build_flags (`#1395 <https://github.com/CPFL/Autoware/pull/1395>`_)
* [Feature] Makes sure that all binaries have their dependencies linked (`#1385 <https://github.com/CPFL/Autoware/pull/1385>`_)
* [bugfix] Static tf broadcaster in ndt_mapping
  Description
  When ndt_mapping node is launched and points_callback() function is called, br.sendTransform() command is not executed and thus transform from map frame to base_link frame is not published.
* Contributors: Esteve Fernandez, alexampa, yukikitsukawa

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
* set default queue_size 10
* Fix queue sizes of pubs/subs for ensuring real-time
* Contributors: Akihito OHSATO, yukikitsukawa

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
* Prepare release for 1.6.0
* modified setResolution to be called before setInputTarget is applied
* use_fast_pcl set default false
* modified ndt_cpu complie option
* fix typo
* use header.frame_id included in initialpose topic
* fix tf_mapping regarding number of digits of pose
* fixed CMakeLists and compile error
* separate executable
* change specification according PCL ver.
* Fix redeclaration error when compiling ndt_mapping.cpp and ndt_matching.cpp
* Add ndt_cpu library
* [fix] NDT_GPU ndt_localizer (`#854 <https://github.com/cpfl/autoware/issues/854>`_)
  * fix CMakeLists.txt of ndt_localizer
  * Fixed CUDA/FAST_PCL conflict
  * Fixed ndt_matching
* apply clang-format
* remove inline functions
* fix calculation of ndt_pose from localizer_pose
* add approximate_ndt_mapping (`#811 <https://github.com/cpfl/autoware/issues/811>`_)
* set use_gpu false by default
* add ndt_gpu in fast_pcl library
* add cuda inplementation of ndt algorithm in pcl
* apply clang-format
* bug fix
* Add thread_func.]
* remove currnet_pose publisher
* Contributors: Akihito Ohsato, Yamato ANDO, Yuki Kitsukawa, anhnv-3991, kitsukawa, yukikitsukawa

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
* add map_extender
* Contributors: Yusuke FUJII, yukikitsukawa

1.4.0 (2017-08-04)
------------------
* version number must equal current release number so we can start releasing in the future
* added changelogs
* Contributors: Dejan Pangercic

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* ndt_mapping ndt_matching, fix typo
* convert to autoware_msgs
* Contributors: YamatoAndo

1.2.0 (2017-06-07)
------------------
* fix a typo
* ndt with imu and odom for predict pose
* add imuUpsideDown() and params
* not use imu linear accleration y and z
* ndt_matching debug end (ndt_mapping not yet)
* NDT with imu(not yet)
* ndt with imu (temporary)
* ROS Kinectic Upgrade tested on Ubuntu 16.04 and OpenCV 3.2.0
  Modules not included:
  -orb_localizer
  -dpm_ocv node and lib
  Everything else working
  Added some libraries for Gazebo on src/extras
* Update for kinetic
* Contributors: YamatoAndo, Yukihiro Saito, amc-nu

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------
* add tf_mapping
  select points_topic in points_downsample.launch
* switch to output=log
* add max_iterations, min_scan_range and min_add_scan_shift
* add max_iterations
* Apply clang-format.
* Modifed the origin of the map.
* Contributors: yukikitsukawa

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
* Apply clang-format.
* Add param use_local_transform.
* send transform in global coordinates
* add ndt_tku library
* eliminate warning
* Add ndt_mapping_tku.
* Integrate ndt_matching_tku.
* Add ndt_matching_tku.
* Add get height
  If 'Get Height' checkbox is enabled on ndt_matching, you get height of initial pose by 2D Pose Estimate.
  This is height of nearest point in /points_map.
* Switch output from screen to log
* Fix ndt_mapping
  Improve CMakeLists.txt
* Set precision of log file of ndt_matching
* Modified file name of log for ndt_matching/icp_matching.
* Update interface.yaml of ndt_localizer, icp_localizer and points_filter
* Add measuring align_time and getFitnessScore_time.
  Fix warnings.
* ndt_mapping, lazy_ndt_mapping support OpenMP.
* Add ifdef for PCL 1.7.1
* Switch use_openmp true/false in ndt_matching.launch
* Prallelized ndt_matching
* Add module graph tool
* Use fast_pcl only when pcl 1.7.2 or higher version is installed
  pcl package of Ubuntu 14.04 is version 1.7.1 and some header files
  which are included in fast_pcl are missed in pcl 1.7.1.
* Fix deprecated code
  std::basic_ios does not implement 'operator void*' in C++11 specification.
  But GCC 4.8 still supports it with '-std=c++11' option, so there is no
  problem until now. However newer GCC removes it and we should use
  'operator !' or 'operator bool' instead of 'operator void*' after C++11.
* Add fast_pcl library.
* Add lazy_ndt_mapping.
  Add checkbox for lazy_ndt_mapping in Computing tab.
* Set input target only when map has been updated.
  Remove urdf directory in ndt_localizer.
* Fix TF tree of ndt_mapping.
* Remove unnecessary parameters from config window of ndt_matching.
* Apply clang-format.
* Changed directory structure.
  Add PointsFilterInfo.msg.
  Modified to publish points_filter_info.
* Rename directory (filter->points_filter).
  Add queue counter for ndt_mapping.
* Modified to select how to calculate offset for first matching iteration.
  Rename variables.
* Bug fix of distance_filter.
  Add random_filter.
  Modified ndt_matching to subscribe /filtered_points instead of /points_raw.
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* No publish /current_pose, Publish estimated_vel(geometry_msgs/Vector3Stamped)
* Change variable name (velodyne_sub-> scan_sub)
* Modified to switch localizer (lidar) easily.
* Combine velodyne_callback and hokuyo_callback into scan_callback.
* Improve ndt_mapping.
* Modified current_scan_time.
* Modified current_scan_time.
* Runtime Manager Computing tab, add Synchronization button
* Support quick_start.
  Modified not to use transform_pointcloud.
* Modified ndt_matching.launch. (output="screen"->"log")
* ndt_matching supports setup tab.
* Modified TF tree
  Add localizer_pose
* Modified ndt_matching.launch
* Update for integrated velodyne package
* Modified TF Tree.
  Before: world -> map -> velodyne -> base_link
  After: world -> map -> base_link -> velodyne
* bug fix , changed current pose to center of rear tires
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Set use_predict_pose off.
* Modified ndt_matching.launch to support 3D URG.
* Add predict_pose.
  Use predict_pose if predict_pose_error > 0.5.
  Specify timestamp of estimate_twist.
* Change topic type of ndt_stat.
* Remove unnecessary code.
* Change variable names.
  Clean the code.
  Add estimate_twist.
  Add ndt_stat.msg.
* Add ndt_stat.msg
* Developing for fail-safe.
* Publish /estimated_vel_mps and /estimated_vel_kmph.
* Improve local2global.cpp
* Initial commit for public release
* Switch output from screen to log
* Modified file name of log for ndt_matching/icp_matching.
* Update interface.yaml of ndt_localizer, icp_localizer and points_filter
* Add measuring align_time and getFitnessScore_time.
  Fix warnings.
* Fix deprecated code
  std::basic_ios does not implement 'operator void*' in C++11 specification.
  But GCC 4.8 still supports it with '-std=c++11' option, so there is no
  problem until now. However newer GCC removes it and we should use
  'operator !' or 'operator bool' instead of 'operator void*' after C++11.
* Remove a dependency of ndt_localizer.
  Add icp_stat.msg.
* Add missing ndt_localizer dependency
* Add checkbox of icp_matching to Computing tab.
  Add ConfigICP.msg.
* Parameter tuning.
* Add icp_localizer package.
* Contributors: Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi, Yukihiro Saito, h_ohta, kondoh, pdsljp, syouji, yukikitsukawa
