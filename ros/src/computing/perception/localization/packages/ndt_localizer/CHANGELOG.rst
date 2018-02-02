^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ndt_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2018-01-20)
------------------

1.6.0 (2017-12-11)
------------------
* Merge pull request `#971 <https://github.com/CPFL/Autoware/issues/971>`_ from CPFL/fix/ndt_fastpcl_memoryleak
  modified setResolution to be called before setInputTarget is applied
* modified setResolution to be called before setInputTarget is applied
* Merge pull request `#970 <https://github.com/CPFL/Autoware/issues/970>`_ from CPFL/fix/ndt_fastpcl_memoryleak
  Fix not to use NDT (fast_pcl) to leak memory
* use_fast_pcl set default false
* Merge pull request `#878 <https://github.com/CPFL/Autoware/issues/878>`_ from CPFL/feature/ndt_pcl_gpu
  Feature/ndt pcl gpu
* modified ndt_cpu complie option
* Merge pull request `#960 <https://github.com/CPFL/Autoware/issues/960>`_ from CPFL/fix/initialpose_frame_id
  Use initialpose header.frame_id at initialposeCallback function
* fix typo
* use header.frame_id included in initialpose topic
* Merge branch develop into feature/ndt_pcl_gpu
* Merge pull request `#956 <https://github.com/CPFL/Autoware/issues/956>`_ from CPFL/fix/ndt_merge_master_to_dev
  Fix/ndt merge master to dev (ndt_localizer)
* Merge branch master into develop (ndt_localizer)
* Merge branch 'develop' into feature/tlr_turn_support
* Merge pull request `#954 <https://github.com/CPFL/Autoware/issues/954>`_ from CPFL/fix/tf_mapping
  Fix/tf mapping
* fix tf_mapping regarding number of digits of pose
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge branch 'develop' into feature/OpenPlanner
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Merge pull request `#913 <https://github.com/CPFL/Autoware/issues/913>`_ from CPFL/feature/ndt-multithread
  switching the map by self position [ndt_matching]
* fixed CMakeLists and compile error
* merge develop and fixed slow switching map
* Merge pull request `#904 <https://github.com/CPFL/Autoware/issues/904>`_ from CPFL/hotfix/ndt_separate_executable
  [Bugfix] The NDT executable file was divided into two
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* separate executable
* change specification according PCL ver.
* Fix redeclaration error when compiling ndt_mapping.cpp and ndt_matching.cpp
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Merge branch 'develop' into feature/ndt_pcl_gpu
  Conflicts:
  ros/src/computing/perception/localization/packages/ndt_localizer/CMakeLists.txt
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_mapping/ndt_mapping.cpp
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_matching/ndt_matching.cpp
* Add ndt_cpu library
* Merge branch 'master' into feature/decision
* [fix] NDT_GPU ndt_localizer (`#854 <https://github.com/CPFL/Autoware/issues/854>`_)
  * fix CMakeLists.txt of ndt_localizer
  * Fixed CUDA/FAST_PCL conflict
  * Fixed ndt_matching
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Merge pull request `#794 <https://github.com/CPFL/Autoware/issues/794>`_ from CPFL/feature/ndt_pcl_gpu
  Feature/ndt pcl gpu
* apply clang-format
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge branch 'develop' of github.com:CPFL/Autoware into feature/remote_monitor
* remove inline functions
* Merge for ndt_pcl_gpu
* Merge pull request `#832 <https://github.com/CPFL/Autoware/issues/832>`_ from CPFL/fix/ndt_matching
  fix calculation of ndt_pose from localizer_pose
* fix calculation of ndt_pose from localizer_pose
* merge develop
* merge develop
* add approximate_ndt_mapping (`#811 <https://github.com/CPFL/Autoware/issues/811>`_)
* set use_gpu false by default
* add ndt_gpu in fast_pcl library
* add cuda inplementation of ndt algorithm in pcl
* apply clang-format
* bug fix
* Add thread_func.]
* Contributors: AMC, Akihito Ohsato, Yamato ANDO, YamatoAndo, Yuki Iida, Yuki Kitsukawa, Yusuke FUJII, anhnv-3991, hatem-darweesh, kitsukawa, yukikitsukawa

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------
* add map_extender
* Contributors: yukikitsukawa

1.4.0 (2017-08-04)
------------------

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* Resolved merge conflict by new feature
* ndt_mapping ndt_matching, fix typo
* convert to autoware_msgs
* Contributors: YamatoAndo, Yusuke FUJII

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
* Merge pull request `#335 <https://github.com/CPFL/Autoware/issues/335>`_ from CPFL/new_integrate_pcl
  Add fast_pcl library.
  Use normal pcl if pcl version is 1.7.1 or less and use fast_pcl if pcl version is 1.7.2.
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
* Merge remote-tracking branch 'origin/master' into synchrogazed
  Conflicts:
  ros/src/computing/perception/localization/packages/ndt_localizer/launch/ndt_matching.launch
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_matching/ndt_matching.cpp
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
* Merge branch 'master' of https://github.com/CPFL/Autoware into ndt_matching_mod_tf_tree
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
* Contributors: Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi, Yuki Kitsukawa, Yukihiro Saito, h_ohta, kondoh, pdsljp, syouji, yukikitsukawa
