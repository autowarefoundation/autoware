^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package object_map
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.6.0 (2017-12-11)
------------------
* merge develop and fixed slow switching map
* Merge branch 'master' into feature/decision
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* merge develop
* Merge branch 'develop' of https://github.com/CPFL/Autoware into feature/remote_monitor
* Contributors: Yamato ANDO, Yuki Iida, Yusuke FUJII, yukikitsukawa

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------
* Add feature to put simulated obstacles in astar planner
* Contributors: TomohitoAndo

1.4.0 (2017-08-04)
------------------

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* Resolved merge conflict by new feature
* Add map offset parameters to points2costmap node
* Add dist_transform node
* convert to autoware_msgs
* Contributors: TomohitoAndo, YamatoAndo, Yusuke FUJII

1.2.0 (2017-06-07)
------------------
* fixed build issues
* fix circular-dependency
* Merge remote-tracking branch 'origin/develop' into feature/lidar_segmentation
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Update potential field
* Contributors: AMC, Shohei Fujii, Yukihiro Saito, Yusuke FUJII

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------
* Add missing dependency
* Change to use final object topic in potential field
* Cleaned potential field node
* Add vscan points in potential field
* Add potential field package
* Contributors: Yukihiro Saito, h_ohta

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
* Add parameter for subscribing topic
* Fix costmap orientation
* Fix to subscribe the new topic
* Ignore 0 ranges
* Fix cost calculation for unknown costs
* Change variable name to be easier to understand
* Fix calculation of index
* Remove needless nesting
* Modify calculation for costs
* Remove needless compiling flags
* Fix dependencies
* Remove unused header
* Initialize a previous position when declared
* Change variable type from integer to bool
* Impletement some functions as struct method
* Use call by value instead of call by reference with premitive data types
* Add license statement
* Remeve automatically generated comments
* Add semantics package
* Contributors: Syohei YOSHIDA, TomohitoAndo
