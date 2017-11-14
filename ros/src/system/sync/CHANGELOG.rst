^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package synchronization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* fix build issues due to autoware_msgs
* Resolved merge conflict by new feature
* convert to autoware_msgs
* Contributors: YamatoAndo, Yusuke FUJII

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
* Add Single thread sync node
* Use template lib in sync package
* Fixed bug in time_monitor.msg. caused by float64 points_image
* Fix wrong namespace in sync package
* Add template sync lib
* Dealed with points_image
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* Remove garbage file
* Removed wrong dependency
* Merge remote-tracking branch 'origin/fix_timestamp' into synchrogazed
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/rtmgr.py
  ros/src/util/packages/runtime_manager/scripts/rtmgr.wxg
* Add obj_person mode in the sync package
* modify CMakeList.txt in synchronization package
* modify correct topic name in sync
* modify correct topic data in time monitor
* fixed bug for sync generator
* modify CMakeList in synchronization package to correct file names
* modify node, sbscribing topic and publishing topic names dor sync
* modify correct timestamp in range fusion
* modify subscribed topic names in time monitor
* fixed bugs of exclusive controll in sync\_*_node
* Fix compile error in the scan2image package
* Add cycle time into a time monitor topic
* Add time monitor topic in the time monitor node
* Add a time difference topic for visualizer in a sync drivers node
* Add time monitor node
* add a sync_drivers node for synchronization between sensors
* Create sync nodes(test version) for object detections
* Update dpm_ocv
  - support using both GPU and CPU
  - clean up code
* Initial commit for public release
* Contributors: Shinpei Kato, Syohei YOSHIDA, Yukihiro Saito
