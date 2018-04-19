^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package synchronization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Prepare release for 1.6.0
* Contributors: Yamato ANDO

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
* fix build issues due to autoware_msgs
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
