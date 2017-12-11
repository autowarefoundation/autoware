^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pointgrey
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2017-12-11)
------------------
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge branch 'develop' into feature/OpenPlanner
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* merge develop and fixed slow switching map
* Fixed mirrored images on Ladybug camera (`#906 <https://github.com/CPFL/Autoware/issues/906>`_)
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Merge branch 'develop' into feature/ndt_pcl_gpu
  Conflicts:
  ros/src/computing/perception/localization/packages/ndt_localizer/CMakeLists.txt
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_mapping/ndt_mapping.cpp
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_matching/ndt_matching.cpp
* Runtime manager updated to new package names (`#870 <https://github.com/CPFL/Autoware/issues/870>`_)
  [fix] Runtime manager updated to new pgrey package names
* Merge branch 'master' into feature/decision
* [Feature] Updates to Pointgrey package/Ladybug node (`#852 <https://github.com/CPFL/Autoware/issues/852>`_)
  * Added support for FindXerces on Indigo (and Cmake less than 3.1.3)
  Changed default scaling value to 20% of original image size (to improve performance)
  Changed published image format from RGB to Bayer(to reduce bag size)
  * Tested on real camera, updated
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* merge develop
* Merge branch 'develop' of https://github.com/CPFL/Autoware into feature/remote_monitor
* Contributors: Abraham Monrroy, Yamato ANDO, Yuki Iida, Yusuke FUJII, anhnv-3991, hatem-darweesh, yukikitsukawa

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------
* add web ui
* Fix cmake and package
* Fix build error
* Contributors: Akihito OHSATO, hironari.yashiro

1.4.0 (2017-08-04)
------------------

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------

1.2.0 (2017-06-07)
------------------

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
* Fix ladybug driver
* Add module graph tool
* Fix typo
* Add encoding check to use bayer image format
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Add ladybug node
* Initial commit for public release
* Contributors: Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi
