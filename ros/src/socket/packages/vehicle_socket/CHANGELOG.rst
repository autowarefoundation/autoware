^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vehicle_socket
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2017-12-11)
------------------
* Merge branch 'develop' into feature/OpenPlanner
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* merge develop and fixed slow switching map
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Merge branch 'develop' into feature/ndt_pcl_gpu
  Conflicts:
  ros/src/computing/perception/localization/packages/ndt_localizer/CMakeLists.txt
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_mapping/ndt_mapping.cpp
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_matching/ndt_matching.cpp
* Fix build error
* Merge pull request `#844 <https://github.com/CPFL/Autoware/issues/844>`_ from CPFL/feature/remote_monitor
  Feature/remote monitor
* Merge branch 'master' into feature/decision
* refactor code
* refactor code
* Merge branch 'master' of github.com:cpfl/autoware into develop
* refactor msg and add blinker to msg
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* merge develop
* update vehicle sender to support twist gate
* Contributors: Akihito Ohsato, Yamato ANDO, Yuki Iida, Yusuke FUJII, anhnv-3991, hatem-darweesh, yukikitsukawa

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
* Rewrite vehicle_sender and autoware_socket for applying the topic /ctrl_cmd.
* Contributors: USUDA Hisashi

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
* Add module graph tool
* Add missing dependencies
* Add ROS include path to 'include_directories'
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* Correct runtime manager dependencies
* Publish mode_info
* Change data transfer format from Vehicle to ROS PC
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Initial commit for public release
* Contributors: Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi, syouji
