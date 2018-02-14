^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosinterface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2018-01-20)
------------------

1.6.0 (2017-12-11)
------------------
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge branch 'develop' into feature/OpenPlanner
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* merge develop and fixed slow switching map
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Missing Compiler flags (`#897 <https://github.com/CPFL/Autoware/issues/897>`_)
  [fix] Qt compilation errors missing Compiler flags on Indigo
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
* Contributors: Abraham Monrroy, Yamato ANDO, YamatoAndo, Yuki Iida, Yusuke FUJII, hatem-darweesh, yukikitsukawa

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

1.2.0 (2017-06-07)
------------------
* Delete a line that cause an error in some environment
  This line cause an error like below
  ==========
  terminate called after throwing an instance of
  'ros::InvalidNameException'
  what():  Character [_] is not valid as the first character in Graph
  Resource Name [__name__points2vscan].  Valid characters are a-z, A-Z, /
  and in some cases ~.
  ==========
* Update RobotSDK
  This is required to port vehicle_tracker
* Delete a line that cause an error in some environment
  This line cause an error like below
  ==========
  terminate called after throwing an instance of
  'ros::InvalidNameException'
  what():  Character [_] is not valid as the first character in Graph
  Resource Name [__name__points2vscan].  Valid characters are a-z, A-Z, /
  and in some cases ~.
  ==========
* Update RobotSDK
  This is required to port vehicle_tracker
* Contributors: Manato Hirabayashi

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
* Lidar segmentation (`#499 <https://github.com/CPFL/Autoware/issues/499>`_)
  * Lidar tracker restructuration
  * Added points_preprocessor package, including; ground filtering and space filtering.
* Add missing dependencies
* Add ROS include path to 'include_directories'
* Clean up Qt5 configuration
  Use pkg-config as possible instead of absolute pathes.
* Initial commit for public release
* Contributors: Abraham Monrroy, Shinpei Kato, Syohei YOSHIDA
