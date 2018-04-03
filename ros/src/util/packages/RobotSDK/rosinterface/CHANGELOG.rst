^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosinterface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Missing Compiler flags (`#897 <https://github.com/cpfl/autoware/issues/897>`_)
  [fix] Qt compilation errors missing Compiler flags on Indigo
* refactor CMakeLists.txt. use automoc, autouic and autorcc
* Contributors: Abraham Monrroy, Yamato ANDO

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
* Lidar segmentation (`#499 <https://github.com/cpfl/autoware/issues/499>`_)
  * Lidar tracker restructuration
  * Added points_preprocessor package, including; ground filtering and space filtering.
* Add missing dependencies
* Add ROS include path to 'include_directories'
* Clean up Qt5 configuration
  Use pkg-config as possible instead of absolute pathes.
* Initial commit for public release
* Contributors: Abraham Monrroy, Shinpei Kato, Syohei YOSHIDA
