^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scan2image
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* ROS Kinectic Upgrade tested on Ubuntu 16.04 and OpenCV 3.2.0
  Modules not included:
  -orb_localizer
  -dpm_ocv node and lib
  Everything else working
  Added some libraries for Gazebo on src/extras
* Contributors: amc-nu

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
* Add module graph tool
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* Merge remote-tracking branch 'origin/master' into synchrogazed
  Conflicts:
  ros/src/sensing/fusion/packages/scan2image/CMakeLists.txt
* Correct calibration_camera_lidar dependnecy about message header
* Fix compile error in the scan2image package
* Change init function from memset to fill_n
* fixed bug of scan2image
  the third argument in memset() is wrong
* remove MAX_IMAGE_HIGHT, MAX_IMAGE_WIDTH.
  ( add memset() instead of them )
* change from static variable to auto variable
  because variables (i, count) do not needed static var.
* fixed bug of init
* fixed calibration bug's & cosmetic changes
* modified scan2image and calibration_test for calibration_publisher
* Initial commit for public release
* Contributors: Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi, Yukihiro Saito
