^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package viewers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* fix a build issue due to autoware_msgs on the Indigo
* Contributors: Yusuke FUJII

1.2.0 (2017-06-07)
------------------
* Fixed compatibility issues with indigo
* fix circular-dependency
* ROS Kinectic Upgrade tested on Ubuntu 16.04 and OpenCV 3.2.0
  Modules not included:
  -orb_localizer
  -dpm_ocv node and lib
  Everything else working
  Added some libraries for Gazebo on src/extras
* Update for kinetic
* Contributors: Shohei Fujii, Yukihiro Saito, Yusuke FUJII, amc-nu

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
* Add missing dependencies.
* Add module graph tool
* Modify to get camera ID from runtime manager
  * Make it possible to obtain camera ID in each node to subscribe topics
  under camera name space selectively
  * Get image size from sensor_msgs::Image instead of CameraInfo
* Correct points2image dependency
  This is not incomplete because point2image has cycle dependency.
* Unify how to receive /image_raw
  In order to use both rgb8 and bayer image format of /image_raw
* Modify launch files to specify source camera
* KLT based Multi Tracking
  -Added Launch file access from RTM
  -Modified ImageViewer to show circles instead of rectangles
* Klt Code Re arranging.
* Make image source selectable from launch file
  In order to use multiple camera, I modified
  - all viewers
  - some cv_tracker's packages and a library
  - lane_detector package
  - some road_wizard package
  so that input image topic can be changed easily from launch file
* KLT based Multi Tracking
  -Added Launch file access from RTM
  -Modified ImageViewer to show circles instead of rectangles
* modified scan2image and calibration_test for calibration_publisher
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Klt Code Re arranging.
* Make pointCloud topic selectable subscribed in points_image_viewer
* Modified image_d_viewe to make subscribed topic name being selectable
* fix-dependency
* Modified viewers to make them resizable and to make close button enable
* Initial commit for public release
* Contributors: AMC, Hiroki Ohta, Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi, Yukihiro Saito, yukikitsukawa
