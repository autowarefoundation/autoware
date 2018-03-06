^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package orb_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* fix dependencies
* Contributors: Dejan Pangercic, Yusuke FUJII

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* fix a build issue due to autoware_msgs on the Indigo
* Contributors: Yusuke FUJII

1.2.0 (2017-06-07)
------------------
* Reorganization and fix for Ubuntu 16.04
* clean directories
* fix circular-dependency
* ROS Kinectic Upgrade tested on Ubuntu 16.04 and OpenCV 3.2.0
  Modules not included:
  -orb_localizer
  -dpm_ocv node and lib
  Everything else working
  Added some libraries for Gazebo on src/extras
* Update for kinetic
* Contributors: Adi Sujiwo, Shohei Fujii, Yukihiro Saito, Yusuke FUJII, amc-nu

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
* Fix .gitignore in each packages
* Fix cmake error in orb_localizer
* Clarify location of header files
* Launch file; change orb_matching to suit multiple nodes
* Add dependencies in README
* Add framebuffer output from FrameDrawer
* Notes for mapping node
* Bug fix for ROS > indigo
* Remove explicit inclusion of indigo TF
* Regression in MapDrawer, please backport from previous version
* Add argument for downsampling
* downsamples.py: Utility for creating kitti-compatible dataset
* Reduce dependencies on config file
* orbndt.py: Support loading from PoseStamped messages
* Remove unneeded files
* orb_evaluator node
* Added dumpmap; orb_matching node.
* Added orb_matching
* Refactored orb_mapping
* orb_mapping separated
* Libraries compiled; Licenses checked
* Refactoring; add License for g2o
* Add License file for Pangolin
* Cleanup `#1 <https://github.com/cpfl/autoware/issues/1>`_
* Initial integration of ORB-SLAM2
* Add orb_localizer.
* ORB SLAM Fix with History
* ORB-SLAM Localizer with External Reference
* No need to flip axis after computing scale
* Renamed to ORB-Localizer. Modify the bag_mapper
* Contributors: AMC, Adi Sujiwo, h_ohta, yukikitsukawa
