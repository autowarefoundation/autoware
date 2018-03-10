^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package integrated_viewer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Add point-size-combo-box to image_viewer_plugin
  Now drawn point size can be specified from this combo box
* Make drawn points_image bigger in image_viewer_plugin
* convert to autoware_msgs
* Contributors: Manato Hirabayashi, YamatoAndo

1.2.0 (2017-06-07)
------------------
* fix a build issue for kinetic
* Add function to visualize lane_detect result
  Update image_viewer_plugin so that lane_detector result can be visualized
* Fixed compatibility issues with indigo
* enabled integrated viewer on the kinetic
* fix circular-dependency
* ROS Kinectic Upgrade tested on Ubuntu 16.04 and OpenCV 3.2.0
  Modules not included:
  -orb_localizer
  -dpm_ocv node and lib
  Everything else working
  Added some libraries for Gazebo on src/extras
* Update for kinetic
* Contributors: Manato Hirabayashi, Shohei Fujii, Yukihiro Saito, Yusuke FUJII, amc-nu

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
* Fix dependency
* Difference of Normals Segmentation added to the pipeline
* Difference of Normals Segmentation added to the pipeline
* Modify to update selectable topic automatically
  * Selectable topic list will be updated when any of combo box is opened
  * "Update topic list" button is abolished
* Add RViz integrated viewer plugin to display detection result
  Integrated viewer and Traffic Light plugins are available by:
  RViz Menu -> "Panels" -> "Add New Panel"  and
  select "ImageViewerPlugin" or "TrafficLightPlugin"
  in "integrated_viewer" section.
* Add RViz plugin to display traffic light recognition result
* Contributors: AMC, Manato Hirabayashi, h_ohta
