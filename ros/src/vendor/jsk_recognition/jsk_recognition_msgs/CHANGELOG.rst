^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_recognition_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.6 (2018-11-02)
------------------
* Install sample and test  into SHARE_DESTINATION (`#2345 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2345>`_)
* jsk_pcl_ros: support lazy mode for pointcloud_screenpoint nodelet (`#2277 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2277>`_)p
  * add 'bool no_update' to TransformScreenpoint.srv

* Contributors: Yuki Furuta, Yuto Uchimi

1.2.5 (2018-04-09)
------------------

1.2.4 (2018-01-12)
------------------

1.2.3 (2017-11-23)
------------------

1.2.2 (2017-07-23)
------------------

1.2.1 (2017-07-15)
------------------

1.2.0 (2017-07-15)
------------------
* [ADD NEW MESSAGE] Add Object.msg, ObjectArray.msg to represent object property (`#2148 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2148>`_)
  * Message to represent object property (Object.msg/ObjectArray.msg)

* Enhance PeoplePoseEstimation2D (`#2162 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2162>`_
  * scripts/people_pose_array_to_pose_array.py: Visualize people 3D pose on rviz in sample

* Contributors: Kentaro Wada, Yuki Furuta

1.1.3 (2017-07-07)
------------------
* [jsk_perception] apply candidates node supports topic update (`#2143 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2143>`_)
  * update Label msg API
  * add Label and LabelArray msg
* Rewrite KinfuNodelet with some enhancements and new features (`#2129 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2129>`_)
  * Save kinfu mesh model with bbox and ground frame id
  * Create polygon mesh with bbox request in kinfu
  * Create jsk_recognition_msgs/TrackingStatus.msg and use it in  Kinfue
* [jsk_perception] PeoplePoseEstimation2D (`#2115 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2115>`_)
  * [jsk_recogntion_msgs/PoseArray] Add score
  * [jsk_perception/people_pose_estimation_2d] Modified type of PeoplePose.msg
  * [jsk_recognition_msgs] Add people_pose msgs
* Contributors: Kei Okada, Kentaro Wada, Shingo Kitagawa, Iori Yanokura

1.1.2 (2017-06-16)
------------------
* [jsk_recognition_msgs] add segment messages. (`#2047 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2047>`_ )
* Generate README by script (`#2064 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2064>`_ )
* Contributors: Kentaro Wada, Masaki Murooka

1.1.1 (2017-03-04)
------------------

1.1.0 (2017-02-09)
------------------

1.0.4 (2017-02-09)
------------------

1.0.3 (2017-02-08)
------------------
* Compute box overlap and publish it (`#1993 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1993>`_ )
  * add Accuracy.msg
* Contributors: Kentaro Wada

1.0.2 (2017-01-12)
------------------

1.0.1 (2016-12-13)
------------------

1.0.0 (2016-12-12)
------------------

0.3.29 (2016-10-30)
-------------------

0.3.28 (2016-10-29)
-------------------
* [Major Release] Copy jsk_pcl_ros/srv and  jsk_perception/srv files to jsk_recognition_msgs (`#1914 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1914>`_ )
* Copy deprecated srv files to jsk_recognition_msgs
  - jsk_pcl_ros/srv -> jsk_recognition_msgs/srv
  - jsk_perception/srv -> jsk_recognition_msgs/srv
  TODO
  - 1. Migrate current code for srv files in jsk_recognition_msgs
  - 2. Remove srv files in jsk_pcl_ros and jsk_perception
* Contributors: Kei Okada, Kentaro Wada

0.3.27 (2016-10-29)
-------------------

0.3.26 (2016-10-27)
-------------------
* Apply context to label probability (`#1901 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1901>`_)
* Contributors: Kentaro Wada

0.3.25 (2016-09-16)
-------------------

0.3.24 (2016-09-15)
-------------------

0.3.23 (2016-09-14)
-------------------

0.3.22 (2016-09-13)
-------------------

0.3.21 (2016-04-15)
-------------------

0.3.20 (2016-04-14)
-------------------

0.3.19 (2016-03-22)
-------------------

0.3.18 (2016-03-21)
-------------------

0.3.17 (2016-03-20)
-------------------

0.3.16 (2016-02-11)
-------------------

0.3.15 (2016-02-09)
-------------------

0.3.14 (2016-02-04)
-------------------
* [jsk_perception] BoundingBoxToRectArray and rect_array_to_image_marker.py
* [jsk_pcl_ros] Publish current tracking status (running or idle)
  from particle_fitler_tracking.
  And add some scripts to visualize them.
* [jsk_recognition_msgs] Add min/max fields to  PlotDataArray
* [jsk_recognition_msgs] Update PlotData message to support more 2d plotting
* Contributors: Ryohei Ueda

0.3.13 (2015-12-19)
-------------------

0.3.12 (2015-12-19)
-------------------
* [jsk_pcl_ros_utils] Introduce new package called jsk_pcl_ros_utils
  in order to speed-up compilation of jsk_pcl_ros
* Contributors: Ryohei Ueda

0.3.11 (2015-12-18)
-------------------

0.3.10 (2015-12-17)
-------------------

0.3.9 (2015-12-14)
------------------
* [jsk_perception] Add PolygonArrayColorHistogram
* Contributors: Ryohei Ueda

0.3.8 (2015-12-08)
------------------
* [jsk_pcl_ros] Add Failure flag to Torus message
* Remove types on docs for jsk_pcl_ros
  See http://docs.ros.org/indigo/api/jsk_recognition_msgs/html/index-msg.html for message types
* Contributors: Kentaro Wada, Ryohei Ueda

0.3.7 (2015-11-19)
------------------
* Merge pull request `#1276 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1276>`_ from mmurooka/add-octomap-contact
  [jsk_pcl_ros] Add octomap contact
* [jsk_recognition_msgs] Add VectorArray.msg
* add message for octomap_server_contact
* [jsk_recognition_msgs] Add new msg ClassificationResult
* [jsk_recognition_msgs] Sort msg files in CMakeLists.txt
* [jsk_recognition_msgs] Add WeightedPoseArray
* add new output msg for handle estimate
* Contributors: Kentaro Wada, Masaki Murooka, Ryohei Ueda, Yu Ohara

0.3.6 (2015-09-11)
------------------

0.3.5 (2015-09-09)
------------------

0.3.4 (2015-09-07)
------------------

0.3.3 (2015-09-06)
------------------

0.3.2 (2015-09-05)
------------------

0.3.1 (2015-09-04)
------------------

0.3.0 (2015-09-04)
------------------

0.2.18 (2015-09-04)
-------------------
* [jsk_recognition_msgs] Add script to convert
  jsk_recognition_msgs/PlotData into csv
* [jsk_pcl_ros] Add tool to visualize variance of raser scan
* Contributors: Ryohei Ueda

0.2.17 (2015-08-21)
-------------------
* [jsk_recognition_msgs/PolygonArray] Add lebels and likelihood for
  colorizing on rviz
* Contributors: Ryohei Ueda

0.2.16 (2015-08-19)
-------------------

0.2.15 (2015-08-18)
-------------------

0.2.14 (2015-08-13)
-------------------
* [jsk_recognition_msgs] Add value field to BoundingBox to represent likelihood
* [jsk_recognition_msgs] HistogramWithRange message to represent rich histogram
  data
* [jsk_pcl_ros] Add config topic to chain heightmap configuration
* [jsk_perception] Scripts for bof and its hist extractor
* Contributors: Kentaro Wada, Ryohei Ueda

0.2.13 (2015-06-11)
-------------------
* [jsk_perception] Use ImageDifferenceValue.msg instead of Float32Stamped.msg
* [jsk_recognition_msgs] Add Float32Stamped.msg
* Contributors: Kentaro Wada

0.2.12 (2015-05-04)
-------------------
* JSK Recognition Msg for handling Array of 2D Rects
* Contributors: iKrishneel

0.2.11 (2015-04-13)
-------------------

0.2.10 (2015-04-09)
-------------------

0.2.9 (2015-03-29)
------------------
* 0.2.8
* Update Changelog
* Contributors: Ryohei Ueda

0.2.8 (2015-03-29)
------------------

0.2.7 (2015-03-26)
------------------

0.2.6 (2015-03-25)
------------------

0.2.5 (2015-03-17)
------------------

0.2.4 (2015-03-08)
------------------
* [jsk_recognition_msgs] Add resolution to SimpleOccupancyGrid
* Contributors: Ryohei Ueda

0.2.3 (2015-02-02)
------------------
* add CATKIN_DEPENDS
* [jsk_recognition_msgs] Add new message for occupancy grid for more
  simple usage
* Contributors: Ryohei Ueda, Kei Okada

0.2.2 (2015-01-30)
------------------

0.2.1 (2015-01-30)
------------------

0.2.0 (2015-01-29)
------------------

0.1.34 (2015-01-29)
-------------------
* [jsk_pcl_ros, jsk_perception] Use jsk_recognition_msgs
* Contributors: Ryohei Ueda

0.1.33 (2015-01-24)
-------------------
* [jsk_recognition_msgs] Add jsk_recognition_msgs
* Contributors: Ryohei Ueda

0.1.32 (2015-01-12)
-------------------

0.1.31 (2015-01-08)
-------------------

0.1.30 (2014-12-24 16:45)
-------------------------

0.1.29 (2014-12-24 12:43)
-------------------------

0.1.28 (2014-12-17)
-------------------

0.1.27 (2014-12-09)
-------------------

0.1.26 (2014-11-23)
-------------------

0.1.25 (2014-11-21)
-------------------

0.1.24 (2014-11-15)
-------------------

0.1.23 (2014-10-09)
-------------------

0.1.22 (2014-09-24)
-------------------

0.1.21 (2014-09-20)
-------------------

0.1.20 (2014-09-17)
-------------------

0.1.19 (2014-09-15)
-------------------

0.1.18 (2014-09-13)
-------------------

0.1.17 (2014-09-07)
-------------------

0.1.16 (2014-09-04)
-------------------

0.1.15 (2014-08-26)
-------------------

0.1.14 (2014-08-01)
-------------------

0.1.13 (2014-07-29)
-------------------

0.1.12 (2014-07-24)
-------------------

0.1.11 (2014-07-08)
-------------------

0.1.10 (2014-07-07)
-------------------

0.1.9 (2014-07-01)
------------------

0.1.8 (2014-06-29)
------------------

0.1.7 (2014-05-31)
------------------

0.1.6 (2014-05-30)
------------------

0.1.5 (2014-05-29)
------------------

0.1.4 (2014-04-25)
------------------

0.1.3 (2014-04-12)
------------------

0.1.2 (2014-04-11)
------------------

0.1.1 (2014-04-10)
------------------
