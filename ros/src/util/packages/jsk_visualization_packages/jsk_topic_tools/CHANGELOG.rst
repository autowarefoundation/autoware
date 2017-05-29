^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_topic_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.3 (2015-07-24)
------------------
* [jsk_topic_tools] Install missing executables
* [jsk_topic_tools/standalone_complexed_nodelet] Support if and unless
  fields and read parameter from ~nodelet_%lu as well as ~nodelet
* [jsk_topic_tools] Introduce new nodelet manager called
  standalone_complexed_nodelet.
  It reads nodelet clients from rosparam and launch them. It is a general
  model for nodelet like stereo_image_proc. It does not need different
  processes for manager/clients
* [jsk_topic_tools] Make advertise template method critical section in
  order to avoid race condition between advertise and connectionCallback
* [jsk_topic_tools] Add StringRelay nodelet to test DiagnosticNodelet class
* Contributors: Ryohei Ueda

2.0.2 (2015-07-07)
------------------
* [jsk_topic_tools] add install config directory
* [jsk_topic_tools] Add number of subscribers to diagnostic information
* [jsk_topic_tools/Relay] Add more readable diagnostic including last time it receives input topic
* [jsk_topic_tools/Relay] Add diagnostic information
* [jsk_topic_tools] Update default diagnostic message to be more useful
* Contributors: Yuki Furuta, Ryohei Ueda

2.0.1 (2015-06-28)
------------------
* [jsk_topic_tools] Add DeprecatedRelay nodelet for deprecated topics
* Contributors: Ryohei Ueda

2.0.0 (2015-06-19)
------------------

1.0.72 (2015-06-07)
-------------------
* [jsk_topic_tools] Add global nodehandle
* Contributors: Kentaro Wada

1.0.71 (2015-05-17)
-------------------
* [jsk_topic_tools] Add ~always_subscribe parameter to ConnectionBasedNodelet
  and DiagnosticNodelet to always subscribe input topics
* Contributors: Ryohei Ueda

1.0.70 (2015-05-08)
-------------------
* [jsk_topic_tools/Passthrough] Add ~request service like Snapshot
* Contributors: Ryohei Ueda

1.0.69 (2015-05-05)
-------------------
* [jsk_topic_tools] Shorter test duration for topic_buffer/hztest_chatter_update
* Contributors: Ryohei Ueda

1.0.68 (2015-05-05)
-------------------
* [jsk_topic_tools] Add log_utils.h to print with __PRETY_FUNCTION__
* Contributors: Ryohei Ueda

1.0.67 (2015-05-03)
-------------------
* [jsk_topic_tools] Do not subscribe input if no need in Passthrough nodelet
* [jsk_topic_tools] Remove non-used TransportHint from relay_nodelet
* Contributors: Ryohei Ueda

1.0.66 (2015-04-03)
-------------------

1.0.65 (2015-04-02)
-------------------

1.0.64 (2015-03-29)
-------------------
* [jsk_topic_tools] Publish timestamp from snapshot as it publishes ~output
* [jsk_topic_tools] Add ~stop service to force to stop publishing messages
* Contributors: Ryohei Ueda

1.0.63 (2015-02-19)
-------------------
* [jsk_topic_tools] Add Passthrough nodelet to relay topics during
  specified duration
* Contributors: Ryohei Ueda

1.0.62 (2015-02-17)
-------------------
* [jsk_topic_tools] Add ~latch option to snapshot nodelet
* Contributors: Ryohei Ueda

1.0.61 (2015-02-11)
-------------------
* [jsk_topic_tools] Fix snapshot to publish first message correctly
* [jsk_topic_tools] Add service interface to change output topic of relay node
* anonymous node
* add flatten mode for array type message
* remove space after ,
* add argument exception handler
* add csv exporter for rosbag
* Contributors: Yuki Furuta, Ryohei Ueda

1.0.60 (2015-02-03)
-------------------
* [jsk_topic_tools] add std_srvs

1.0.59 (2015-02-03)
-------------------
* [jsk_topic_tools] Add document about nodelet utility classes
* [jsk_topic_tools] Fix license: WillowGarage -> JSK Lab
* [jsk_topic_tools] Add documentation about color_utils.h
* Remove rosbuild files
* [jsk_topic_tools] Return true in service callback of snapshot nodelet
* [jsk_topci_tools] Fix heatColor function to return std_msgs::ColorRGBA
* [jsk_topic_tools] Add new utility to take snapshot of topic
* Contributors: Ryohei Ueda

1.0.58 (2015-01-07)
-------------------
* [jsk_topic_tools] Indigo test seems to be broken,
  so skip testing on indigo
* [jsk_topic_tools] Do not implement updateDiagnostic
  as pure virtual method
* Reuse isMasterAlive function across scripts which
  want to check master state
* Contributors: Ryohei Ueda

1.0.57 (2014-12-23)
-------------------
* Add function to compute heat color gradient
* Add new script: static_transform_pose_stamped. It looks like tf's
  satatic_transform_publisher but it re-publishes geometry_msgs/PoseStamped.
* Contributors: Ryohei Ueda

1.0.56 (2014-12-17)
-------------------

1.0.55 (2014-12-09)
-------------------
* added topic_buffer_periodic_test.launch and added argument to topic_buffer_client/server_sample.launch
* add mutex lock in callback and thread function
* enable to select periodic mode from server param
* enable to select periodic mode from server param
* send request periodic publish from client when rosparam is set
* add update periodically function
* Contributors: Yuki Furuta, Masaki Murooka

1.0.54 (2014-11-15)
-------------------

1.0.53 (2014-11-01)
-------------------
* add nodelet to check vital of topic
* Contributors: Ryohei Ueda

1.0.52 (2014-10-23)
-------------------
* Move several utilities for roscpp from jsk_pcl_ros
* Contributors: Ryohei Ueda

1.0.51 (2014-10-20)
-------------------

1.0.50 (2014-10-20)
-------------------
* use 300 for default message_num, rostopic hz uses 50000? https://github.com/ros/ros_comm/blob/indigo-devel/tools/rostopic/src/rostopic/__init__.py#L111
* use median instead of average
* Contributors: Kei Okada

1.0.49 (2014-10-13)
-------------------
* Fix location of catkin_package of jsk_topic_tools
* Contributors: Ryohei Ueda

1.0.48 (2014-10-12)
-------------------

1.0.47 (2014-10-08)
-------------------
* Install executables build as single nodelet
* LightweightThrottle does not subscribe any topics if no need
* fix mutex lock of relay node
* Do not subscribe topics until mux/output is subscribed
* Contributors: Ryohei Ueda

1.0.46 (2014-10-03)
-------------------
* Do not use sleep inside of lightweight_throttle

1.0.45 (2014-09-29)
-------------------

1.0.44 (2014-09-26)
-------------------

1.0.43 (2014-09-26)
-------------------

1.0.42 (2014-09-25)
-------------------

1.0.41 (2014-09-23)
-------------------
* Compile transform_merger on catkin
* Use PLUGINLIB_EXPORT_CLASS instead of deprecated PLUGINLIB_DECLARE_CLASS
* Contributors: Ryohei Ueda

1.0.40 (2014-09-19)
-------------------
* Add diagnostic utils from jsk_pcl_ros
* Contributors: Ryohei Ueda

1.0.39 (2014-09-17)
-------------------

1.0.38 (2014-09-13)
-------------------
* add new utility function colorCategory20 to jsk_topic_tools
* Contributors: Ryohei Ueda

1.0.36 (2014-09-01)
-------------------
* Add rosparam_utils.cpp: utility functions for ros parameters
* Contributors: Ryohei Ueda

1.0.35 (2014-08-16)
-------------------
* add nodelet.cmake to export utility cmake macro to
  compile nodelet libraries
* Contributors: Ryohei Ueda

1.0.34 (2014-08-14)
-------------------
* add new class: VitalChecker from jsk_pcl_ros
* Contributors: Ryohei Ueda

1.0.33 (2014-07-28)
-------------------
* compile time_acucmulator.cpp on rosbuild environment
* add depend to dynamic_tf_reconfigure
* Contributors: Ryohei Ueda, Yuto Inagaki

1.0.32 (2014-07-26)
-------------------
* fix compilation for jsk_topic_tools::TimeAccumulator
* Contributors: Ryohei Ueda

1.0.31 (2014-07-23)
-------------------
* add class TimeAccumulator to measure and accumurate time to jsk_topic_tools
* Contributors: Ryohei Ueda

1.0.30 (2014-07-15)
-------------------
* add tool to check the existence of ros nodes and publish them to diagnostics
* Contributors: Ryohei Ueda

1.0.29 (2014-07-02)
-------------------

1.0.28 (2014-06-24)
-------------------
* initialize variable in relay_nodelet
* shutdown subscriber if no need to publish message in relay nodelet
* Merge pull request #466 from garaemon/add-single-executable-for-nodelet
  Add single executables for nodelets of jsk_topic_tools
* add single executable files for each nodelet in jsk_topic_tools
* add test code for block nodelet
* add nodelet to BLOCK topic pipeline according to the number of the subscribers
* add nodelet to relay topic
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.27 (2014-06-10)
-------------------
* add nodelet to relay topic
* Contributors: Ryohei Ueda

1.0.26 (2014-05-30)
-------------------

1.0.25 (2014-05-26)
-------------------

1.0.24 (2014-05-24)
-------------------

1.0.23 (2014-05-23)
-------------------

1.0.22 (2014-05-22)
-------------------
* add new nodelet: HzMeasure to measure message rate
* display info in debug mode
* print ignoring tf
* Merge remote-tracking branch 'tarukosu/ignore-specific-transform' into ignore-specific-transform
* add output='screen'
* use joint_states_pruned_buffered instead of _update
* remap /joint_states to /joint_states_pruned_update
* add ignoreing tf config
* add launch file for send joint state and other tf
* prune velocity and effort in joint state
* ignoring tf designated in yaml
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.21 (2014-05-20)
-------------------

1.0.20 (2014-05-09)
-------------------

1.0.19 (2014-05-06)
-------------------

1.0.18 (2014-05-04)
-------------------

1.0.17 (2014-04-20)
-------------------

1.0.16 (2014-04-19)
-------------------

1.0.15 (2014-04-19)
-------------------

1.0.14 (2014-04-19)
-------------------

1.0.13 (2014-04-19)
-------------------

1.0.12 (2014-04-18)
-------------------

1.0.11 (2014-04-18)
-------------------

1.0.10 (2014-04-17)
-------------------
* change the length of the name field according to the topic now the script subscribes
* print topic name rather than topic index and prettier format
* add test launch file for topic_compare and run it on catkin and rosbuild
* add test script and do not run load_manifest, it's not required
* add topic_compare.py
* Contributors: Ryohei Ueda, Yuki Furuta

1.0.9 (2014-04-12)
------------------
* use ShapeShifter rather than ShapeShifterEvent
* fix for goovy SEGV
  * use ros::Subscriber's pointer
  * use topic_tools::ShapeShiter rather than ShapeShifterEvent
  * not call getPrivateNodeHandle so many times
* Contributors: Ryohei Ueda

1.0.8 (2014-04-11)
------------------

1.0.7 (2014-04-10)
------------------
* add documentation on nodelet xml
* Contributors: Ryohei Ueda

1.0.6 (2014-04-07)
------------------
* add a sample for mux nodelet and does not use mux nodehandle.
  not using mux NodeHandle is different from original mux in topic_tools.
  now private nodehandle, which is the name of nodelet instance,
  behaves as 'mux' name of mux/topic_tools.
  If you want to use mux_** tools, you just specify nodelet name as mux name.
* implement nodelet version of mux with the same api to topic_tools and no need to specify the
  message type as well as topic_tools/mux
* add rostopic dependency to run test for LightweightThrottle
* update documentation of nodelet xml
* add test code for LightwehgitThrottle
* add a sample launch file for LightwehgitThrottle
* publish data only if any subscriber is
* compile nodelet on rosbuild too
* fixing dependency for nodelet usage
  depends to nodelet on manifest.xml, package.xml and catkin.cmake
* add xml declaration for nodlet plugin
* read update_rate from the parameter ~update_rate
* implement lightweight nodelet throttle
* add lightweight nodelet throttle skelton cpp/header file
* change arg name and node name
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.4 (2014-03-27)
------------------
* move the location of generate_messages and catkin_package to avoid emtpy
  catkin variables problem caused by roseus. it's a hack.
* Contributors: Ryohei Ueda

1.0.3 (2014-03-19)
------------------

1.0.2 (2014-03-12)
------------------
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: fix typo: dependp -> depend
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: add depend tag to jsk_topic_tools/manifest.xml because of previous breaking change of manifest.xml
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: replace .test suffix with .launch in jsk_topic_tools' rosbuild cmake
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: add full path to rostest of ros_topic_tools
* Contributors: Ryohei Ueda

1.0.1 (2014-03-07)
------------------
* set all package to 1.0.0
* Contributors: Kei Okada

1.0.0 (2014-03-05)
------------------
* set all package to 1.0.0
* fix typo CATKIN-DEPEND -> CATKIN_DEPEND
* add install to catkin.cmake
* (kill_server_and_check_close_wait.py) num=1 is ok for test_close_wait_check?
* add rostest and roscpp_tutorials
* use rosdep instead of depend
* add rostest
* add description in topic buffer sample program
* add buffer client and server for tf
* merge transform message to publish at low rate
* add sample launch files for specific transform
* do not initialize pub_update in use_service mode and restart serviceClient if sc_update.call failed, fixed Issue `#266 <https://github.com/jsk-ros-pkg/jsk_common/issues/266>`_
* rename to test_topic_buffer_close_wait.launch and add kill_server_and_check_close_wait.py
* add test launch for CLOSE_WAIT problem
* fixing output of ROS_INFO
* supporting topicized /update and parameterized /list
* fix test code chatter_update only publish every 10 min
* update topic_buffer_server/cliet, client automatically calls /update service to get latest information on server side ,see Issue `#260 <https://github.com/jsk-ros-pkg/jsk_common/issues/260>`_
* support update_rate param to configure how often client calls /update, see issue `#260 <https://github.com/jsk-ros-pkg/jsk_common/issues/260>`_
* client to call update to get current information on publish rate
* add rosbuild_add_rostest
* fix output message
* fix problem reported on `#260 <https://github.com/jsk-ros-pkg/jsk_common/issues/260>`_, add test code
* add more verbose message
* add sample launch file using topic_buffer
* update for treating multiple tf
* wait until service is available
* add specific transform publisher and subscriber
* add fixed_rate and latched parameter
* make catkin to work jsk_topic_tools
* add update service in topic_buffer_server
* fix xml: catkinize jsk_topic_tools
* fix broken xml: catkinize jsk_topic_tools
* fix broken xml: catkinize jsk_topic_tools
* catkinize jsk_topic_tools
* add jsk_topic_tools
* Contributors: Ryohei Ueda, Kei Okada, youhei, Yusuke Furuta
