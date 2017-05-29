^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_rviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.25 (2015-10-10)
-------------------
* [jsk_rviz_plugins] Fix font size of PeoplePositionMeasurementArray
* [jsk_rviz_plugins] Add script for diagnostics sample
* [jsk_rviz_plugins] Compile PeoplePositionMeasurementArrayDisplay
* [jsk_rviz_plugins/VideoCapture] Check file permission to write correctly
* [jsk_rviz_plugins] Use readthedocs to document
* [jsk_rviz_plugins] Add index page for sphinx + readthedocs
* [jsk_rviz_plugins] Use jsk_recognition_utils instead of jsk_pcl_ros to
  speed up compilation
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.24 (2015-09-08)
-------------------
* [jsk_rviz_plugins/PolygonArrayDisplay] Fix compilation error because of
  the latest jsk_recongition_utils changes
* [jsk_rqt_plugins/TwistStamped] Fix duplicated delete
* [jsk_rviz_plugins] Allow width/height 0 image (fix segfault)
* [jsk_rviz_plugins/PolygonArray] Coloring by labels and likelihood fields
  of jsk_recognition_msgs/PolygonArray
* [jsk_rviz_plugins/TwistStamped] Decide circle thickness according to
  radius of circle
* [jsk_rviz_plugins/BoundingBoxArray] Normalize value color gradation
* [jsk_rviz_plugins/BoundingBoxArray] Update coloring method to support
  coloring by values and labels.
* [jsk_rviz_plugins] Remove footstep texts from rviz when reset the plugin
* [jsk_rqt_plugins] Add sample launch for PolygonArray
* [jsk_rviz_plugins/PolygonArray] Use enum property to choose coloring method
* [jsk_rviz_plugins/TfTrajectory] Use status property to show error rather than
  ROS_ERROR
* [jsk_rviz_plugins/RobotCommandInterface] Use smaller icon size
* [jsk_rviz_plugins] Use ~robot_command_buttons parameter to configure RobotCommandInterfaceAction
* [jsk_rviz_plugins/TFTrajectory] Initialize line width
* [jsk_rviz_plugins/TFTrajectory] Add movie link to README
* [jsk_rviz_plugins] A rviz plugin to visualize tf trajectory as path
* [jsk_rviz_plugins][OverlayImage] Automatically setup size with negative val
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.23 (2015-07-15)
-------------------
* [jsk_rviz_plugins/PoseArray] Clear pose array if checkbox is unchecked
* fix coords bug
* Contributors: Ryohei Ueda, Yu Ohara

1.0.22 (2015-06-24)
-------------------
* [jsk_rviz_plugins/OverlayImage] Support alpha channel if image_encoding
  is BGRA8 or RGBA8
* Contributors: Ryohei Ueda

1.0.21 (2015-06-11)
-------------------
* [jsk_rviz_plugins/PolygonArrayDisplay] Cleanup codes to be within 80 columns
* [jsk_rviz_plugins/BoundingBoxArray] Immediately apply change of attributes
* [jsk_rviz_plugins/BoundingBoxArray] Refactor codes by splitting processMessages into several functions
* [jsk_rviz_plugins/BoundingBoxArray] Use symmetrical radius for coordinates arrow
* [jsk_rviz_plugins/BoundingBoxArray] Fix coding style around if/else/for
* [jsk_rviz_plugins/BoundingBoxArray] Check if the size of box is nan
* [jsk_rviz_plugins/BoundingBoxArray] Fix indent to be within 80 columns
* Contributors: Ryohei Ueda

1.0.20 (2015-05-04)
-------------------
* [jsk_rviz_plugins] add rotate speed to pictogram
* [jsk_rviz_plugins] add String PopupMode for Pictogram
* [jsk_rviz_plugins] Make arrow nodes invisible as default in PolygonArrayDisplay not to show normal if no needed
* [jsk_rviz_plugins] Check size of BoundingBox
* Contributors: Ryohei Ueda, Yuto Inagaki

1.0.19 (2015-04-09)
-------------------
* [jsk_rviz_plugins] Fix initialization order in Plotter2DDisplay in order  to avoid call std::vector::resize with uninitialized length
* [jsk_rviz_plugins] Obsolate SparseOccupancyGridArray, it's replaced by SimpleOccupancyGridArray
* [jsk_rviz_plugins] Use jsk_pcl_ros/geo_util to reconstruct 3d
  information in SimpleOccupancyGridArrayDisplay
* [jsk_rviz_plugins] Add image of SimpleOccupancyGridArray
* [jsk_rviz_plugins] Support auto coloring in SimpleOccupancyGridArray
* [jsk_rviz_plugins] Support 4th parameter of plane coefficients in SimpleOccupancyGridArrayDisplay
* [jsk_rviz_plugins] Add SimpleOccupancyGridArrayDisplay
* [jsk_rviz_plugins] add tmp pose array display
* [jsk_rviz_plugins] Change plotter color from 30%
* add_mesh_model_in_transformable_marker
* [jsk_rviz_plugins] Do not update min/max value when re-enabling Plotter2D
* [jsk_rviz_plugins] Change color of plotter from 50 percent of max value
* [jsk_rviz_plugins] add showing coords option for bounding box array display
* [jsk_rviz_plugins] Add utility script to visualize difference between to tf frame on rviz
* [jsk_rviz_plugins] Check direction vector is non-nan in PolygonArrayDisplay
* [jsk_pcl_ros] Fix license: WillowGarage -> JSK Lab
* [jsk_pcl_ros] Fix install path and install headers
* [jsk_rviz_plugins] Do not show disabled properties of OverlayText, Plotter2D and PieChart
* [jsk_pcl_ros] Make overlay sample more faster
* [jsk_rviz_plugins] Change color from 60 percent of maximum value in PieChartDisplay and Plotter2DDisplay
* [jsk_rviz_plugins] Draw PieChart at the first time
* Remove rosbuild files
* [jsk_rviz_plugins] Update PieChartDisplay only if value changed
* [jsk_rviz_plugins] Do not change texture size and position in processMessage
* [jsk_rviz_plugins] Optimize PieChartDisplay, draw image in update() method instead of processMessage
* Contributors: Ryohei Ueda, Yu Ohara, Yuto Inagaki

1.0.18 (2015-01-30)
-------------------
* add depends to cv_bridge instaed of opencv2

1.0.17 (2015-01-29)
-------------------
* [jsk_rviz_plugins] Add TwistStampedDisplay
* [jsk_rviz_plugins] Use jsk_recognition_msgs
* update README file for mainly panels
* [jsk_rviz_plugins] Add document of PolygonArray display
* add publishing pointcloud information as overlay text
* add record action panel
* remove unused QLineEdit variable
* add normal option for torus display
* [jsk_rviz_plugins] Refactor PolygonArrayDisplay class
* [jsk_rviz_plugins] Add "Show Normal" to PolygonArrayDisplay
* add object fit operator panel
* Make torus more smooth and add beatiful parameter
* add torus array display
* Contributors: Ryohei Ueda, JSK Lab member, Yuto Inagaki

1.0.16 (2015-01-04)
-------------------
* [jsk_rviz_plugins] Fix namespace of TabletViewController
* [jsk_rviz_plugins] Fix namespace jsk_rviz_plugin -> jsk_rviz_plugins
* [jsk_rviz_plugins] Utility script to draw the number of samples during
  capturing data
* [jsk_rviz_plugins] Remove invalid codes of ScreenshotListenerTool
* [jsk_rviz_plugins] VideoCaptureDisplay Display to capture rviz as movie
* [jsk_rviz_plugins] ScreenshotListenerTool: A simple tool to listen to
  a service and save screenshot to specified file
* [jsk_rviz_plugins] Avoid Segmentation Fault when size 0 texture is
  specified

1.0.15 (2014-12-13)
-------------------
* Add new plugin and message to display array of pictograms
* Remove pictogram when the display is disabled
* Fix policy to move head using rviz: Do not consider movement of mouse,
  just use the position of the mouse. Because we cannot ignore
  network latency
* Fix several parameters suitable for surface
* Add panel for tablet demonstration
* Add view_controller_msgs
* Compute difference to mouse position
* Add TabletViewController to control robot from tablet using rviz
* Check texture is available or not when initializing CameraInfo
* Paster image on the bottom of the camera parameter pyramid
* Contributors: Ryohei Ueda

1.0.14 (2014-12-09)
-------------------
* Add more action to pictogram
* Add documentation about pictogram
* Do not rewrite texture if no need
* Add sample to visualize all the pictograms
* Add FontAwesome fonts and several improvements about font drawing:
  1) decide size of font according to font metrics
  2) do not re-write pictogram texture if no need
* Support deletion of pictogram
* Add color field to Pictogram.msg
* Add sample script for pictogram
* Add display to visualize pictogram
* fixed parameter namespace mismatch.
* set the components to align left
* added button for start_impedance_for_drill
* added service to check marker existence. added copy to marker operation.
* fix quatation signiture for function name in robot_command_interface.cpp
* refact and delete some unneeded includes
* add empty_service_call_interface
* add robot_command_interface
* Change the size of menu according to the change of title and fix
  position of the popup window if the window is larger than the rviz
* Use name for decomposed topic of motor_states_temperature_decomposer.py
* Change color of text according to the foreground color of PieChart
* Show value as string on Plotter2DDisplay
* Decompose joint_state's effort value and read the max value from robot_description
* Fix motor_state_decomposer.py
* Take title into account to decide the size of OverlayMenu
* compacting the panel with using tab
* move msg to jsk_rviz_plugins
* add depend on jsk_interactive_marker
* add transformable marker operator panel
* Coloring footstep by jsk_footstep_msgs::Footstep::footstep_group
* Show text on footstep to display left or right
* Separate 'OvertakeProperties' into 'Overtake Color Properties' and
  'Overtake Position Properties'
* Script to decompose MotorStates/temperature into std_msgs/Float32
* Contributors: Ryohei Ueda, Masaki Murooka, Yuto Inagaki

1.0.13 (2014-10-10)
-------------------
* Add "overtake properties" property to OverlayTextDisplay
* Call queueRender after opening/closing properties in Open/CloseAllTool
* Contributors: Ryohei Ueda

1.0.12 (2014-09-23)
-------------------

1.0.11 (2014-09-22)
-------------------
* Do not ues deprecated PLUGINLIB_DECLARE_CLASS
* Draw polygon as 'face' on PolygonArrayDisplay
* Use jsk_topic_tools::colorCategory20 to colorize automatically
* Add tool plugin to close/open all the displays on rviz
* Contributors: Ryohei Ueda

1.0.10 (2014-09-13)
-------------------
* Fix color of people visualizer by initializing color to sky blue
* Fix texture color of camera info by filling color value of texture image
* Fix caching of overlay textures of OverlayMenuDisplay to support
  changing menus
* add relay camera info node
* Add new plugin to visualize sensor_msgs/CameraInfo
* Ignore first message means CLOSE in OverlayMenuDisplay
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.9 (2014-09-07)
------------------

1.0.8 (2014-09-04)
------------------
* add enum menu to TargetVisualizer and PeoplePositionMeasurementDisplay
  to select the style of the visualizer
* do not depends on people_msgs on groovy
* add SimpeCircleFacingVisualizer class
* separate a code to draw visualizer into facing_visualizer.cpp
* add rviz plugin for face_detector
* cleanup package.xml of jsk_rviz_plugins
* Contributors: Ryohei Ueda

1.0.7 (2014-08-06)
------------------
* show "stalled" if no diagnostic message received in OverlayDiagnosticDisplay
* add utility class for Overlay: OverlayObject and ScopedPixelBuffer in overlay_utils.cpp
* spcify max/min values for the properties of Plotter2D
* fix color error when changing the size of the window of Plotter2D
* add offset to compute the absolute position of the grid
* Remove non-used color property in OverlayDiagnosticsDisplay
* Remove OverlayDiagnostic correctly (not remaining overlay texture).
* under line of the caption should be longer than the length of the
  caption in TargetVisualizer
* align the position of the text of TargetVisualizer to left
* add CancelAction and PublishTopic plugin to hydro of jsk_rviz_plugin
* add visualizer to visualize pose stamped with target mark
* Contributors: Ryohei Ueda

1.0.6 (2014-07-14)
------------------
* add new plugin to visualize diagnostic status on ovrelay layer
* hide movable text of DiagnosticDisplay at first
* support font size field in DiagnosticDisplay
* diagnostics namespace and frame_id fields of DiagnosticsDisplay is now
  selectable according to the current ROS topics
* support axis color to colorize SparseOccupancyGridMap
* use rviz::PointCloud to render jsk_pcl_ros::SparseOccupancyGridArray to optimize
* hotfix to fix the position of overlay text
* does not update scale if the dimension is same to the previous data in OccupancyGridDisplay
* implement rviz plugin to visualize jsk_pcl_ros::SparseOccupancyGridArray
* add QuietInteractiveMarker
* Contributors: Ryohei Ueda

1.0.5 (2014-06-29)
------------------
* add overlay camera display
* close overlay menu firmly
* add new rviz plugin: OverlayImage
  visualize sensor_msgs::Image as HUD on rviz 3D rendering window
* add new plugin: OverlayMenu
* Contributors: Ryohei Ueda

1.0.4 (2014-05-31)
------------------
* jsk_rviz_plugins: use depend tag add mk/rosbuild to build_depend
* update the initial parameter of FootstepDisplay
* add line width property to BoundingBoxArrayDisplay
* add new plugin: BoundingBoxArray for jsk_pcl_ros/BoundingBoxArray
* Contributors: Ryohei Ueda, Kei Okada

1.0.3 (2014-05-22)
------------------
* add normals param and change skip_rate to set Percentage

1.0.2 (2014-05-21)
------------------
* Fixes a moc generation error with boost >= 1.48
* add color which will be deviced by curvature

1.0.1 (2014-05-20)
------------------
* add README and images, modify some fails
* Contributors: Yuto Inagaki

1.0.0 (2014-05-17)
------------------
* show border as default. add auto coloring option to show
  clusters efficiently.
* decrease the number of the error messages from NormalDispaly
* Contributors: Ryohei Ueda

0.0.3 (2014-05-15)
------------------
* supress erro message of NormalDisplay
* depends to hark_msgs is no longer needed
* Contributors: Ryohei Ueda, Kei Okada

0.0.2 (2014-05-15)
------------------
* overlay sample for groovy
* make NormalDisplay work on catkin.
  add normal_visual.cpp to jsk_rviz_plugins.so
* fix for using ambient_sound
* rename the name of plugin from PolygonArrayDisplay to PolygonArray
* add rviz_plugins icons
* change the color of the pie chart according to the absolute value
* smaller size for the font and add new line to the text of diagnostics display
* add a bool property to toggle auto scale for Plotter2DDisplay
* Merge remote-tracking branch 'refs/remotes/origin/master' into add-auto-color-changing-feature-to-plotters
  Conflicts:
  jsk_rviz_plugins/src/plotter_2d_display.cpp
  jsk_rviz_plugins/src/plotter_2d_display.h
* add auto color change boolean property and max color to change
  the color according to the value
* add sample for overlay rviz plugins
* support DELETE action to disable OvelrayText
* use qt to draw OverlayText
* does not call setSceneBlending twice
* add caption to 2d plotter
* add margin to plotter
* does not create QPainter without argument to supress the warning message of "painter not activate"
* initialize `orbit_theta_` and check overflow of the value
* add `update_interval_` to control the time to update the chart
* do not delete movable text in when the widget is disabled, delete it in deconstructor
* does not plot a chart if rviz is invoked with the plotter plugin disabled
* add DiagnosticsDisplay
* call hide in the destructor of overlay widgets
* add text to show caption and value.
  in order to toggle caption, added new check box.
  as caption, use the widget name.
* implement piechart on rviz using overlay technique
* add showborder property to 2d rviz plotter
* add plotter2d plugin
* use non-static and uniq string for overlay object
* implement OverlayText display plugin
* compile overlay text display
* add OverlayText.msg
* delete unneeded wrench files
* delete unneeded effort related files
* Merge pull request `#23 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/23>`_ from aginika/add-normal-diplay
  Add normal diplay
* add color channel and style property
* update to display in rviz
* update norml_display
* add normal_displays and normal_visuals
* delete point_display.cpp and point_visual.cpp
* Add the line to make the code in hydro
* ignore lib directory under jsk_rviz_plugins
* add gitignore for jsk_rviz_plugins
* do not create .so file under src directory
* depends on rviz using <depend> tag, because rviz failed to detect plugins from jsk_rviz_plugins without depend tag
* remove duplicated include line from polygon_array_display.h
  this duplication and quates in #include line happens compilation error about
  moc file of qt4
* `#7 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/7>`_: add wxwidgets dependency to jsk_rviz_plugins
* add dummy jsk-rviz-plugins.test
* use rosdep name for rviz and actionlib_msgs
* rendering backside face
* enabling alpha blending for PolygonArray
* fixing catkin cmake and dependency
* adding plugin to visualize PolygonArray
* add depends to jsk_footstep_msgs
* clear cache when toggle the check box of Footstep
* adding rviz plugin to visualize footstep
* paint point black if color is not available
* add select_point_cloud_publish_action for publish select points (no color)
* select action using combobox
* change msg type to actionlib_msgs
* add panel to cancel action
* add jsk_rviz_plugin::PublishTopic and remove Effort, wrenchStamped, PointStamped
* add rviz panel to send empty msg
* comment out SOURCE_FILES waiting for Issue `#246 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/246>`_
* use EXTRA_CMAKE_FLAGS to check to use ROSBUILD
* add dependencies to jsk_hark_msgs
* fix: validateFloats should be class method
* fix strequal ROS_DISTRO env
* use ROS_Distributions instead of ROS_DISTRO for electric
* add ambient_sound for groovy
* write libjsk_rviz_plugins under {PROJECT_SOURCE_DIR}/lib for and add export rviz to packages.xml, for groovy/catkin compile
* add debug message
* remove LIBRARY_OUTPUT_PATH and use catkin_package
* fix version
* fix to install plugin_descriptoin.xml and libjsk_rviz_plugins.so
* add comments
* fix for electric
* change msg:hark_msgs/HarkPower -> jsk_hark_msgs/HarkPower
* support groovy/cmake compile
* fix typo jsk_rviz_plugin -> jsk_rviz_plugins
* add test
* add package.xml
* add grad property
* added display ambient sound power
* add robot_description property
* add effort/max_effort property
* fix set sample color value for any scale value
* support enable button for each joint `#3597460 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/3597460>`_
* remove color property
* fix when max_effort is zero, `#3595106 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/3595106>`_
* support scale for effort_plugin, `#3595106 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/3595106>`_
* update jsk_rviz_plugins
* add jsk_rviz_plugins
* Contributors: Shohei Fujii, Youhei Kakiuchi, Kei Okada, Yuto Inagaki, Satoshi Iwaishi, Ryohei Ueda, Yusuke Furuta
