^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package naive_motion_predict
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------
* [fix] not predicting objects at x<=0 or y<=0 (`#2122 <https://github.com/CPFL/Autoware/issues/2122>`_)
* [Feature] costmap generator (`#1774 <https://github.com/CPFL/Autoware/issues/1774>`_)
  * * Initial commit for visualization package
  * Removal of all visualization messages from perception nodes
  * Visualization dependency removal
  * Launch file modification
  * * Fixes to visualization
  * Error on Clustering CPU
  * Reduce verbosity on markers
  * Publish acceleration and velocity from ukf tracker
  * Remove hardcoded path
  * Updated README
  * updated prototype
  * Prototype update for header and usage
  * Removed unknown label from being reported
  * Updated publishing orientation to match develop
  * * Published all the trackers
  * Added valid field for visualization and future compatibility with ADAS ROI filtering
  * Add simple functions
  * * Reversed back UKF node to develop
  * Formatted speed
  * Refactor codes
  * Make tracking visualization work
  * Relay class info in tracker node
  * Remove dependency to jskbbox and rosmarker in ukf tracker
  * apply rosclang to ukf tracker
  * Refactor codes
  * Revert "apply rosclang to ukf tracker"
  * Revert "Remove dependency to jskbbox and rosmarker in ukf tracker"
  * Revert "Relay class info in tracker node"
  * delete dependency to jsk and remove pointcloud_frame
  * get direction nis
  * set velocity_reliable true in tracker node
  * Add divided function
  * add function
  * Sanity checks
  * Relay all the data from input DetectedObject
  * Divided function work both for immukf and sukf
  * Add comment
  * Refactor codes
  * Pass immukf test
  * make direction assisted tracking work
  * Visualization fixes
  * Refactor codes
  * Tracker Merging step added
  * Added launch file support for merging phase
  * lane assisted with sukf
  * * change only static objects
  * keep label of the oldest tracker
  * Static Object discrimination
  * Non rotating bouding box
  * no disappear if detector works
  * Modify removeRedundant a bit
  * initial commit for costmap generator
  * add vague stucture
  * add brief structure fot points2costmap
  * refactor codes
  * add sensor points costmap
  * add waypoint costmap
  * bug fix for wayarea2costmap
  * add simple structure for objects2costmap
  * add vague structure for waypoints2costmap
  * Replacement of JSK visualization for RViz Native Markers
  * add occupancy grid visualization
  * add objects costmap
  * add minimum height threshold for pointcloud
  * debug computing.yaml from objects_topic to objects_input
  * Add blurred costmap
  * Add comment on computing.yml
  * visualizing bug fix
  * Make sure object's cost is 100 and blurred outside of objects
  * add costmap_generator package
  * add unit tests
  * delete test launch file
  * Apply autoware ros clang
  * Add README
  * sync develop's readme
  * sync develop's code
  * add convex hull costmap
  * Relay ros header appropriately
  * change interaface for generating costmap from points
  * add test for points2costmap
  * Modify for how to pick up convex-hull points
  * add test
  * add test for objects2costmap
  * Added missing include
  * Added missing grid_map_ros dependency
  * Updated include
  * Re-strutured include folders
  * Generic folder name
  * Fix/costmap generator (`#2077 <https://github.com/CPFL/Autoware/issues/2077>`_)
  * segmentation fault in  CheckAssignPoints2GridCell
  * Remove redundant codes in test
  * Add some variables in SetUp
  * rename class
  * rename files
  * modify tests
  * Add scription in SetUp
  * Remove unnecesary in_object
  * Refactor test codes
* [fix] Install commands for all the packages (`#1861 <https://github.com/CPFL/Autoware/issues/1861>`_)
  * Initial fixes to detection, sensing, semantics and utils
  * fixing wrong filename on install command
  * Fixes to install commands
  * Hokuyo fix name
  * Fix obj db
  * Obj db include fixes
  * End of final cleaning sweep
  * Incorrect command order in runtime manager
  * Param tempfile not required by runtime_manager
  * * Fixes to runtime manager install commands
  * Remove devel directory from catkin, if any
  * Updated launch files for robosense
  * Updated robosense
  * Fix/add missing install (`#1977 <https://github.com/CPFL/Autoware/issues/1977>`_)
  * Added launch install to lidar_kf_contour_track
  * Added install to op_global_planner
  * Added install to way_planner
  * Added install to op_local_planner
  * Added install to op_simulation_package
  * Added install to op_utilities
  * Added install to sync
  * * Improved installation script for pointgrey packages
  * Fixed nodelet error for gmsl cameras
  * USe install space in catkin as well
  * add install to catkin
  * Fix install directives (`#1990 <https://github.com/CPFL/Autoware/issues/1990>`_)
  * Fixed installation path
  * Fixed params installation path
  * Fixed cfg installation path
  * Delete cache on colcon_release
* Fix license notice in corresponding package.xml
* check for valid object before prediction
* Contributors: Abraham Monrroy Cano, Kosuke Murakami, amc-nu

1.10.0 (2019-01-17)
-------------------
* Fixes for catkin_make
* Switch to Apache 2 license (develop branch) (`#1741 <https://github.com/CPFL/Autoware/issues/1741>`_)
  * Switch to Apache 2
  * Replace BSD-3 license header with Apache 2 and reassign copyright to the
  Autoware Foundation.
  * Update license on Python files
  * Update copyright years
  * Add #ifndef/define _POINTS_IMAGE_H\_
  * Updated license comment
* Use colcon as the build tool (`#1704 <https://github.com/CPFL/Autoware/issues/1704>`_)
  * Switch to colcon as the build tool instead of catkin
  * Added cmake-target
  * Added note about the second colcon call
  * Added warning about catkin* scripts being deprecated
  * Fix COLCON_OPTS
  * Added install targets
  * Update Docker image tags
  * Message packages fixes
  * Fix missing dependency
* Feature/perception visualization cleanup (`#1648 <https://github.com/CPFL/Autoware/issues/1648>`_)
  * * Initial commit for visualization package
  * Removal of all visualization messages from perception nodes
  * Visualization dependency removal
  * Launch file modification
  * * Fixes to visualization
  * Error on Clustering CPU
  * Reduce verbosity on markers
  * intial commit
  * * Changed to 2 spaces indentation
  * Added README
  * Fixed README messages type
  * 2 space indenting
  * ros clang format
  * Publish acceleration and velocity from ukf tracker
  * Remove hardcoded path
  * Updated README
  * updated prototype
  * Prototype update for header and usage
  * Removed unknown label from being reported
  * Updated publishing orientation to match develop
  * * Published all the trackers
  * Added valid field for visualization and future compatibility with ADAS ROI filtering
  * Add simple functions
  * Refacor code
  * * Reversed back UKF node to develop
  * Formatted speed
  * Refactor codes
  * Refactor codes
  * Refactor codes
  * Refacor codes
  * Make tracking visualization work
  * Relay class info in tracker node
  * Remove dependency to jskbbox and rosmarker in ukf tracker
  * apply rosclang to ukf tracker
  * Refactor codes
  * Refactor codes
  * add comment
  * refactor codes
  * Revert "Refactor codes"
  This reverts commit 135aaac46e49cb18d9b76611576747efab3caf9c.
  * Revert "apply rosclang to ukf tracker"
  This reverts commit 4f8d1cb5c8263a491f92ae5321e5080cb34b7b9c.
  * Revert "Remove dependency to jskbbox and rosmarker in ukf tracker"
  This reverts commit 4fa1dd40ba58065f7afacc5e478001078925b27d.
  * Revert "Relay class info in tracker node"
  This reverts commit 1637baac44c8d3d414cc069f3af12a79770439ae.
  * delete dependency to jsk and remove pointcloud_frame
  * get direction nis
  * set velocity_reliable true in tracker node
  * Add divided function
  * add function
  * Sanity checks
  * Relay all the data from input DetectedObject
  * Divided function work both for immukf and sukf
  * Add comment
  * Refactor codes
  * Pass immukf test
  * make direction assisted tracking work
  * Visualization fixes
  * Refacor codes
  * Refactor codes
  * Refactor codes
  * refactor codes
  * refactor codes
  * Refactor codes
  * refactor codes
  * Tracker Merging step added
  * Added launch file support for merging phase
  * lane assisted with sukf
  * Refactor codes
  * Refactor codes
  * * change only static objects
  * keep label of the oldest tracker
  * Static Object discrimination
  * Non rotating bouding box
  * no disappear if detector works
  * Modify removeRedundant a bit
  * Replacement of JSK visualization for RViz Native Markers
  * Added Models namespace to visualization
  * Naming change for matching the perception component graph
  * * Added 3D Models for different classes in visualization
  * 2D Rect node visualize_rects added to visualization_package
* Fix Ros/ROS naming convention
* Contributors: Abraham Monrroy Cano, Esteve Fernandez, amc-nu

1.9.1 (2018-11-06)
------------------

1.9.0 (2018-10-31)
------------------
* [Feature] Prediction package template (naive_motion_predictor) (`#1616 <https://github.com/CPFL/Autoware/issues/1616>`_)
* Contributors: Kosuke Murakami
