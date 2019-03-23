^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package costmap_generator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------
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
* Contributors: Kosuke Murakami
