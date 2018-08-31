^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package op_utilities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2018-08-31)
------------------
* Support old behavior of insert static object for obstacle avoidance testing
  Only one simulated car available in the runtime manager
  update for copywrite note
  insert autoware_build_flags to new nodes
* Update README.md
* Add README files for OpenPlanner packages
* Modify Map loading for OpenPlanner, now it reads from Autoware vector map messages, old behavior still works but from launch file only.
  Delete way_planner, dp_planner from UI, but they still accessible from roslaunch.
* Update op_utility files for csv files loading
  Update MappingHelpers with latest modifications
  Update PlanningHelpers with latest modifications
  add op_common_param node, for setting OpenPlanner parameter for all related nodes such as lidar_kf_contour_track
  Improve tracking by including size different in association function
  Update way_planner, dp_planner for compatibility with new Mapping Modifications, Map format is backward compatible
* Update OpenPlanner libraries (op_planner, op_utitity, op_ros_helpers)
  Update ring ground filter with latest implementation
  Update lidar_kf_contour_track with latest implementation
  Add op_utilities nodes (op_bag_player, op_data_logger, op_pose2tf)
  Modify autoware_msgs for OpenPlanner use (CloudCluster, DetectedObject, lane, waypoint)
  Update UI computing.yaml for the new nodes and modifies parameters
  Update UI sensing.yaml for updated ring_ground_filter params
* Contributors: Hatem Darweesh, hatem-darweesh
