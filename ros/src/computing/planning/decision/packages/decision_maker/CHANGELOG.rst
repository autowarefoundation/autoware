^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package decision_maker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------
* Add build option
* apply clang-format
* update decision maker config
* Add to support dynamical parameter for decision_maker
* Add build flags
* fixed launch file for planner selector
* Fix a build error for decision maker node
* Move the decision part of the state machine library to decision_Maker node. This is WIP.
* fix a header dependencies and remove unnecessary part
* Change file composition
* Add publisher for target velocities
* Removed dynamic reconfigure
* Fixed forgetting to rename(state_machine node to decision_maker node)
* integrate planner_selector package to decision_maker package
* Add decision packages into runtime_manager
* apply clang-format
* organize package files and directories
* Add a decision_maker package
  The decision_maker package determines the intention of what actions the
  local planner and control node should take based on perception nodes,
  global planner nodes, map data, sensor data.
  This commit corresponds only to the following functions.
  - Behavior state recognition
  - Dynamic selection of local planner (It is necessary to change the topic name of local planner)
* Contributors: Yusuke FUJII

* Add build option
* apply clang-format
* update decision maker config
* Add to support dynamical parameter for decision_maker
* Add build flags
* fixed launch file for planner selector
* Fix a build error for decision maker node
* Move the decision part of the state machine library to decision_Maker node. This is WIP.
* fix a header dependencies and remove unnecessary part
* Change file composition
* Add publisher for target velocities
* Removed dynamic reconfigure
* Fixed forgetting to rename(state_machine node to decision_maker node)
* integrate planner_selector package to decision_maker package
* Add decision packages into runtime_manager
* apply clang-format
* organize package files and directories
* Add a decision_maker package
  The decision_maker package determines the intention of what actions the
  local planner and control node should take based on perception nodes,
  global planner nodes, map data, sensor data.
  This commit corresponds only to the following functions.
  - Behavior state recognition
  - Dynamic selection of local planner (It is necessary to change the topic name of local planner)
* Contributors: Yusuke FUJII

* Add build option
* apply clang-format
* update decision maker config
* Add to support dynamical parameter for decision_maker
* Add build flags
* fixed launch file for planner selector
* Fix a build error for decision maker node
* Move the decision part of the state machine library to decision_Maker node. This is WIP.
* fix a header dependencies and remove unnecessary part
* Change file composition
* Add publisher for target velocities
* Removed dynamic reconfigure
* Fixed forgetting to rename(state_machine node to decision_maker node)
* integrate planner_selector package to decision_maker package
* Add decision packages into runtime_manager
* apply clang-format
* organize package files and directories
* Add a decision_maker package
  The decision_maker package determines the intention of what actions the
  local planner and control node should take based on perception nodes,
  global planner nodes, map data, sensor data.
  This commit corresponds only to the following functions.
  - Behavior state recognition
  - Dynamic selection of local planner (It is necessary to change the topic name of local planner)
* Contributors: Yusuke FUJII
