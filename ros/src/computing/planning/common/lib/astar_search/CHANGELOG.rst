^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package astar_search
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------
* [Feature] Improve Hybrid A* planner (`#1594 <https://github.com/CPFL/Autoware/issues/1594>`_)
  * Delete obstacle_sim from astar_planner package, replaced to lidar_fake_perception
  * Modify package name, astar_planner -> waypoint_planner, and create astar_planner library package
  * Delete obstacle_avoid/astar* and modify its dependency to astar_planner library
  * Fix astar_navi with astar_planner library
  * Refactor astar_navi by separating HAstar library and fixing coodinate system
  * Rename obstacle_avoid -> astar_avoid and under refactoring
  * Fix cost function and configures
  * Fix backward search and refactor configurations
  * Apply clang-format
  * Refactor include
  * Fix typo and so on
  * Improve astar_avoid by incremental goal search
  * Apply clang-format
  * Revert package names
  * Fix package/code names
  * Update runtime_manager
  * Improve astar_avoid to execute avoidance behavior by state transition (by rebuild decision maker)
  * Fix PascalCase message names by `#1408 <https://github.com/CPFL/Autoware/issues/1408>`_
  * Remove obstacle_avoid directory
  * Fix default parameter for costmap topic
  * Fix warning and initialize condition
  * Remove use_avoidance_state mode (TODO: after merging rebuild decision maker)
  * Improve astar_avoid behavior by simple state transition and multi-threading
  * Apply clang-format
  * Fix replan_interval in astar_avoid
  * Add descriptions for paramters
  * Rename pkg name, astar_planner -> waypoint_planner
  * Fix param name
  * Fix avoid waypoints height
  * Fix parameter and formalize code
  * Add README for freespace/waypoint_planner
  * Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Fix CHANGELOG
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  * Fix astar_navi/README.md
  * Add License terms
  * Fix const pointer
  * Added unit test base
  * Restructured folders
  * Fix bug by adding AstarSearch reset
  * Fix WaveFrontNode initialization
  Co-Authored-By: aohsato <aohsato@gmail.com>
  * Fix variable name
  * Refactor threading
  * Re-adding lidar_fake_perception
  * Fix the condition to judge reaching goal
  * Add 'use_decision state' mode to transit avoidance state by decision_maker
  * Fix calcDiffOfRadian (if diff > 2pi)
  * Feature/test astar planner (`#1753 <https://github.com/CPFL/Autoware/issues/1753>`_)
  * Restructured folders
  * Added unit test base
  * Removed remaining folder
  * Test WIP
  * Added astar_util tests and base file for astar_search tests
  * Updated to ROS Cpp Style guidelines
  * Added test for SimpleNode constructor
  * Updated Copyright date
  * Added tests for astar algorithm
  * Added default constructor to WaveFront struct
  * Revert use_state_decision mode (94af7b6)
  * Fix costmap topic names by merging costmap_generator
* Contributors: Akihito Ohsato
