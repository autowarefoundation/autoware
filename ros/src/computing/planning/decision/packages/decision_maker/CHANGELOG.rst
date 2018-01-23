^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package decision_maker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2018-01-20)
------------------

1.6.0 (2017-12-11)
------------------
* Merge branch develop into feature/ndt_pcl_gpu
* Merge branch 'develop' into feature/tlr_turn_support
* Merge pull request `#954 <https://github.com/CPFL/Autoware/issues/954>`_ from CPFL/fix/tf_mapping
  Fix/tf mapping
* R.I.P.
* fix a duplicate state update
* stop light_color publish every state update
* Merge branch 'feature/decision' of github.com:cpfl/autoware into feature/decision
* add new state for lanechange
* Merge branch 'develop' into feature/tlr_turn
* Merge branch 'develop' into feature/ndt_pcl_gpu
* fix a judge intersect stopline
* Merge branch 'feature/decision' of github.com:CPFL/Autoware into feature/decision
* add new state for all stopline pause
* fix a build bug
* fix a build bug
* fix a state num name
* fix a state num name
* add support to manual fixing light
* Merge pull request `#936 <https://github.com/CPFL/Autoware/issues/936>`_ from CPFL/feature/decision
  Feature/decision: Enhancement decision maker node
* fix a moving state
* fix a variable declaration was not enough.
* Add author tag
* Checked coding by cppcheck and apply clang-format
* Add new state
  - TrafficLight State (it will be planning to change "behavior" to
  another category)
  - Crawl(slow speed)
* Add to support multiple lane shift
* Remove debug code and apply clang format
* Fixed:
  - callback
  - laneshift
  Added:
  - publisher for laneid
  - new lanechange flag
  - new param for decisionMaker
* cosme
* add shifted lanes
* add simple obstacle avoid based shifting lane
* Fix not working changed callback
* :put_litter_in_its_place:
* add to insert shift lane
* delete build warning, and change stopline
* update state and remove detection state
* fix a build error
* merge develop and fixed slow switching map
* fix a judge left/right
* Fix a state changing bug
* add support to stopline
* Add feature of to find stopline. and following minor fixes
  - to change vectormap operation to vectormap lib.
  - to change state operation
* Merge branch 'feature/decision' of github.com:cpfl/autoware into feature/decision
* refactor lamp control
* apply clang-format
* Support to lanechange similar to state_machine(old) package
* Add support to switch on/off directional indicator
* remove unnecessary code from decisionmaker
* add support to waypoint velocity control by state
* add mps2kmph
* update decisionmaker and related library
  - add multiplelane path recognition
  - renamed euc
* Changed path state recognition to the way based on /lane_waypoints_array
* Merge branch 'master' into feature/decision
* improve a judge algorithms for right/left-turn in intersection
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Add to support manually decision
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* merge develop
* Merge branch 'feature/decision_maker' of github.com:cpfl/autoware into feature/remote_monitor
* Contributors: AMC, Yamato ANDO, Yuki Iida, Yuki Kitsukawa, Yusuke FUJII, yukikitsukawa

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
