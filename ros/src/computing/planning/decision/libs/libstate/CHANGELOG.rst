^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package state
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2017-12-11)
------------------
* Merge branch develop into feature/ndt_pcl_gpu
* Merge branch 'develop' into feature/tlr_turn_support
* Merge pull request `#954 <https://github.com/CPFL/Autoware/issues/954>`_ from CPFL/fix/tf_mapping
  Fix/tf mapping
* add new state for lanechange
* Merge branch 'develop' into feature/tlr_turn
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge branch 'feature/decision' of github.com:CPFL/Autoware into feature/decision
* add new state for all stopline pause
* Merge pull request `#936 <https://github.com/CPFL/Autoware/issues/936>`_ from CPFL/feature/decision
  Feature/decision: Enhancement decision maker node
* Checked coding by cppcheck and apply clang-format
* Add new state
  - TrafficLight State (it will be planning to change "behavior" to
  another category)
  - Crawl(slow speed)
* Add to support multiple lane shift
* Fixed:
  - callback
  - laneshift
  Added:
  - publisher for laneid
  - new lanechange flag
  - new param for decisionMaker
* add to able to have multiple-state
* change way to state management
* cosme
* Fix not working changed callback
* delete build warning, and change stopline
* update state and remove detection state
* merge develop and fixed slow switching map
* Fix a state changing bug
* add support to stopline
* fix segv
* add state changed callback
* minor fixes
* Add feature of to find stopline. and following minor fixes
  - to change vectormap operation to vectormap lib.
  - to change state operation
* Merge branch 'feature/decision' of github.com:cpfl/autoware into feature/decision
* Support to lanechange similar to state_machine(old) package
* :put_litter_in_its_place:
* Add support to switch on/off directional indicator
* remove unnecessary code from decisionmaker
* add support to waypoint velocity control by state
* update decisionmaker and related library
  - add multiplelane path recognition
  - renamed euc
* Merge branch 'master' into feature/decision
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
* apply clang-format
* Add to support dynamical parameter for decision_maker
* Fix a build error for libstate
* Move the decision part of the state machine library to decision_Maker node. This is WIP.
* Fixed configuration of state
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

* apply clang-format
* Add to support dynamical parameter for decision_maker
* Fix a build error for libstate
* Move the decision part of the state machine library to decision_Maker node. This is WIP.
* Fixed configuration of state
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

* apply clang-format
* Add to support dynamical parameter for decision_maker
* Fix a build error for libstate
* Move the decision part of the state machine library to decision_Maker node. This is WIP.
* Fixed configuration of state
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
