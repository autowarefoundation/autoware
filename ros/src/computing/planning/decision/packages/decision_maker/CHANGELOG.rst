^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package decision_maker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.3 (2018-03-06)
------------------

1.6.2 (2018-02-27)
------------------
* Update CHANGELOG
* Contributors: Yusuke FUJII

1.6.1 (2018-01-20)
------------------
* update CHANGELOG
* Contributors: Yusuke FUJII

1.6.0 (2017-12-11)
------------------
* Prepare release for 1.6.0
* R.I.P.
* fix a duplicate state update
* stop light_color publish every state update
* add new state for lanechange
* fix a judge intersect stopline
* add new state for all stopline pause
* fix a build bug
* fix a build bug
* fix a state num name
* fix a state num name
* add support to manual fixing light
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
* fix a judge left/right
* Fix a state changing bug
* add support to stopline
* Add feature of to find stopline. and following minor fixes
  - to change vectormap operation to vectormap lib.
  - to change state operation
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
* improve a judge algorithms for right/left-turn in intersection
* Add to support manually decision
* Contributors: Yamato ANDO, Yusuke FUJII

1.5.1 (2017-09-25)
------------------
* Release/1.5.1 (`#816 <https://github.com/cpfl/autoware/issues/816>`_)
  * fix a build error by gcc version
  * fix build error for older indigo version
  * update changelog for v1.5.1
  * 1.5.1
* Contributors: Yusuke FUJII

1.5.0 (2017-09-21)
------------------
* Update changelog
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

1.4.0 (2017-08-04)
------------------

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------

1.2.0 (2017-06-07)
------------------

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
