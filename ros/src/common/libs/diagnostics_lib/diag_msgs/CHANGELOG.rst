^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package state
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.0 (2019-01-17)
-------------------

1.9.1 (2018-11-06)
------------------

1.9.0 (2018-10-31)
------------------
* add disable mode for diag_manager
* add diag_lib
  add csv writer
  remove unused comment
  fix maintainer names
  fix email address
  fix dependency
  fix buildtool_depend, remove unused lines from find_package
* Contributors: Masaya Kataoka

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
* add new state for lanechange
* add new state for all stopline pause
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
* Fix a state changing bug
* add support to stopline
* fix segv
* add state changed callback
* minor fixes
* Add feature of to find stopline. and following minor fixes
  - to change vectormap operation to vectormap lib.
  - to change state operation
* Support to lanechange similar to state_machine(old) package
* :put_litter_in_its_place:
* Add support to switch on/off directional indicator
* remove unnecessary code from decisionmaker
* add support to waypoint velocity control by state
* update decisionmaker and related library
  - add multiplelane path recognition
  - renamed euc
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
