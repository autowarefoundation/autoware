^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package amathutils
^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2018-01-20)
------------------

1.6.0 (2017-12-11)
------------------
* Merge branch 'develop' into feature/tlr_turn
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge pull request `#936 <https://github.com/CPFL/Autoware/issues/936>`_ from CPFL/feature/decision
  Feature/decision: Enhancement decision maker node
* Checked coding by cppcheck and apply clang-format
* Add new state
  - TrafficLight State (it will be planning to change "behavior" to
  another category)
  - Crawl(slow speed)
* add support to stopline
* Add feature of to find stopline. and following minor fixes
  - to change vectormap operation to vectormap lib.
  - to change state operation
* add support to waypoint velocity control by state
* add mps2kmph
* update decisionmaker and related library
  - add multiplelane path recognition
  - renamed euc
* Contributors: AMC, Yamato ANDO, Yuki Kitsukawa, Yusuke FUJII

1.5.1 (2017-09-25)
------------------
* fix a build error by gcc version
* Contributors: Yusuke FUJII

1.5.0 (2017-09-21)
------------------
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
* organize package files and directories
* Add a decision_maker package
  The decision_maker package determines the intention of what actions the
  local planner and control node should take based on perception nodes,
  global planner nodes, map data, sensor data.
  This commit corresponds only to the following functions.
  - Behavior state recognition
  - Dynamic selection of local planner (It is necessary to change the topic name of local planner)
* Contributors: Yusuke FUJII
