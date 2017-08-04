^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kitti_player
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2017-08-04)
------------------

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------

1.2.0 (2017-06-07)
------------------
* Fix compile error regarding kitty_player
  * Change compile flag from `-std=c++0x` to `-std=c++11` as `c++0x` seems to be deprecated later than GCC ver4.7
  * Fix `ROS_ERROR_STREAM` usages as `<<` operator in `ROS_ERROR_STREAM` doesn't seem to be able to use with ifstream directly
* hotfix make issue
* KITTI Dataset ROS Publisher, terminal mode only
* Contributors: AMC, Manato Hirabayashi, Yusuke Fujii

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
