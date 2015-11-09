Change history
==============

1.2.0 (2014-08-06)
------------------
* velodyne_pointcloud: remove model-dependent "constants" from
  rawdata.h (`#28
  <https://github.com/ros-drivers/velodyne/issues/28>`_)
* velodyne_pointcloud: change default min_range to 0.9 meters (`#25
  <https://github.com/ros-drivers/velodyne/issues/25>`_)
* Added support for YAML-CPP 0.5+ (`#23
  <https://github.com/ros-drivers/velodyne/pull/23>`_).
* Add dynamic_reconfigure feature.
* Add angular limits to the output point cloud, useful for omitting
  part of it. (`#22 <https://github.com/ros-drivers/velodyne/pull/22>`_).
* Contributors: Jack O'Quin, Scott K Logan, Thomas Solatges

1.1.2 (2013-11-05)
-------------------

 * Move unit test data to download.ros.org (`#18`_).
 * Install missing gen_calibration.py script (`#20`_).

1.1.1 (2013-07-30)
------------------

 * Fix lost frame_id transform problem caused by PCL 1.7 fix (`#13`_).
 * Add support for HDL-64E S2 and S2.1 models, which were not working
   before (`#11`_), thanks to Gabor Meszaros (`#12`_).
 * Add additional parameters to launch files (`#14`_).
 * Contributors: Gabor Meszaros, Jack O'Quin

1.1.0 (2013-07-16)
------------------

 * Fix build problems due to PCL 1.7 API incompatibilities (`#8`_),
   thanks to William Woodall.  This version also works with Groovy, as
   long as the correct ``pcl_conversions`` is installed.
 * Fix errors with Mac OSX compiler (`#8`_).
 * Install ``pluginlib`` XML files (`#9`_).
 * Install some launch and parameter files.
 * Enable unit tests when ``CATKIN_ENABLE_TESTING`` is set (`#10`_).

1.0.1 (2013-06-15)
------------------

 * Declare explicit ``pluginlib`` dependency (`#4`_).

1.0.0 (2013-06-14)
------------------

 * Convert to catkin (`#1`_).
 * Release to Hydro.

0.9.2 (2013-07-08)
------------------

 * Fix Groovy build problem (`#7`_).

0.9.1 (2012-06-05)
------------------

 * Only include "enabled" lasers in YAML calibration file.
 * New param subdirectory for parameter files.
 * Add launch file for the HDL-32E.
 * Add rviz_points.vcg file for viewing Velodyne point clouds with rviz.
 * Fix bug when reading configuration with default minIntensity.
 * Add unit tests with 32E data.
 * Released to Electric, Fuerte and Groovy.

0.9.0 (2012-04-03)
------------------

 * Completely revised API, anticipating a 1.0.0 release.
 * HDL-32E device support.
 * New YAML configuration file format.
 * New velodyne_driver and velodyne_pointcloud packages.
 * Old velodyne_common and velodyne_pcl packages no longer included.
 * Released to Electric, Fuerte and Groovy.

0.2.6 (2011-02-23)
------------------

 * Label all timing-dependent tests "realtime" so they do not run by
   default on the build farm machines.

0.2.5 (2010-11-19)
------------------

 * Initial implementation of new 0.3 interfaces.
 * Support for ROS 1.3 `std_msgs::Header` changes.

0.2.0 (2010-08-17)
------------------

 * Initial release to ROS C-turtle.

.. _`#1`: https://github.com/ros-drivers/velodyne/issues/1
.. _`#4`: https://github.com/ros-drivers/velodyne/issues/4
.. _`#7`: https://github.com/ros-drivers/velodyne/issues/7
.. _`#8`: https://github.com/ros-drivers/velodyne/pull/8
.. _`#9`: https://github.com/ros-drivers/velodyne/issues/9
.. _`#10`: https://github.com/ros-drivers/velodyne/issues/10
.. _`#11`: https://github.com/ros-drivers/velodyne/issues/11
.. _`#12`: https://github.com/ros-drivers/velodyne/pull/12
.. _`#13`: https://github.com/ros-drivers/velodyne/issues/13
.. _`#14`: https://github.com/ros-drivers/velodyne/pull/14
.. _`#17`: https://github.com/ros-drivers/velodyne/issues/17
.. _`#18`: https://github.com/ros-drivers/velodyne/issues/18
.. _`#20`: https://github.com/ros-drivers/velodyne/issues/20
