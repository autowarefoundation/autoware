^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package velodyne_laserscan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.2 (2019-01-28)
------------------

1.5.1 (2018-12-10)
------------------

1.5.0 (2018-10-19)
------------------

1.4.0 (2018-09-19)
------------------
* Merge pull request `#170 <https://github.com/ros-drivers/velodyne/issues/170>`_ from ros-drivers/maint/move_header_files
* Moving header files to traditional location inside include folders.
* Merge pull request `#160 <https://github.com/ros-drivers/velodyne/issues/160>`_ from ros-drivers/maint/updating_package_xml_to_v2
* Updated all package.xmls to ver 2. Cleaned up catkin_lint errors.
  All package.xml files are now compatible with version 2 of the
  package.xml specification in REP 140. Removed some unnecessary
  execute permissions on a few files. Fixed a missing test_depend.
* Merge pull request `#146 <https://github.com/ros-drivers/velodyne/issues/146>`_ from stsundermann/patch-2
  Use std::abs instead of fabsf
* Merge pull request `#150 <https://github.com/ros-drivers/velodyne/issues/150>`_ from ros-drivers/mikaelarguedas-patch-1
* update to use non deprecated pluginlib macro
* Use std::abs instead of fabsf
  cfg\_.resolution is double but fabsf takes a float which may cause truncation of value.
* Contributors: Andre Volk, CNR, Joshua Whitley, Mikael Arguedas, Stephan Sundermann

1.3.0 (2017-11-10)
------------------
* Merge pull request `#110 <https://github.com/ros-drivers/velodyne/issues/110>`_ from kmhallen/master
  Added velodyne_laserscan package
* Added tests for velodyne_laserscan
* Fixed validating PointCloud2 field types
* Package.xml format version 2
* Merge pull request `#1 <https://github.com/ros-drivers/velodyne/issues/1>`_ from volkandre/master
  Fixed bug. Laserscans now cover full 360 degrees.
* Fixed bug. Laserscans now cover full 360 degrees.
* Added velodyne_laserscan package and inserted into existing launch files
* Contributors: Joshua Whitley, Kevin Hallenbeck, kmhallen, volkandre
