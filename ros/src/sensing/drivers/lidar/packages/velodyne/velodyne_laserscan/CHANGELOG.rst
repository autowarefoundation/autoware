^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package velodyne_laserscan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2018-08-31)
------------------
* [Fix] Extend and Update interface.yaml (`#1291 <https://github.com/CPFL/Autoware/pull/1291>`_)
* Contributors: Kenji Funaoka

1.7.0 (2018-05-18)
------------------
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
* Modify package xml version other than 1.6.3
* [feature] vlc32c driver, velodyne drivers updated (`#1166 <https://github.com/CPFL/Autoware/pull/1166>`_)
  * Squashed 'ros/src/sensing/drivers/lidar/packages/velodyne/' changes from 776a358..1a70413
  1a70413 Merge branch 'master' into Autoware
  7976d12 support vlp32c now
  273520e Added hdl32c, fixed naming
  e21b522 Merge pull request `#146 <https://github.com/CPFL/Autoware/pull/146>`_ from stsundermann/patch-2
  0e5a200 Merge pull request `#150 <https://github.com/CPFL/Autoware/pull/150>`_ from ros-drivers/mikaelarguedas-patch-1
  db6b5ee update to use non deprecated pluginlib macro
  560fe12 Use std::abs instead of fabsf
  git-subtree-dir: ros/src/sensing/drivers/lidar/packages/velodyne
  git-subtree-split: 1a704135c529c5d2995cd2c1972ca4f59d5ae1ad
  * Squashed 'ros/src/sensing/drivers/lidar/packages/velodyne/' changes from 1a70413..52c0a0d
  52c0a0d README format
  git-subtree-dir: ros/src/sensing/drivers/lidar/packages/velodyne
  git-subtree-split: 52c0a0d63594ee71a156755954d240d24966829e
  * Squashed 'ros/src/sensing/drivers/lidar/packages/velodyne/' changes from 52c0a0d..a1d6f18
  a1d6f18 Update and rename README.rst to README.md
  git-subtree-dir: ros/src/sensing/drivers/lidar/packages/velodyne
  git-subtree-split: a1d6f186d3340f3ce5059e234ed7e3dcb828d09d
* Contributors: Abraham Monrroy, Kosuke Murakami

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
