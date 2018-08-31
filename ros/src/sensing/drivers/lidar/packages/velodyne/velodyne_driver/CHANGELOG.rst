^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package velodyne_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2018-08-31)
------------------
* [Fix] Extend and Update interface.yaml (`#1291 <https://github.com/CPFL/Autoware/pull/1291>`_)
* Contributors: Kenji Funaoka

1.7.0 (2018-05-18)
------------------
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
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
* Removed flawed subtree
* Contributors: AMC, Abraham Monrroy, Kosuke Murakami

1.6.3 (2018-03-06)
------------------

1.6.2 (2018-02-27)
------------------
* Update CHANGELOG
* Contributors: Yusuke FUJII

1.3.0 (2017-11-10)
------------------
* Merge pull request `#129 <https://github.com/ros-drivers/velodyne/issues/129>`_ from kmhallen/pluginlib_macro
  Modern pluginlib macro
* Update to use non deprecated pluginlib macro
* add launch args to support multiple devices (`#108 <https://github.com/ros-drivers/velodyne/issues/108>`_)
* Merge pull request `#101 <https://github.com/ros-drivers/velodyne/issues/101>`_ from teosnare/master
  velodyne_driver/src/lib/input.cc : fix for device_ip filter
* Merge pull request `#104 <https://github.com/ros-drivers/velodyne/issues/104>`_ from altrouge/launch_options
  Add more options in launch files.
* Rearranged alphabetically.
* Add more options in launch files.
  - rpm, device_ip, port, read_once, read_fast, repeat_delay
* velodyne_driver/src/lib/input.cc : fix for device_ip filter
  Fix for device_ip filter in InputSocket: initialization of devip\_ for correct ip filtering in InputSocket::getPacket.
* velodyne_driver: credit @priyankadey for VLP-16 bug fix (`#96 <https://github.com/ros-drivers/velodyne/issues/96>`_)
* Merge pull request `#96 <https://github.com/ros-drivers/velodyne/issues/96>`_ from priyankadey/master
  updated VLP-16 packet rate from user manual.
* updated VLP-16 packet rate from user manual.
  Also verified with sensor. It reduced overlap in the pointcloud
* update change history
* Merge pull request `#94 <https://github.com/ros-drivers/velodyne/issues/94>`_ from ros-drivers/pcap_port
  velodyne_driver: use port number for PCAP data (`#46 <https://github.com/ros-drivers/velodyne/issues/46>`_, `#66 <https://github.com/ros-drivers/velodyne/issues/66>`_)
* fix g++ 5.3.1 compile errors (`#94 <https://github.com/ros-drivers/velodyne/issues/94>`_)
* merge current master (`#94 <https://github.com/ros-drivers/velodyne/issues/94>`_)
* Merge pull request `#91 <https://github.com/ros-drivers/velodyne/issues/91>`_ from chukcha2/master
  update velodyne_driver package description to include all models
* update velodyne_driver package description to include all models
* Merge pull request `#89 <https://github.com/ros-drivers/velodyne/issues/89>`_ from Tones29/feat_dynrec_driver
  Add dynamic latency configuration to velodyne_driver
* velodyne_driver: Add dynamic_reconfigure and time_offset correction
  The value of time_offset is added to the calculated time stamp in live mode for each packet.
* velodyne_driver: Make input destructors virtual
* prepare change history for coming Indigo release (`#59 <https://github.com/ros-drivers/velodyne/issues/59>`_)
* velodyne_driver: use port number for PCAP data (`#66 <https://github.com/ros-drivers/velodyne/issues/66>`_)
* Merge pull request `#39 <https://github.com/ros-drivers/velodyne/issues/39>`_ from zooxco/multivelodyne
  support for multiple velodynes
* Merge pull request `#44 <https://github.com/ros-drivers/velodyne/issues/44>`_ from SISegwayRmp/master
  adding driver and pointcloud support for the VLP16
* adding the VLP16 test scripts and updating the CMakeLists to include the test file from http://download.ros.org/data/velodyne/vlp16.pcap
* adding support for the VLP16
* parameters to set the udp port
* fixed missing header
* cleanup debug line
* parameter and code added for working with multiple velodynes
* Contributors: Andreas Wachaja, Brice Rebsamen, Daniel Jartoux, Denis Dillenberger, Gabor Meszaros, Ilya, Jack O'Quin, Joshua Whitley, Kevin Hallenbeck, Matteo Murtas, Micho Radovnikovich, Priyanka Dey, William Woodall, jack.oquin, junior, phussey
