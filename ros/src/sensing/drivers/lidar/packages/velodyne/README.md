Autoware's Velodyne Driver Subtree
==================================

This directory is part of a subtree fetched from https://github.com/CPFL/velodyne on the **Autoware** branch, a fork from the original https://github.com/ros-drivers/velodyne

This repo adds support to HDL-64 S3 and creates the launch files used by Autoware.
If you need to modify **any** file inside this folder structure, please use the following commands to either push or fetch changes from the subtree.
All the commands written here will suppose you're in the root of Autoware path.

## Pulling in commits from the repository subtree 

Bring latest commits from https://github.com/CPFL/velodyne

`git subtree pull --prefix ros/src/sensing/drivers/lidar/packages/velodyne https://github.com/CPFL/velodyne Autoware --squash`

## Pushing changes to the repository subtree 

If you made any modification to the subtree you are encouraged to commit and publish your changes to the fork. You can do with the following command.

`git subtree push --prefix ros/src/sensing/drivers/lidar/packages/velodyne https://github.com/CPFL/velodyne Autoware`

**End of Section**

**Original README from https://github.com/ros-drivers/velodyne**

---

Overview
========

Velodyne_ is a collection of ROS_ packages supporting `Velodyne high
definition 3D LIDARs`_ .

**Warning**::

  The master branch normally contains code being tested for the next
  ROS release.  It will not always work with every previous release.

The current ``master`` branch works with ROS Kinetic, Jade, and
Indigo.  It may work with Hydro and Groovy, but that has not been
tested recently.  To build for Fuerte from source, check out the
``rosbuild`` branch instead of ``master``.

.. _ROS: http://www.ros.org
.. _Velodyne: http://www.ros.org/wiki/velodyne
.. _`Velodyne high definition 3D LIDARs`: http://www.velodynelidar.com/lidar/lidar.aspx
