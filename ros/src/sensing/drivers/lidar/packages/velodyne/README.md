Autoware's Velodyne Driver Subtree
==================================

[![Build Status](https://travis-ci.org/CPFL/velodyne.svg?branch=Autoware)](https://travis-ci.org/CPFL/velodyne/)

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

Velodyne<sup>1</sup> is a collection of ROS<sup>2</sup> packages supporting `Velodyne high
definition 3D LIDARs`<sup>3</sup>.

**Warning**:

  The master branch normally contains code being tested for the next
  ROS release.  It will not always work with every previous release.
  To check out the source for the most recent release, check out the
  tag `velodyne-<version>` with the highest version number.

The current ``master`` branch works with ROS Indigo and Kinetic.
CI builds are currently run for Lunar and Melodic but extensive
testing has not been completed in those environments.

- <sup>1</sup>ROS: http://www.ros.org
- <sup>2</sup>Velodyne: http://www.ros.org/wiki/velodyne
- <sup>3</sup>`Velodyne high definition 3D LIDARs`: http://www.velodynelidar.com/lidar/lidar.aspx
