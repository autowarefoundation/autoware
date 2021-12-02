# ndt_pcl_modified

## Purpose

This is a modification of [PCL](https://github.com/PointCloudLibrary/pcl)'s NDT.

## Modifications

- You can get the Hessian matrix by getHessian().
- You can get the estimated position for each iteration by getFinalTransformationArray().
- It optimizes rotational axes first, then jointly optimizes rotational and translational axes. [experimental feature]
