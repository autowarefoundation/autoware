# image_projection_based_fusion

## Purpose

The `image_projection_based_fusion` is a package to fuse detected obstacles (bounding box or segmentation) from image and 3d pointcloud or obstacles (bounding box, cluster or segmentation).

## Inner-workings / Algorithms

Detail description of each fusion's algorithm is in the following links.

| Fusion Name                | Description                                                                                     | Detail                                       |
| -------------------------- | ----------------------------------------------------------------------------------------------- | -------------------------------------------- |
| roi_cluster_fusion         | Overwrite a classification label of clusters by that of ROIs from a 2D object detector.         | [link](./docs/roi-cluster-fusion.md)         |
| roi_detected_object_fusion | Overwrite a classification label of detected objects by that of ROIs from a 2D object detector. | [link](./docs/roi-detected-object-fusion.md) |
