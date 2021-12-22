# outlier_filter

## Purpose

The `outlier_filter` is a package for filtering outlier of points.

## Inner-workings / Algorithms

| Filter Name                                    | Description                                                                                                                             | Detail                                       |
| ---------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------- |
| radius search 2d outlier filter                | A method of removing point cloud noise based on the number of points existing within a certain radius                                   | [link](./radius-search-2d-outlier-filter.md) |
| ring outlier filter                            | A method of operating scan in chronological order and removing noise based on the rate of change in the distance between points         | [link](./ring-outlier-filter.md)             |
| voxel grid outlier filter                      | A method of removing point cloud noise based on the number of points existing within a voxel                                            | [link](./voxel-grid-outlier-filter.md)       |
| dual return outlier filter (under development) | A method of removing rain and fog by considering the light reflected from the object in two stages according to the attenuation factor. | [link](./dual-return-outlier-filter.md)      |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
