# detected_object_validation (occupancy grid based validator)

## Purpose

The purpose of this package is to eliminate obvious false positives of DetectedObjects.

## Inner-workings / Algorithms

Compare the occupancy grid map with the DetectedObject, and if a larger percentage of obstacles are in freespace, delete them.

![debug sample image](image/debug_image.png)

Basically, it takes an occupancy grid map as input and generates a binary image of freespace or other.

A mask image is generated for each DetectedObject and the average value (percentage) in the mask image is calculated.
If the percentage is low, it is deleted.

## Inputs / Outputs

### Input

| Name                         | Type                                                  | Description                                                 |
| ---------------------------- | ----------------------------------------------------- | ----------------------------------------------------------- |
| `~/input/detected_objects`   | `autoware_auto_perception_msgs::msg::DetectedObjects` | DetectedObjects                                             |
| `~/input/occupancy_grid_map` | `nav_msgs::msg::OccupancyGrid`                        | OccupancyGrid with no time series calculation is preferred. |

### Output

| Name               | Type                                                  | Description               |
| ------------------ | ----------------------------------------------------- | ------------------------- |
| `~/output/objects` | `autoware_auto_perception_msgs::msg::DetectedObjects` | validated DetectedObjects |

## Parameters

| Name             | Type  | Description                                        |
| ---------------- | ----- | -------------------------------------------------- |
| `mean_threshold` | float | The percentage threshold of allowed non-freespace. |
| `enable_debug`   | bool  | Whether to display debug images or not?            |

## Assumptions / Known limits

Currently, only vehicle represented as BoundingBox are supported.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
