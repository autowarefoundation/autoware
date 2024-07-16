# low_intensity_cluster_filter

## Purpose

The `low_intensity_cluster_filter` is a node that filters clusters based on the intensity of their pointcloud.

Mainly this focuses on filtering out unknown objects with very low intensity pointcloud, such as fail detection of unknown objects caused by raindrop or water splash from ego or other fast moving vehicles.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name           | Type                                                     | Description            |
| -------------- | -------------------------------------------------------- | ---------------------- |
| `input/object` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | input detected objects |

### Output

| Name            | Type                                                     | Description               |
| --------------- | -------------------------------------------------------- | ------------------------- |
| `output/object` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | filtered detected objects |

## Parameters

### Core Parameters

| Name                              | Type  | Default Value | Description                                             |
| --------------------------------- | ----- | ------------- | ------------------------------------------------------- |
| `filter_target_label.UNKNOWN`     | bool  | false         | If true, unknown objects are filtered.                  |
| `filter_target_label.CAR`         | bool  | false         | If true, car objects are filtered.                      |
| `filter_target_label.TRUCK`       | bool  | false         | If true, truck objects are filtered.                    |
| `filter_target_label.BUS`         | bool  | false         | If true, bus objects are filtered.                      |
| `filter_target_label.TRAILER`     | bool  | false         | If true, trailer objects are filtered.                  |
| `filter_target_label.MOTORCYCLE`  | bool  | false         | If true, motorcycle objects are filtered.               |
| `filter_target_label.BICYCLE`     | bool  | false         | If true, bicycle objects are filtered.                  |
| `filter_target_label.PEDESTRIAN`  | bool  | false         | If true, pedestrian objects are filtered.               |
| `max_x`                           | float | 60.00         | Maximum of x of the filter effective range              |
| `min_x`                           | float | -20.00        | Minimum of x of the filter effective range              |
| `max_y`                           | float | 20.00         | Maximum of y of the filter effective range              |
| `min_y`                           | float | -20.00        | Minium of y of the filter effective range               |
| `intensity_threshold`             | float | 1.0           | The threshold of average intensity for filter           |
| `existence_probability_threshold` | float | 0.2           | The existence probability threshold to apply the filter |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
