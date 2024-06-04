# object_position_filter

## Purpose

The `object_position_filter` is a node that filters detected object based on x,y values.
The objects only inside of the x, y bound will be published.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name           | Type                                             | Description            |
| -------------- | ------------------------------------------------ | ---------------------- |
| `input/object` | `autoware_perception_msgs::msg::DetectedObjects` | input detected objects |

### Output

| Name            | Type                                             | Description               |
| --------------- | ------------------------------------------------ | ------------------------- |
| `output/object` | `autoware_perception_msgs::msg::DetectedObjects` | filtered detected objects |

## Parameters

### Core Parameters

| Name                             | Type  | Default Value | Description                                                     |
| -------------------------------- | ----- | ------------- | --------------------------------------------------------------- |
| `filter_target_label.UNKNOWN`    | bool  | false         | If true, unknown objects are filtered.                          |
| `filter_target_label.CAR`        | bool  | false         | If true, car objects are filtered.                              |
| `filter_target_label.TRUCK`      | bool  | false         | If true, truck objects are filtered.                            |
| `filter_target_label.BUS`        | bool  | false         | If true, bus objects are filtered.                              |
| `filter_target_label.TRAILER`    | bool  | false         | If true, trailer objects are filtered.                          |
| `filter_target_label.MOTORCYCLE` | bool  | false         | If true, motorcycle objects are filtered.                       |
| `filter_target_label.BICYCLE`    | bool  | false         | If true, bicycle objects are filtered.                          |
| `filter_target_label.PEDESTRIAN` | bool  | false         | If true, pedestrian objects are filtered.                       |
| `upper_bound_x`                  | float | 100.00        | Bound for filtering. Only used if filter_by_xy_position is true |
| `lower_bound_x`                  | float | 0.00          | Bound for filtering. Only used if filter_by_xy_position is true |
| `upper_bound_y`                  | float | 50.00         | Bound for filtering. Only used if filter_by_xy_position is true |
| `lower_bound_y`                  | float | -50.00        | Bound for filtering. Only used if filter_by_xy_position is true |

## Assumptions / Known limits

Filtering is performed based on the center position of the object.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
