# object_lanelet_filter

## Purpose

The `object_lanelet_filter` is a node that filters detected object by using vector map.
The objects only inside of the vector map will be published.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name               | Type                                             | Description            |
| ------------------ | ------------------------------------------------ | ---------------------- |
| `input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin`          | vector map             |
| `input/object`     | `autoware_perception_msgs::msg::DetectedObjects` | input detected objects |

### Output

| Name            | Type                                             | Description               |
| --------------- | ------------------------------------------------ | ------------------------- |
| `output/object` | `autoware_perception_msgs::msg::DetectedObjects` | filtered detected objects |

## Parameters

### Core Parameters

| Name                             | Type | Default Value | Description                               |
| -------------------------------- | ---- | ------------- | ----------------------------------------- |
| `filter_target_label.UNKNOWN`    | bool | false         | If true, unknown objects are filtered.    |
| `filter_target_label.CAR`        | bool | false         | If true, car objects are filtered.        |
| `filter_target_label.TRUCK`      | bool | false         | If true, truck objects are filtered.      |
| `filter_target_label.BUS`        | bool | false         | If true, bus objects are filtered.        |
| `filter_target_label.TRAILER`    | bool | false         | If true, trailer objects are filtered.    |
| `filter_target_label.MOTORCYCLE` | bool | false         | If true, motorcycle objects are filtered. |
| `filter_target_label.BICYCLE`    | bool | false         | If true, bicycle objects are filtered.    |
| `filter_target_label.PEDESTRIAN` | bool | false         | If true, pedestrian objects are filtered. |

## Assumptions / Known limits

The lanelet filter is performed based on the shape polygon and bounding box of the objects.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
