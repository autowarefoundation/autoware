# dummy_perception_publisher

## Purpose

This node publishes the result of the dummy detection with the type of perception.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name           | Type                                      | Description             |
| -------------- | ----------------------------------------- | ----------------------- |
| `/tf`          | `tf2_msgs/TFMessage`                      | TF (self-pose)          |
| `input/object` | `dummy_perception_publisher::msg::Object` | dummy detection objects |

### Output

| Name                    | Type                                                     | Description            |
| ----------------------- | -------------------------------------------------------- | ---------------------- |
| `output/dynamic_object` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | Publishes objects      |
| `output/points_raw`     | `sensor_msgs::msg::PointCloud2`                          | point cloud of objects |

## Parameters

| Name                        | Type   | Default Value | Explanation                                 |
| --------------------------- | ------ | ------------- | ------------------------------------------- |
| `visible_range`             | double | 100.0         | sensor visible range [m]                    |
| `detection_successful_rate` | double | 0.8           | sensor detection rate. (min) 0.0 - 1.0(max) |
| `enable_ray_tracing`        | bool   | true          | if True, use ray tracking                   |
| `use_object_recognition`    | bool   | true          | if True, publish objects topic              |

### Node Parameters

None.

### Core Parameters

None.

## Assumptions / Known limits

TBD.
