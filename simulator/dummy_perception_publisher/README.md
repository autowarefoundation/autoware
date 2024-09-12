# dummy_perception_publisher

## Purpose

This node publishes the result of the dummy detection with the type of perception.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name           | Type                                      | Description             |
| -------------- | ----------------------------------------- | ----------------------- |
| `/tf`          | `tf2_msgs/TFMessage`                      | TF (self-pose)          |
| `input/object` | `tier4_simulation_msgs::msg::DummyObject` | dummy detection objects |

### Output

| Name                                | Type                                                     | Description             |
| ----------------------------------- | -------------------------------------------------------- | ----------------------- |
| `output/dynamic_object`             | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | dummy detection objects |
| `output/points_raw`                 | `sensor_msgs::msg::PointCloud2`                          | point cloud of objects  |
| `output/debug/ground_truth_objects` | `autoware_perception_msgs::msg::TrackedObjects`          | ground truth objects    |

## Parameters

| Name                        | Type   | Default Value | Explanation                                      |
| --------------------------- | ------ | ------------- | ------------------------------------------------ |
| `visible_range`             | double | 100.0         | sensor visible range [m]                         |
| `detection_successful_rate` | double | 0.8           | sensor detection rate. (min) 0.0 - 1.0(max)      |
| `enable_ray_tracing`        | bool   | true          | if True, use ray tracking                        |
| `use_object_recognition`    | bool   | true          | if True, publish objects topic                   |
| `use_base_link_z`           | bool   | true          | if True, node uses z coordinate of ego base_link |
| `publish_ground_truth`      | bool   | false         | if True, publish ground truth objects            |
| `use_fixed_random_seed`     | bool   | false         | if True, use fixed random seed                   |
| `random_seed`               | int    | 0             | random seed                                      |

### Node Parameters

None.

### Core Parameters

None.

## Assumptions / Known limits

TBD.
