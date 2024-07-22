# autoware_detected_object_feature_remover

## Purpose

The `autoware_detected_object_feature_remover` is a package to convert topic-type from `DetectedObjectWithFeatureArray` to `DetectedObjects`.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name      | Type                                                         | Description                         |
| --------- | ------------------------------------------------------------ | ----------------------------------- |
| `~/input` | `tier4_perception_msgs::msg::DetectedObjectWithFeatureArray` | detected objects with feature field |

### Output

| Name       | Type                                             | Description      |
| ---------- | ------------------------------------------------ | ---------------- |
| `~/output` | `autoware_perception_msgs::msg::DetectedObjects` | detected objects |

## Parameters

None

## Assumptions / Known limits
