# cluster merger

## Purpose

cluster_merger is a package for merging pointcloud clusters as detected objects with feature type.

## Inner-working / Algorithms

The clusters of merged topics are simply concatenated from clusters of input topics.

## Input / Output

### Input

| Name             | Type                                                     | Description         |
| ---------------- | -------------------------------------------------------- | ------------------- |
| `input/cluster0` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | pointcloud clusters |
| `input/cluster1` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | pointcloud clusters |

### Output

| Name              | Type                                                  | Description     |
| ----------------- | ----------------------------------------------------- | --------------- |
| `output/clusters` | `autoware_auto_perception_msgs::msg::DetectedObjects` | merged clusters |

## Parameters

| Name              | Type   | Description                          | Default value |
| :---------------- | :----- | :----------------------------------- | :------------ |
| `output_frame_id` | string | The header frame_id of output topic. | base_link     |

## Assumptions / Known limits

<!-- Write assumptions and limitations of your implementation.

Example:
  This algorithm assumes obstacles are not moving, so if they rapidly move after the vehicle started to avoid them, it might collide with them.
  Also, this algorithm doesn't care about blind spots. In general, since too close obstacles aren't visible due to the sensing performance limit, please take enough margin to obstacles.
-->

## (Optional) Error detection and handling

<!-- Write how to detect errors and how to recover from them.

Example:
  This package can handle up to 20 obstacles. If more obstacles found, this node will give up and raise diagnostic errors.
-->

## (Optional) Performance characterization

<!-- Write performance information like complexity. If it wouldn't be the bottleneck, not necessary.

Example:
  ### Complexity

  This algorithm is O(N).

  ### Processing time

  ...
-->

## (Optional) References/External links

<!-- Write links you referred to when you implemented.

Example:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (Optional) Future extensions / Unimplemented parts
