# object_merger

## Purpose

object_merger is a package for merging detected objects from two methods by data association.

## Inner-workings / Algorithms

The successive shortest path algorithm is used to solve the data association problem (the minimum-cost flow problem). The cost is calculated by the distance between two objects and gate functions are applied to reset cost, s.t. the maximum distance, the maximum area and the minimum area.

## Inputs / Outputs

### Input

| Name            | Type                                                  | Description       |
| --------------- | ----------------------------------------------------- | ----------------- |
| `input/object0` | `autoware_auto_perception_msgs::msg::DetectedObjects` | detection objects |
| `input/object1` | `autoware_auto_perception_msgs::msg::DetectedObjects` | detection objects |

### Output

| Name            | Type                                                  | Description      |
| --------------- | ----------------------------------------------------- | ---------------- |
| `output/object` | `autoware_auto_perception_msgs::msg::DetectedObjects` | modified Objects |

## Parameters

No Parameters.

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

Data association algorithm was the same as that of multi_object_tracker, but the algorithm of multi_object_tracker was already updated.
