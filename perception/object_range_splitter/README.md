# object_range_splitter

## Purpose

object_range_splitter is a package to divide detected objects into two messages by the distance from the origin.

## Inner-workings / Algorithms

<!-- Write how this package works. Flowcharts and figures are great. Add sub-sections as you like.

Example:
  ### Flowcharts

  ...(PlantUML or something)

  ### State Transitions

  ...(PlantUML or something)

  ### How to filter target obstacles

  ...

  ### How to optimize trajectory

  ...
-->

## Inputs / Outputs

### Input

| Name           | Type                                                  | Description      |
| -------------- | ----------------------------------------------------- | ---------------- |
| `input/object` | `autoware_auto_perception_msgs::msg::DetectedObjects` | detected objects |

### Output

| Name                        | Type                                                  | Description                  |
| --------------------------- | ----------------------------------------------------- | ---------------------------- |
| `output/long_range_object`  | `autoware_auto_perception_msgs::msg::DetectedObjects` | long range detected objects  |
| `output/short_range_object` | `autoware_auto_perception_msgs::msg::DetectedObjects` | short range detected objects |

## Parameters

| Name          | Type  | Description                                          |
| ------------- | ----- | ---------------------------------------------------- |
| `split_range` | float | the distance boundary to divide detected objects [m] |

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

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
