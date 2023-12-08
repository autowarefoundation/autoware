# multi_object_tracker

## Purpose

The results of the detection are processed by a time series. The main purpose is to give ID and estimate velocity.

## Inner-workings / Algorithms

This multi object tracker consists of data association and EKF.

![multi_object_tracker_overview](image/multi_object_tracker_overview.svg)

### Data association

The data association performs maximum score matching, called min cost max flow problem.
In this package, mussp[1] is used as solver.
In addition, when associating observations to tracers, data association have gates such as the area of the object from the BEV, Mahalanobis distance, and maximum distance, depending on the class label.

### EKF Tracker

Models for pedestrians, bicycles (motorcycles), cars and unknown are available.
The pedestrian or bicycle tracker is running at the same time as the respective EKF model in order to enable the transition between pedestrian and bicycle tracking.
For big vehicles such as trucks and buses, we have separate models for passenger cars and large vehicles because they are difficult to distinguish from passenger cars and are not stable. Therefore, separate models are prepared for passenger cars and big vehicles, and these models are run at the same time as the respective EKF models to ensure stability.

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

| Name      | Type                                                  | Description |
| --------- | ----------------------------------------------------- | ----------- |
| `~/input` | `autoware_auto_perception_msgs::msg::DetectedObjects` | obstacles   |

### Output

| Name       | Type                                                 | Description        |
| ---------- | ---------------------------------------------------- | ------------------ |
| `~/output` | `autoware_auto_perception_msgs::msg::TrackedObjects` | modified obstacles |

## Parameters

<!-- Write parameters of this package.

Example:
  ### Node Parameters

  | Name                   | Type | Description                     |
  | ---------------------- | ---- | ------------------------------- |
  | `output_debug_markers` | bool | whether to output debug markers |
-->

### Core Parameters

Node parameters are defined in [multi_object_tracker.param.yaml](config/multi_object_tracker.param.yaml) and association parameters are defined in [data_association.param.yaml](config/data_association.param.yaml).

#### Node parameters

| Name                        | Type   | Description                                                                                                                 |
| --------------------------- | ------ | --------------------------------------------------------------------------------------------------------------------------- |
| `***_tracker`               | string | EKF tracker name for each class                                                                                             |
| `world_frame_id`            | double | object kinematics definition frame                                                                                          |
| `enable_delay_compensation` | bool   | if True, tracker use timers to schedule publishers and use prediction step to extrapolate object state at desired timestamp |
| `publish_rate`              | double | Timer frequency to output with delay compensation                                                                           |

#### Association parameters

| Name                | Type   | Description                                 |
| ------------------- | ------ | ------------------------------------------- |
| `can_assign_matrix` | double | Assignment table for data association       |
| `max_dist_matrix`   | double | Maximum distance table for data association |
| `max_area_matrix`   | double | Maximum area table for data association     |
| `min_area_matrix`   | double | Minimum area table for data association     |
| `max_rad_matrix`    | double | Maximum angle table for data association    |

## Assumptions / Known limits

<!-- Write assumptions and limitations of your implementation.

Example:
  This algorithm assumes obstacles are not moving, so if they rapidly move after the vehicle started to avoid them, it might collide with them.
  Also, this algorithm doesn't care about blind spots. In general, since too close obstacles aren't visible due to the sensing performance limit, please take enough margin to obstacles.
-->

See the [model explanations](models.md).

## (Optional) Error detection and handling

<!-- Write how to detect errors and how to recover from them.

Example:
  This package can handle up to 20 obstacles. If more obstacles found, this node will give up and raise diagnostic errors.
-->

## (Optional) Performance characterization

### Evaluation of muSSP

According to our evaluation, muSSP is faster than normal [SSP](src/data_association/successive_shortest_path) when the matrix size is more than 100.

Execution time for varying matrix size at 95% sparsity. In real data, the sparsity was often around 95%.
![mussp_evaluation1](image/mussp_evaluation1.png)

Execution time for varying the sparsity with matrix size 100.
![mussp_evaluation2](image/mussp_evaluation2.png)

## (Optional) References/External links

This package makes use of external code.

| Name                                                      | License                                                   | Original Repository                  |
| --------------------------------------------------------- | --------------------------------------------------------- | ------------------------------------ |
| [muSSP](src/data_association/mu_successive_shortest_path) | [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0) | <https://github.com/yu-lab-vt/muSSP> |

[1] C. Wang, Y. Wang, Y. Wang, C.-t. Wu, and G. Yu, “muSSP: Efficient
Min-cost Flow Algorithm for Multi-object Tracking,” NeurIPS, 2019

## (Optional) Future extensions / Unimplemented parts

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
