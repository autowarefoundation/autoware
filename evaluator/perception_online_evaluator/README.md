# Perception Evaluator

A node for evaluating the output of perception systems.

## Purpose

This module allows for the evaluation of how accurately perception results are generated without the need for annotations. It is capable of confirming performance and can evaluate results from a few seconds prior, enabling online execution.

## Inner-workings / Algorithms

- Calculates lateral deviation between the predicted path and the actual traveled trajectory.
- Calculates lateral deviation between the smoothed traveled trajectory and the perceived position to evaluate the stability of lateral position recognition.
- Calculates yaw deviation between the smoothed traveled trajectory and the perceived position to evaluate the stability of yaw recognition.
- Calculates yaw rate based on the yaw of the object received in the previous cycle to evaluate the stability of the yaw rate recognition.

## Inputs / Outputs

| Name              | Type                                                   | Description                                       |
| ----------------- | ------------------------------------------------------ | ------------------------------------------------- |
| `~/input/objects` | `autoware_auto_perception_msgs::msg::PredictedObjects` | The predicted objects to evaluate.                |
| `~/metrics`       | `diagnostic_msgs::msg::DiagnosticArray`                | Diagnostic information about perception accuracy. |
| `~/markers`       | `visualization_msgs::msg::MarkerArray`                 | Visual markers for debugging and visualization.   |

## Parameters

| Name                              | Type         | Description                                                                                      |
| --------------------------------- | ------------ | ------------------------------------------------------------------------------------------------ |
| `selected_metrics`                | List         | Metrics to be evaluated, such as lateral deviation, yaw deviation, and predicted path deviation. |
| `smoothing_window_size`           | Integer      | Determines the window size for smoothing path, should be an odd number.                          |
| `prediction_time_horizons`        | list[double] | Time horizons for prediction evaluation in seconds.                                              |
| `stopped_velocity_threshold`      | double       | threshold velocity to check if vehicle is stopped                                                |
| `target_object.*.check_deviation` | bool         | Whether to check deviation for specific object types (car, truck, etc.).                         |
| `debug_marker.*`                  | bool         | Debugging parameters for marker visualization (history path, predicted path, etc.).              |

## Assumptions / Known limits

It is assumed that the current positions of PredictedObjects are reasonably accurate.

## Future extensions / Unimplemented parts

- Increase rate in recognition per class
- Metrics for objects with strange physical behavior (e.g., going through a fence)
- Metrics for splitting objects
- Metrics for problems with objects that are normally stationary but move
- Disappearing object metrics
