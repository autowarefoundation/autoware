# Planning Evaluator

## Purpose

This package provides nodes that generate metrics to evaluate the quality of planning and control.

## Inner-workings / Algorithms

The evaluation node calculates metrics each time it receives a trajectory `T(0)`.
Metrics are calculated using the following information:

- the trajectory `T(0)` itself.
- the previous trajectory `T(-1)`.
- the _reference_ trajectory assumed to be used as the reference to plan `T(0)`.
- the current ego pose.
- the set of objects in the environment.

These information are maintained by an instance of class `MetricsCalculator`
which is also responsible for calculating metrics.

### Stat

Each metric is calculated using a `Stat` instance which contains
the minimum, maximum, and mean values calculated for the metric
as well as the number of values measured.

### Metric calculation and adding more metrics

All possible metrics are defined in the `Metric` enumeration defined
`include/planning_evaluator/metrics/metric.hpp`.
This file also defines conversions from/to string as well as human readable descriptions
to be used as header of the output file.

The `MetricsCalculator` is responsible for calculating metric statistics
through calls to function:

```C++
Stat<double> MetricsCalculator::calculate(const Metric metric, const Trajectory & traj) const;
```

Adding a new metric `M` requires the following steps:

- `metrics/metric.hpp`: add `M` to the `enum`, to the from/to string conversion maps, and to the description map.
- `metrics_calculator.cpp`: add `M` to the `switch/case` statement of the `calculate` function.
- Add `M` to the `selected_metrics` parameters.

## Inputs / Outputs

### Inputs

| Name                           | Type                                              | Description                                       |
| ------------------------------ | ------------------------------------------------- | ------------------------------------------------- |
| `~/input/trajectory`           | `autoware_planning_msgs::msg::Trajectory`         | Main trajectory to evaluate                       |
| `~/input/reference_trajectory` | `autoware_planning_msgs::msg::Trajectory`         | Reference trajectory to use for deviation metrics |
| `~/input/objects`              | `autoware_perception_msgs::msg::PredictedObjects` | Obstacles                                         |

### Outputs

Each metric is published on a topic named after the metric name.

| Name        | Type                                    | Description                                             |
| ----------- | --------------------------------------- | ------------------------------------------------------- |
| `~/metrics` | `diagnostic_msgs::msg::DiagnosticArray` | DiagnosticArray with a DiagnosticStatus for each metric |

When shut down, the evaluation node writes the values of the metrics measured during its lifetime
to a file as specified by the `output_file` parameter.

## Parameters

{{ json_to_markdown("evaluator/autoware_planning_evaluator/schema/autoware_planning_evaluator.schema.json") }}

## Assumptions / Known limits

There is a strong assumption that when receiving a trajectory `T(0)`,
it has been generated using the last received reference trajectory and objects.
This can be wrong if a new reference trajectory or objects are published while `T(0)` is being calculated.

Precision is currently limited by the resolution of the trajectories.
It is possible to interpolate the trajectory and reference trajectory to increase precision but would make computation significantly more expensive.

## Future extensions / Unimplemented parts

- Use `Route` or `Path` messages as reference trajectory.
- RSS metrics (done in another node <https://tier4.atlassian.net/browse/AJD-263>).
- Add option to publish the `min` and `max` metric values. For now only the `mean` value is published.
- `motion_evaluator_node`.
  - Node which constructs a trajectory over time from the real motion of ego.
  - Only a proof of concept is currently implemented.
