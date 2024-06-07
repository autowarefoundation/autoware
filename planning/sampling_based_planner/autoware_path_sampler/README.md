# Path Sampler

## Purpose

This package implements a node that uses sampling based planning to generate a drivable trajectory.

## Feature

This package is able to:

- make the trajectory smooth;
- keep the trajectory inside the drivable area;
- avoid static obstacles;
- stop if no valid trajectory can be generated.

Note that the velocity is just taken over from the input path.

## Inputs / Outputs

### input

| Name               | Type                                          | Description                                        |
| ------------------ | --------------------------------------------- | -------------------------------------------------- |
| `~/input/path`     | autoware_planning_msgs/msg/Path               | Reference path and the corresponding drivable area |
| `~/input/odometry` | nav_msgs/msg/Odometry                         | Current state of the ego vehicle                   |
| `~/input/objects`  | autoware_perception_msgs/msg/PredictedObjects | objects to avoid                                   |

### output

| Name                  | Type                                  | Description                                                       |
| --------------------- | ------------------------------------- | ----------------------------------------------------------------- |
| `~/output/trajectory` | autoware_planning_msgs/msg/Trajectory | generated trajectory that is feasible to drive and collision-free |

## Algorithm

Sampling based planning is decomposed into 3 successive steps:

1. Sampling: candidate trajectories are generated.
2. Pruning: invalid candidates are discarded.
3. Selection: the best remaining valid candidate is selected.

### Sampling

Candidate trajectories are generated based on the current ego state and some target state.
2 sampling algorithms are currently implemented: sampling with bézier curves or with polynomials in the frenet frame.

### Pruning

The validity of each candidate trajectory is checked using a set of hard constraints.

- collision: ensure no collision with static obstacles;
- curvature: ensure smooth curvature;
- drivable area: ensure the trajectory stays within the drivable area.

### Selection

Among the valid candidate trajectories, the _best_ one is determined using a set of soft constraints (i.e., objective functions).

- curvature: prefer smoother trajectories;
- length: prefer longer trajectories;
- lateral deviation: prefer trajectories close to the reference path.

Each soft constraint is associated with a weight to allow tuning of the preferences.

## Limitations

The quality of the candidates generated with polynomials in frenet frame greatly depend on the reference path.
If the reference path is not smooth, the resulting candidates will probably be undriveable.

Failure to find a valid trajectory current results in a suddenly stopping trajectory.

## Comparison with the `autoware_path_optimizer`

The `autoware_path_optimizer` uses an optimization based approach,
finding the optimal solution of a mathematical problem if it exists.
When no solution can be found, it is often hard to identify the issue due to the intermediate mathematical representation of the problem.

In comparison, the sampling based approach cannot guarantee an optimal solution but is much more straightforward,
making it easier to debug and tune.

## How to Tune Parameters

The sampling based planner mostly offers a trade-off between the consistent quality of the trajectory and the computation time.
To guarantee that a good trajectory is found requires generating many candidates which linearly increases the computation time.

TODO

### Drivability in narrow roads

### Computation time

### Robustness

### Other options

## How To Debug

TODO
