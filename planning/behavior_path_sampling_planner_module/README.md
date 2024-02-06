# Behavior Path Sampling Based Planner

WARNING: This module is experimental and has not been properly tested on a real vehicle, use only on simulations.

## Purpose

This package implements a node that uses sampling based planning to generate a drivable trajectory for the behavior path planner. It is heavily based off the [sampling_based_planner module](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/sampling_based_planner).

## Features

This package is able to:

- create a smooth trajectory to avoid static obstacles.
- guarantees the generated trajectory (if any) complies with customizable hard constraints.
- transitions to a success state after the ego vehicle merges to its goal lane.
- re-use previously generated outputs to re-sample new alternative paths

Note that the velocity is just taken over from the input path.

## Algorithm

Sampling based planning is decomposed into 3 successive steps:

1. Sampling: candidate trajectories are generated.
2. Pruning: invalid candidates are discarded.
3. Selection: the best remaining valid candidate is selected.

### Sampling

Candidate trajectories are generated based on the current ego state and some target state.
1 sampling algorithms is currently implemented: polynomials in the frenet frame.

### Pruning

The validity of each candidate trajectory is checked using a set of hard constraints.

- collision: ensure no collision with static obstacles;
- curvature: ensure smooth curvature;
- drivable area: ensure the trajectory stays within the drivable area.

### Selection

Among the valid candidate trajectories, the _best_ one is determined using a set of soft constraints (i.e., objective functions).

- curvature: prefer smoother trajectories;
- length: prefer trajectories with longer remaining path length;
- lateral deviation: prefer trajectories close to the goal.

Each soft constraint is associated with a weight to allow tuning of the preferences.

## Limitations

The quality of the candidates generated with polynomials in frenet frame greatly depend on the reference path.
If the reference path is not smooth, the resulting candidates will probably be un-drivable.

Failure to find a valid trajectory current results in a suddenly stopping trajectory.

The module has troubles generating paths that converge rapidly to the goal lanelet. Basically, after overcoming all obstacles, the module should prioritize paths that rapidly make the ego vehicle converge back to its goal lane (ie. paths with low average and final lateral deviation). However, this does not function properly at the moment.

Detection of proper merging can be rough: Sometimes, the module when detects that the ego has converged on the goal lanelet and that there are no more obstacles, the planner transitions to the goal planner, but the transition is not very smooth and could cause discomfort for the user.

## Future works

Some possible improvements for this module include:

-Implementing a dynamic weight tuning algorithm: Dynamically change weights depending on the scenario (ie. to prioritize more the paths with low curvature and low avg. lat. deviation after all obstacles have been avoided).

-Implementing multi-objective optimization to improve computing time and possibly make a more dynamic soft constraints weight tuning. [Related publication](https://ieeexplore.ieee.org/abstract/document/10180226).

-Implement bezier curves as another method to obtain samples, see the [sampling_based_planner module](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/sampling_based_planner).

-Explore the possibility to replace several or other behavior path modules with the sampling based behavior path module.

-Perform real-life tests of this module once it has matured and some of its limitations have been solved.

## Other possibilities

The module is currently aimed at creating paths for static obstacle avoidance. However, the nature of sampling planner allows this module to be expanded or repurposed to other tasks such as lane changes, dynamic avoidance and general reaching of a goal. It is possible, with a good dynamic/scenario dependant weight tuning to use the sampling planning approach as a replacement for the other behavior path modules, assuming good candidate pruning and good soft constraints weight tuning.
