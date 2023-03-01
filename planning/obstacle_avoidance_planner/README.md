## Purpose

This package generates a trajectory that is kinematically-feasible to drive and collision-free based on the input path, drivable area.
Only position and orientation of trajectory are updated in this module, and velocity is just taken over from the one in the input path.

## Feature

This package is able to

- make the trajectory smooth
- make the trajectory inside the drivable area as much as possible
  - NOTE: Static obstacles to avoid can be removed from the drivable area.
- insert stop point before the planned footprint will be outside the drivable area

Note that the velocity is just taken over from the input path.

## Inputs / Outputs

### input

| Name               | Type                                 | Description                                        |
| ------------------ | ------------------------------------ | -------------------------------------------------- |
| `~/input/path`     | autoware_auto_planning_msgs/msg/Path | Reference path and the corresponding drivable area |
| `~/input/odometry` | nav_msgs/msg/Odometry                | Current Velocity of ego vehicle                    |

### output

| Name                  | Type                                       | Description                                                       |
| --------------------- | ------------------------------------------ | ----------------------------------------------------------------- |
| `~/output/trajectory` | autoware_auto_planning_msgs/msg/Trajectory | Optimized trajectory that is feasible to drive and collision-free |

## Flowchart

Flowchart of functions is explained here.

```plantuml
@startuml
title pathCallback
start

:isDataReady;

:createPlannerData;

group generateOptimizedTrajectory
  group optimizeTrajectory
    :check replan;
    if (replanning required?) then (yes)
      :getEBTrajectory;
      :getModelPredictiveTrajectory;
      if (optimization failed?) then (no)
      else (yes)
        :send previous\n trajectory;
      endif
    else (no)
      :send previous\n trajectory;
    endif
  end group

  :applyInputVelocity;
  :insertZeroVelocityOutsideDrivableArea;
  :publishDebugMarkerOfOptimization;
end group


:extendTrajectory;

:setZeroVelocityAfterStopPoint;

:publishDebugData;

stop
@enduml
```

### createPlannerData

The following data for planning is created.

```cpp
struct PlannerData
{
  // input
  Header header;
  std::vector<TrajectoryPoint> traj_points; // converted from the input path
  std::vector<geometry_msgs::msg::Point> left_bound;
  std::vector<geometry_msgs::msg::Point> right_bound;

  // ego
  geometry_msgs::msg::Pose ego_pose;
  double ego_vel;
};
```

### check replan

When one of the following conditions are met, trajectory optimization will be executed.
Otherwise, previously optimized trajectory is used with updating the velocity from the latest input path.

max_path_shape_around_ego_lat_dist

- Ego moves longer than `replan.max_ego_moving_dist` in one cycle. (default: 3.0 [m])
  - This is for when the ego pose is set again in the simulation.
- Trajectory's end, which is considered as the goal pose, moves longer than `replan.max_goal_moving_dist` in one cycle. (default: 15.0 [ms])
  - When the goal pose is set again, the planning should be reset.
- Time passes. (default: 1.0 [s])
  - The optimization is skipped for a while sine the optimization is sometimes heavy.
- The input path changes laterally longer than `replan.max_path_shape_around_ego_lat_dist` in one cycle. (default: 2.0)

### getEBTrajectory

The latter optimization (model predictive trajectory) assumes that the reference path is smooth enough.
This function makes the input path smooth by elastic band.

More details about elastic band can be seen [here](docs/eb.md).

### getModelPredictiveTrajectory

This module makes the trajectory kinematically-feasible and collision-free.
We define vehicle pose in the frenet coordinate, and minimize tracking errors by optimization.
This optimization considers vehicle kinematics and collision checking with road boundary and obstacles.
To decrease the computation cost, the optimization is applied to the shorter trajectory (default: 50 [m]) than the whole trajectory, and concatenate the remained trajectory with the optimized one at last.

The trajectory just in front of the ego must not be changed a lot so that the steering wheel will be stable.
Therefore, we use the previously generated trajectory in front of the ego.

Optimization center on the vehicle, that tries to locate just on the trajectory, can be tuned along side the vehicle vertical axis.
This parameter `mpt.kinematics.optimization center offset` is defined as the signed length from the back-wheel center to the optimization center.
Some examples are shown in the following figure, and it is shown that the trajectory of vehicle shape differs according to the optimization center even if the reference trajectory (green one) is the same.

![mpt_optimization_offset](./media/mpt_optimization_offset.svg)

More details can be seen [here](docs/mpt.md).

### insertZeroVelocityOutsideDrivableArea

Optimized trajectory is too short for velocity planning, therefore extend the trajectory by concatenating the optimized trajectory and the behavior path considering drivability.
Generated trajectory is checked if it is inside the drivable area or not, and if outside drivable area, output a trajectory inside drivable area with the behavior path or the previously generated trajectory.

As described above, the behavior path is separated into two paths: one is for optimization and the other is the remain. The first path becomes optimized trajectory, and the second path just is transformed to a trajectory. Then a trajectory inside the drivable area is calculated as follows.

- If optimized trajectory is **inside the drivable area**, and the remained trajectory is inside/outside the drivable area,
  - the output trajectory will be just concatenation of those two trajectories.
  - In this case, we do not care if the remained trajectory is inside or outside the drivable area since generally it is outside the drivable area (especially in a narrow road), but we want to pass a trajectory as long as possible to the latter module.
- If optimized trajectory is **outside the drivable area**, and the remained trajectory is inside/outside the drivable area,
  - and if the previously generated trajectory **is memorized**,
    - the output trajectory will be the previously generated trajectory, where zero velocity is inserted to the point firstly going outside the drivable area.
  - and if the previously generated trajectory **is not memorized**,
    - the output trajectory will be a part of trajectory just transformed from the behavior path, where zero velocity is inserted to the point firstly going outside the drivable area.

Optimization failure is dealt with the same as if the optimized trajectory is outside the drivable area.
The output trajectory is memorized as a previously generated trajectory for the next cycle.

_Rationale_
In the current design, since there are some modelling errors, the constraints are considered to be soft constraints.
Therefore, we have to make sure that the optimized trajectory is inside the drivable area or not after optimization.

### alignVelocity

Velocity is assigned in the result trajectory from the velocity in the behavior path.
The shapes of the trajectory and the path are different, therefore the each nearest trajectory point to the path is searched and interpolated linearly.

## Limitation

- Computation cost is sometimes high.
- Because of the approximation such as linearization, some narrow roads cannot be run by the planner.
- Roles of planning for `behavior_path_planner` and `obstacle_avoidance_planner` are not decided clearly. Both can avoid obstacles.

## Comparison to other methods

Trajectory planning problem that satisfies kinematically-feasibility and collision-free has two main characteristics that makes hard to be solved: one is non-convex and the other is high dimension.
Based on the characteristics, we investigate pros/cons of the typical planning methods: optimization-based, sampling-based, and learning-based method.

### Optimization-based method

- pros: comparatively fast against high dimension by leveraging the gradient descent
- cons: often converge to the local minima in the non-convex problem

### Sampling-based method

- pros: realize global optimization
- cons: high computation cost especially in the complex case

### Learning-based method

- under research yet

Based on these pros/cons, we chose the optimization-based planner first.
Although it has a cons to converge to the local minima, it can get a good solution by the preprocessing to approximate the problem to convex that almost equals to the original non-convex problem.

## How to Tune Parameters

### Drivability in narrow roads

- set `option.drivability_check.use_vehicle_circles` true
  - use a set of circles as a shape of the vehicle when checking if the generated trajectory will be outside the drivable area.
- make `mpt.clearance.soft_clearance_from_road` smaller
- make `mpt.kinematics.optimization_center_offset` different

  - The point on the vehicle, offset forward from the base link` tries to follow the reference path.

  - This may cause the a part of generated trajectory will be outside the drivable area.

### Computation time

- Loose EB optimization

  - 1. make `eb.common.delta_arc_length` large and `eb.common.num_points` small
    - This makes the number of design variables smaller
    - Be careful about the trajectory length between MPT and EB as shown in Assumptions.
    - However, empirically this causes large turn at the corner (e.g. The vehicle turns a steering wheel to the opposite side (=left) a bit just before the corner turning to right)
  - 2. make `eb.qp.eps_abs` and `eb.qp.eps_rel` small
    - This causes very unstable reference path generation for MPT, or turning a steering wheel a little bit larger

- Enable computation reduction flag

  - 1. set l_inf_norm true (by default)
    - use L-inf norm optimization for MPT w.r.t. slack variables, resulting in lower number of design variables
  - 2. set enable_warm_start true
  - 3. set enable_manual_warm_start true (by default)
  - 4. set steer_limit_constraint false
    - This causes no assumption for trajectory generation where steering angle will not exceeds its hardware limitation
  - 5. make the number of collision-free constraints small
    - How to change parameters depend on the type of collision-free constraints
      - If
    - This may cause the trajectory generation where a part of ego vehicle is out of drivable area

- Disable publishing debug visualization markers
  - set `option.is_publishing_*` false

### Robustness

- Check if the trajectory before EB, after EB, or after MPT is not robust
  - if the trajectory before EB is not robust
  - if the trajectory after EB is not robust
  - if the trajectory after MPT is not robust
    - make `mpt.weight.steer_input_weight` or `mpt.weight.steer_rate_weight` larger, which are stability of steering wheel along the trajectory.

### Other options

- `option.enable_skip_optimization` skips EB and MPT optimization.
- `option.enable_smoothing` enables EB which is smoothing the trajectory for MPT.
  - EB is not required if the reference path for MPT is smooth enough and does not change its shape suddenly
- `option.enable_calculation_time_info` enables showing each calculation time for functions and total calculation time on the terminal.
- `option.enable_outside_drivable_area_stop` enables stopping just before the generated trajectory point will be outside the drivable area.
- `mpt.option.plan_from_ego` enables planning from the ego pose when the ego's velocity is zero.
- `mpt.option.max_plan_from_ego_length` maximum length threshold to plan from ego. it is enabled when the length of trajectory is shorter than this value.
- `mpt.option.two_step_soft_constraint` enables two step of soft constraints for collision free
  - `mpt.option.soft_clearance_from_road` and `mpt.option.soft_second_clearance_from_road` are the weight.

## How To Debug

How to debug can be seen [here](docs/debug.md).
