# Behavior Path Planner

## Purpose / Use cases

The `behavior_path_planner` module is responsible to generate

1. **path** based on the traffic situation,
2. **drivable area** that the vehicle can move (defined in the path msg),
3. **turn signal** command to be sent to the vehicle interface.

## Design

### Scene modules design

#### Support modules

Behavior path planner has following scene modules.

| Name                 | Description                                                                                                                                                                | Details                                                                 |
| :------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------- |
| Lane Following       | this module generates reference path from lanelet centerline.                                                                                                              | LINK                                                                    |
| Avoidance            | this module generates avoidance path when there is objects that should be avoid.                                                                                           | [LINK](./docs/behavior_path_planner_avoidance_design.md)                |
| Avoidance By LC      | this module generates lane change path when there is objects that should be avoid.                                                                                         | [LINK](./docs/behavior_path_planner_avoidance_by_lane_change_design.md) |
| Lane Change          | this module is performed when it is necessary and a collision check with other vehicles is cleared.                                                                        | [LINK](./docs/behavior_path_planner_lane_change_design.md)              |
| External Lane Change | WIP                                                                                                                                                                        | LINK                                                                    |
| Goal Planner         | this module is performed when ego-vehicle is in the road lane and goal is in the shoulder lane. ego-vehicle will stop at the goal.                                         | [LINK](./docs/behavior_path_planner_goal_planner_design.md)             |
| Pull Out             | this module is performed when ego-vehicle is stationary and footprint of ego-vehicle is included in shoulder lane. This module ends when ego-vehicle merges into the road. | [LINK](./docs/behavior_path_planner_start_planner_design.md)            |
| Side Shift           | (for remote control) shift the path to left or right according to an external instruction.                                                                                 | [LINK](./docs/behavior_path_planner_side_shift_design.md)               |

![behavior_modules](./image/behavior_modules.png)

All scene modules are implemented by inheriting base class `scene_module_interface.hpp`.

#### How to implement new module?

WIP

---

### Manager design

The role of manager is to launch the appropriate scene module according to the situation. (e.g. if there is parked-vehicle in ego's driving lane, the manager launches the avoidance module.)

Now, it is able to select two managers with different architecture.

| Name                                  | Description                                                                                                                                                                | Details                                                |
| :------------------------------------ | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :----------------------------------------------------- |
| Behavior Tree based manager (default) | this manager launches scene modules based on Behavior Tree. all scene modules run exclusively.                                                                             | LINK(WIP)                                              |
| BT-free manager (unstable)            | this manager is developed in order to achieve complex scenario, and launches scene modules without Behavior Tree. multiple modules can run simultaneously on this manager. | [LINK](./docs/behavior_path_planner_manager_design.md) |

The manager is switched by flag `COMPILE_WITH_OLD_ARCHITECTURE` in CmakeLists.txt of `behavior_path_planner` package. Please set the flag **FALSE** if you try to use BT-free manager.

```cmake
cmake_minimum_required(VERSION 3.14)
project(behavior_path_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(magic_enum CONFIG REQUIRED)

set(COMPILE_WITH_OLD_ARCHITECTURE TRUE) # <- HERE

...
```

**What is the Behavior Tree?**: In the behavior path planner, the behavior tree mechanism is used to manage which modules are activated in which situations. In general, this "behavior manager" like function is expected to become bigger as more and more modules are added in the future. To improve maintainability, we adopted the behavior tree. The behavior tree has the following advantages: easy visualization, easy configuration management (behaviors can be changed by replacing configuration files), and high scalability compared to the state machine.

The current behavior tree structure is shown below. Each modules (LaneChange, Avoidance, etc) have _Request_, _Ready_, and _Plan_ nodes as a common function.

- **Request**: Check if there is a request from the module (e.g. LaneChange has a request when there are multi-lanes and the vehicle is not on the preferred lane),
- **Ready**: Check if it is safe to execute the plan (e.g. LaneChange is ready when the lane_change path does not have any conflicts with other dynamic objects on S-T space).
- **Plan**: Calculates path and set it to the output of the BehaviorTree. Until the internal status returns SUCCESS, it will be in running state and will not transit to another module.
- **ForceApproval**: A lane change-specific node that overrides the result of _Ready_ when a forced lane change command is given externally.

![behavior_path_planner_bt_config](./image/behavior_path_planner_bt_config.png)

## Inputs / Outputs / API

### output

| Name                        | Type                                                     | Description                                                                                    |
| :-------------------------- | :------------------------------------------------------- | :--------------------------------------------------------------------------------------------- |
| ~/input/path                | `autoware_auto_planning_msgs::msg::PathWithLaneId`       | the path generated by modules.                                                                 |
| ~/input/path_candidate      | `autoware_auto_planning_msgs::msg::Path`                 | the path the module is about to take. to be executed as soon as external approval is obtained. |
| ~/input/turn_indicators_cmd | `autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand` | turn indicators command.                                                                       |
| ~/input/hazard_lights_cmd   | `tier4_planning_msgs::msg::PathChangeModuleArray`        | hazard lights command.                                                                         |
| ~/input/ready_module        | `tier4_planning_msgs::msg::PathChangeModule`             | (for remote control) modules that are ready to be executed.                                    |
| ~/input/running_modules     | `tier4_planning_msgs::msg::PathChangeModuleArray`        | (for remote control) current running module.                                                   |

### input

| Name                           | Type                                                   | Description                                                                           |
| :----------------------------- | :----------------------------------------------------- | :------------------------------------------------------------------------------------ |
| ~/input/route                  | `autoware_auto_mapping_msgs::msg::LaneletRoute`        | current route from start to goal.                                                     |
| ~/input/vector_map             | `autoware_auto_mapping_msgs::msg::HADMapBin`           | map information.                                                                      |
| ~/input/objects                | `autoware_auto_perception_msgs::msg::PredictedObjects` | dynamic objects from perception module.                                               |
| ~/input/occupancy_grid_map/map | `nav_msgs::msg::OccupancyGrid`                         | occupancy grid map from perception module. This is used for only Goal Planner module. |
| ~/input/kinematic_state        | `nav_msgs::msg::Odometry`                              | for ego velocity.                                                                     |

## General features of behavior path planner

### Drivable area generation logic

The behavior path planner generates drivable area that is defined in `autoware_auto_planning_msgs::msg::PathWithLaneId` and `autoware_auto_planning_msgs::msg::Path` messages as:

```c++
std::vector<geometry_msgs::msg::Point> left_bound;
std::vector<geometry_msgs::msg::Point> right_bound;
```

Optionally, the drivable area can be expanded by a static distance.
Expansion parameters are defined for each module of the `behavior_path_planner` and should be prefixed accordingly (see `config/drivable_area_expansion.yaml` for an example).

| Name                             | Unit | Type            | Description                                | Default value |
| :------------------------------- | :--- | :-------------- | :----------------------------------------- | :------------ |
| drivable_area_right_bound_offset | [m]  | double          | expansion distance of the right bound      | 0.0           |
| drivable_area_right_bound_offset | [m]  | double          | expansion distance of the left bound       | 0.0           |
| drivable_area_types_to_skip      |      | list of strings | types of linestrings that are not expanded | [road_border] |

Click [here](./docs/behavior_path_planner_drivable_area_design.md) for details.

### Turn signal decision logic

The behavior path planner outputs turn signal information as `autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand`. It uses rule-based algorithm to determine blinkers.

```c++
module autoware_auto_vehicle_msgs {
  module msg {
    module TurnIndicatorsCommand_Constants {
      const uint8 NO_COMMAND = 0;
      const uint8 DISABLE = 1;
      const uint8 ENABLE_LEFT = 2;
      const uint8 ENABLE_RIGHT = 3;
    };

    @verbatim (language="comment", text=
    " Command for controlling turn indicators.")

    struct TurnIndicatorsCommand {
      builtin_interfaces::msg::Time stamp;

      @default (value=0)
      uint8 command;
    };
  };
};
```

Click [here](./docs/behavior_path_planner_turn_signal_design.md) for details.

### Shifted path generation logic

Some modules have to generate shifted path from reference path, then shift path generation logic is implemented as library **path shifter**. **path shifter** takes a reference path and shift lines and output a shifted path.

```c++
struct ShiftLine
{
  Pose start{};  // shift start point in absolute coordinate
  Pose end{};    // shift start point in absolute coordinate

  // relative shift length at the start point related to the reference path
  double start_shift_length{};

  // relative shift length at the end point related to the reference path
  double end_shift_length{};

  size_t start_idx{};  // associated start-point index for the reference path
  size_t end_idx{};    // associated end-point index for the reference path
};
```

Click [here](./docs/behavior_path_planner_path_generation_design.md) for details.

## References / External links

This module depends on the external [BehaviorTreeCpp](https://github.com/BehaviorTree/BehaviorTree.CPP) library.

<!-- cspell:ignore Vorobieva, Minoiu, Enache, Mammar, IFAC -->

[[1]](https://www.sciencedirect.com/science/article/pii/S1474667015347431) H. Vorobieva, S. Glaser, N. Minoiu-Enache, and S. Mammar, “Geometric path planning for automatic parallel parking in tiny spots”, IFAC Proceedings Volumes, vol. 45, no. 24, pp. 36–42, 2012.

## Future extensions / Unimplemented parts

-

## Related issues

-
