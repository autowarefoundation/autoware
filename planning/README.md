# Planning Components

## Getting Started

The Autoware.Universe Planning Modules represent a cutting-edge component within the broader open-source autonomous driving software stack. These modules play a pivotal role in autonomous vehicle navigation, skillfully handling route planning, dynamic obstacle avoidance, and real-time adaptation to varied traffic conditions.

- For high level concept of Planning Components, please refer to [Planning Component Design Document](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/planning/)
- To understand how Planning Components interacts with other components, please refer to [Planning Component Interface Document](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/planning/)
- The [Node Diagram](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/node-diagram/) illustrates the interactions, inputs, and outputs of all modules in the Autoware.Universe, including planning modules.

## Planning Module

The **Module** in the Planning Component refers to the various components that collectively form the planning system of the software. These modules cover a range of functionalities necessary for autonomous vehicle planning. Autoware's planning modules are modularized, meaning users can customize which functions are enabled by changing the configuration. This modular design allows for flexibility and adaptability to different scenarios and requirements in autonomous vehicle operations.

### How to Enable or Disable Planning Module

Enabling and disabling modules involves managing settings in key configuration and launch files.

### Key Files for Configuration

The `default_preset.yaml` file acts as the primary configuration file, where planning modules can be disable or enabled. Furthermore, users can also set the type of motion planner across various motion planners. For example:

- `launch_avoidance_module`: Set to `true` to enable the avoidance module, or `false` to disable it.
- `motion_stop_planner_type`: Set `default` to either `obstacle_stop_planner` or `obstacle_cruise_planner`.

!!! note

    Click [here](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/preset/default_preset.yaml) to view the `default_preset.yaml`.

The [launch files](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/tier4_planning_launch/launch/scenario_planning/lane_driving) reference the settings defined in `default_preset.yaml` to apply the configurations when the behavior path planner's node is running. For instance, the parameter `avoidance.enable_module` in

```xml
<param name="avoidance.enable_module" value="$(var launch_avoidance_module)"/>
```

corresponds to launch_avoidance_module from `default_preset.yaml`.

### Parameters configuration

There are multiple parameters available for configuration, and users have the option to modify them in [here](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config/planning). It's important to note that not all parameters are adjustable via `rqt_reconfigure`. To ensure the changes are effective, modify the parameters and then restart Autoware. Additionally, detailed information about each parameter is available in the corresponding documents under the planning tab.

### Integrating a Custom Module into Autoware: A Step-by-Step Guide

This guide outlines the steps for integrating your custom module into Autoware:

- Add your modules to the `default_preset.yaml` file. For example

```yaml
- arg:
  name: launch_intersection_module
  default: "true"
```

- Incorporate your modules into the [launcher](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/tier4_planning_launch/launch/scenario_planning). For example in [behavior_planning.launch.xml](https://github.com/autowarefoundation/autoware.universe/blob/main/launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml):

```xml
<arg name="launch_intersection_module" default="true"/>

<let
  name="behavior_velocity_planner_launch_modules"
  value="$(eval &quot;'$(var behavior_velocity_planner_launch_modules)' + 'behavior_velocity_planner::IntersectionModulePlugin, '&quot;)"
  if="$(var launch_intersection_module)"
/>
```

- If applicable, place your parameter folder within the appropriate existing parameter folder. For example [intersection_module's parameters](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/intersection.param.yaml) is in [behavior_velocity_planner](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner).
- Insert the path of your parameters in the [tier4_planning_component.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/tier4_planning_component.launch.xml). For example `behavior_velocity_planner_intersection_module_param_path` is used.

```xml
<arg name="behavior_velocity_planner_intersection_module_param_path" value="$(var behavior_velocity_config_path)/intersection.param.yaml"/>
```

- Define your parameter path variable within the corresponding launcher. For example in [behavior_planning.launch.xml](https://github.com/autowarefoundation/autoware.universe/blob/04aa54bf5fb0c88e70198ca74b9ac343cc3457bf/launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml#L191)

```xml
<param from="$(var behavior_velocity_planner_intersection_module_param_path)"/>
```

!!! note

    Depending on the specific module you wish to add, the relevant files and steps may vary. This guide provides a general overview and serves as a starting point. It's important to adapt these instructions to the specifics of your module.

## Join Our Community-Driven Effort

Autoware thrives on community collaboration. Every contribution, big or small, is invaluable to us. Whether it's reporting bugs, suggesting improvements, offering new ideas, or anything else you can think of – we welcome it all with open arms.

### How to Contribute?

Ready to contribute? Great! To get started, simply visit our [Contributing Guidelines](https://autowarefoundation.github.io/autoware-documentation/main/contributing/) where you'll find all the information you need to jump in. This includes instructions on submitting bug reports, proposing feature enhancements, and even contributing to the codebase.

### Join Our Planning & Control Working Group Meetings

The Planning & Control working group is an integral part of our community. We meet bi-weekly to discuss our current progress, upcoming challenges, and brainstorm new ideas. These meetings are a fantastic opportunity to directly contribute to our discussions and decision-making processes.

Meeting Details:

- **Frequency:** Bi-weekly
- **Day:** Thursday
- **Time:** 08:00 AM UTC (05:00 PM JST)
- **Agenda:** Discuss current progress, plan future developments. You can view and comment on the minutes of past meetings [here](https://github.com/orgs/autowarefoundation/discussions?discussions_q=is%3Aopen+label%3Ameeting%3Aplanning-control-wg+).

Interested in joining our meetings? We’d love to have you! For more information on how to participate, visit the following link: [How to participate in the working group](https://github.com/autowarefoundation/autoware-projects/wiki/Autoware-Planning-Control-Working-Group#how-to-participate-in-the-working-group).

### Citations

Occasionally, we publish papers specific to the Planning Component in Autoware. We encourage you to explore these publications and find valuable insights for your work. If you find them useful and incorporate any of our methodologies or algorithms in your projects, citing our papers would be immensely helpful. This support allows us to reach a broader audience and continue contributing to the field.

If you use the Jerk Constrained Velocity Planning algorithm in [Motion Velocity Smoother](./autoware_velocity_smoother/README.md) module in the Planning Component, we kindly request you to cite the relevant paper.

<!-- cspell:ignore Shimizu, Horibe, Watanabe, Kato -->

Y. Shimizu, T. Horibe, F. Watanabe and S. Kato, "[Jerk Constrained Velocity Planning for an Autonomous Vehicle: Linear Programming Approach](https://arxiv.org/abs/2202.10029)," 2022 International Conference on Robotics and Automation (ICRA)

```tex
@inproceedings{shimizu2022,
  author={Shimizu, Yutaka and Horibe, Takamasa and Watanabe, Fumiya and Kato, Shinpei},
  booktitle={2022 International Conference on Robotics and Automation (ICRA)},
  title={Jerk Constrained Velocity Planning for an Autonomous Vehicle: Linear Programming Approach},
  year={2022},
  pages={5814-5820},
  doi={10.1109/ICRA46639.2022.9812155}}
```
