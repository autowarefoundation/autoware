# Motion Velocity Planner

## Overview

`motion_velocity_planner` is a planner to adjust the trajectory velocity based on the obstacles around the vehicle.
It loads modules as plugins. Please refer to the links listed below for detail on each module.

![Architecture](./docs/MotionVelocityPlanner-InternalInterface.drawio.svg)

- [Out of Lane](../autoware_motion_velocity_out_of_lane_module/README.md)

Each module calculates stop and slow down points to be inserted in the ego trajectory.
These points are assumed to correspond to the `base_link` frame of the ego vehicle as it follows the trajectory.
This means that to stop before a wall, a stop point is inserted in the trajectory at a distance ahead of the wall equal to the vehicle front offset (wheelbase + front overhang, see the [vehicle dimensions](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-dimensions/).

![set_stop_velocity](./docs/set_stop_velocity.drawio.svg)

## Input topics

| Name                                   | Type                                                  | Description                   |
| -------------------------------------- | ----------------------------------------------------- | ----------------------------- |
| `~/input/trajectory`                   | autoware_planning_msgs::msg::Trajectory               | input trajectory              |
| `~/input/vector_map`                   | autoware_map_msgs::msg::LaneletMapBin                 | vector map                    |
| `~/input/vehicle_odometry`             | nav_msgs::msg::Odometry                               | vehicle position and velocity |
| `~/input/accel`                        | geometry_msgs::msg::AccelWithCovarianceStamped        | vehicle acceleration          |
| `~/input/dynamic_objects`              | autoware_perception_msgs::msg::PredictedObjects       | dynamic objects               |
| `~/input/no_ground_pointcloud`         | sensor_msgs::msg::PointCloud2                         | obstacle pointcloud           |
| `~/input/traffic_signals`              | autoware_perception_msgs::msg::TrafficLightGroupArray | traffic light states          |
| `~/input/virtual_traffic_light_states` | tier4_v2x_msgs::msg::VirtualTrafficLightStateArray    | virtual traffic light states  |
| `~/input/occupancy_grid`               | nav_msgs::msg::OccupancyGrid                          | occupancy grid                |

## Output topics

| Name                        | Type                                              | Description                                        |
| --------------------------- | ------------------------------------------------- | -------------------------------------------------- |
| `~/output/trajectory`       | autoware_planning_msgs::msg::Trajectory           | Ego trajectory with updated velocity profile       |
| `~/output/velocity_factors` | autoware_adapi_v1_msgs::msg::VelocityFactorsArray | factors causing change in the ego velocity profile |

## Services

| Name                      | Type                                                     | Description                  |
| ------------------------- | -------------------------------------------------------- | ---------------------------- |
| `~/service/load_plugin`   | autoware_motion_velocity_planner_node::srv::LoadPlugin   | To request loading a plugin  |
| `~/service/unload_plugin` | autoware_motion_velocity_planner_node::srv::UnloadPlugin | To request unloaded a plugin |

## Node parameters

| Parameter        | Type             | Description            |
| ---------------- | ---------------- | ---------------------- |
| `launch_modules` | vector\<string\> | module names to launch |

In addition, the following parameters should be provided to the node:

- [nearest search parameters](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/common/nearest_search.param.yaml);
- [vehicle info parameters](https://github.com/autowarefoundation/sample_vehicle_launch/blob/main/sample_vehicle_description/config/vehicle_info.param.yaml);
- [common planning parameters](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/common/common.param.yaml);
- [smoother parameters](https://autowarefoundation.github.io/autoware.universe/main/planning/autoware_velocity_smoother/#parameters)
- Parameters of each plugin that will be loaded.
