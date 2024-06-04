# autoware_overlay_rviz_plugin

Plugin for displaying 2D overlays over the RViz2 3D scene.

Based on the [jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization)
package, under the 3-Clause BSD license.

## Purpose

This plugin provides a visual and easy-to-understand display of vehicle speed, turn signal, steering status and gears.

## Inputs / Outputs

### Input

| Name                                                    | Type                                                    | Description                          |
| ------------------------------------------------------- | ------------------------------------------------------- | ------------------------------------ |
| `/vehicle/status/velocity_status`                       | `autoware_vehicle_msgs::msg::VelocityReport`            | The topic is vehicle velocity        |
| `/vehicle/status/turn_indicators_status`                | `autoware_vehicle_msgs::msg::TurnIndicatorsReport`      | The topic is status of turn signal   |
| `/vehicle/status/hazard_status`                         | `autoware_vehicle_msgs::msg::HazardReport`              | The topic is status of hazard        |
| `/vehicle/status/steering_status`                       | `autoware_vehicle_msgs::msg::SteeringReport`            | The topic is status of steering      |
| `/vehicle/status/gear_status`                           | `autoware_vehicle_msgs::msg::GearReport`                | The topic is status of gear          |
| `/planning/scenario_planning/current_max_velocity`      | `tier4_planning_msgs::msg::VelocityLimit`               | The topic is velocity limit          |
| `/perception/traffic_light_recognition/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | The topic is status of traffic light |

## Parameter

### Core Parameters

#### SignalDisplay

| Name                     | Type   | Default Value        | Description                       |
| ------------------------ | ------ | -------------------- | --------------------------------- |
| `property_width_`        | int    | 128                  | Width of the plotter window [px]  |
| `property_height_`       | int    | 128                  | Height of the plotter window [px] |
| `property_left_`         | int    | 128                  | Left of the plotter window [px]   |
| `property_top_`          | int    | 128                  | Top of the plotter window [px]    |
| `property_signal_color_` | QColor | QColor(25, 255, 240) | Turn Signal color                 |

## Assumptions / Known limits

TBD.

## Usage

1. Start `rviz2` and click `Add` button under the `Displays` panel.

   ![select_add](./assets/images/select_add.png)

2. Under `By display type` tab, select `autoware_overlay_rviz_plugin/SignalDisplay` and press OK.

3. Enter the names of the topics if necessary.

   ![select_topic_name](./assets/images/select_topic_name.png)
