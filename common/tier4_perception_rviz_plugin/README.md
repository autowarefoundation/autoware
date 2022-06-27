# tier4_perception_rviz_plugin

## Purpose

This plugin is used to generate dummy pedestrians, cars, and obstacles in planning simulator.

## Overview

The CarInitialPoseTool sends a topic for generating a dummy car.  
The PedestrianInitialPoseTool sends a topic for generating a dummy pedestrian.  
The UnknownInitialPoseTool sends a topic for generating a dummy obstacle.  
The DeleteAllObjectsTool deletes the dummy cars, pedestrians, and obstacles displayed by the above three tools.

## Inputs / Outputs

### Output

| Name                                                 | Type                                      | Description                                     |
| ---------------------------------------------------- | ----------------------------------------- | ----------------------------------------------- |
| `/simulation/dummy_perception_publisher/object_info` | `dummy_perception_publisher::msg::Object` | The topic on which to publish dummy object info |

## Parameter

### Core Parameters

#### CarPose

| Name              | Type   | Default Value                                        | Description                                     |
| ----------------- | ------ | ---------------------------------------------------- | ----------------------------------------------- |
| `topic_property_` | string | `/simulation/dummy_perception_publisher/object_info` | The topic on which to publish dummy object info |
| `std_dev_x_`      | float  | 0.03                                                 | X standard deviation for initial pose [m]       |
| `std_dev_y_`      | float  | 0.03                                                 | Y standard deviation for initial pose [m]       |
| `std_dev_z_`      | float  | 0.03                                                 | Z standard deviation for initial pose [m]       |
| `std_dev_theta_`  | float  | 5.0 \* M_PI / 180.0                                  | Theta standard deviation for initial pose [rad] |
| `length_`         | float  | 4.0                                                  | X standard deviation for initial pose [m]       |
| `width_`          | float  | 1.8                                                  | Y standard deviation for initial pose [m]       |
| `height_`         | float  | 2.0                                                  | Z standard deviation for initial pose [m]       |
| `position_z_`     | float  | 0.0                                                  | Z position for initial pose [m]                 |
| `velocity_`       | float  | 0.0                                                  | Velocity [m/s]                                  |

#### BusPose

| Name              | Type   | Default Value                                        | Description                                     |
| ----------------- | ------ | ---------------------------------------------------- | ----------------------------------------------- |
| `topic_property_` | string | `/simulation/dummy_perception_publisher/object_info` | The topic on which to publish dummy object info |
| `std_dev_x_`      | float  | 0.03                                                 | X standard deviation for initial pose [m]       |
| `std_dev_y_`      | float  | 0.03                                                 | Y standard deviation for initial pose [m]       |
| `std_dev_z_`      | float  | 0.03                                                 | Z standard deviation for initial pose [m]       |
| `std_dev_theta_`  | float  | 5.0 \* M_PI / 180.0                                  | Theta standard deviation for initial pose [rad] |
| `length_`         | float  | 10.5                                                 | X standard deviation for initial pose [m]       |
| `width_`          | float  | 2.5                                                  | Y standard deviation for initial pose [m]       |
| `height_`         | float  | 3.5                                                  | Z standard deviation for initial pose [m]       |
| `position_z_`     | float  | 0.0                                                  | Z position for initial pose [m]                 |
| `velocity_`       | float  | 0.0                                                  | Velocity [m/s]                                  |

#### PedestrianPose

| Name              | Type   | Default Value                                        | Description                                     |
| ----------------- | ------ | ---------------------------------------------------- | ----------------------------------------------- |
| `topic_property_` | string | `/simulation/dummy_perception_publisher/object_info` | The topic on which to publish dummy object info |
| `std_dev_x_`      | float  | 0.03                                                 | X standard deviation for initial pose [m]       |
| `std_dev_y_`      | float  | 0.03                                                 | Y standard deviation for initial pose [m]       |
| `std_dev_z_`      | float  | 0.03                                                 | Z standard deviation for initial pose [m]       |
| `std_dev_theta_`  | float  | 5.0 \* M_PI / 180.0                                  | Theta standard deviation for initial pose [rad] |
| `position_z_`     | float  | 0.0                                                  | Z position for initial pose [m]                 |
| `velocity_`       | float  | 0.0                                                  | Velocity [m/s]                                  |

#### UnknownPose

| Name              | Type   | Default Value                                        | Description                                     |
| ----------------- | ------ | ---------------------------------------------------- | ----------------------------------------------- |
| `topic_property_` | string | `/simulation/dummy_perception_publisher/object_info` | The topic on which to publish dummy object info |
| `std_dev_x_`      | float  | 0.03                                                 | X standard deviation for initial pose [m]       |
| `std_dev_y_`      | float  | 0.03                                                 | Y standard deviation for initial pose [m]       |
| `std_dev_z_`      | float  | 0.03                                                 | Z standard deviation for initial pose [m]       |
| `std_dev_theta_`  | float  | 5.0 \* M_PI / 180.0                                  | Theta standard deviation for initial pose [rad] |
| `position_z_`     | float  | 0.0                                                  | Z position for initial pose [m]                 |
| `velocity_`       | float  | 0.0                                                  | Velocity [m/s]                                  |

#### DeleteAllObjects

| Name              | Type   | Default Value                                        | Description                                     |
| ----------------- | ------ | ---------------------------------------------------- | ----------------------------------------------- |
| `topic_property_` | string | `/simulation/dummy_perception_publisher/object_info` | The topic on which to publish dummy object info |

## Assumptions / Known limits

Using a planning simulator

## Usage

1. Start rviz and select + on the tool tab.
   ![select_add](./images/select_add.png)
2. Select one of the following: tier4_perception_rviz_plugin and press OK.
   ![select_plugin](./images/select_plugin.png)
3. Select the new item in the tool tab (2D Dummy Car in the example) and click on it in rviz.
   ![select_dummy_car](./images/select_dummy_car.png)

### Interactive manipulation

You can interactively manipulate the object.

1. Select "Tool Properties" in rviz.
2. Select the corresponding object tab in the Tool Properties.
3. Turn the "Interactive" checkbox on.
   ![tool_properties](./images/tool_properties.png)
4. Select the item in the tool tab in you haven't chosen yet.
5. Key commands are as follows.

| action | key command                            |
| ------ | -------------------------------------- |
| ADD    | Shift + Click Right Button             |
| MOVE   | Hold down Right Button + Drug and Drop |
| DELETE | Alt + Click Right Button               |
