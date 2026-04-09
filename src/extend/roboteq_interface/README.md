# roboteq_interface

The controller packages for vehicles using roboteq motor controller

## Overview

![roboteq interface](docs/roboteq_interface.svg "roboteq interface")

## Directory Structure

| Directory Name                                        | Description                                                                     |
| :---------------------------------------------------- | :------------------------------------------------------------------------------ |
| [controller_profiles](controller_profiles/)           | Configuration profiles for roboteq motor controller.                            |
| [roboteq_can_interface](roboteq_can_interface/)       | ROS2 interface package for roboteq motor controller using CAN communication.    |
| [roboteq_interface](roboteq_interface/)               | Interface package for connecting autoware and roboteq ROS2 interface.           |
| [roboteq_msgs](roboteq_msgs/)                         | Custom ROS2 msg files for the roboteq interface.                                |
| [roboteq_serial_interface](roboteq_serial_interface/) | ROS2 interface package for roboteq motor controller using serial communication. |
