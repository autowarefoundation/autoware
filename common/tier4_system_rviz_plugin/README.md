# tier4_system_rviz_plugin

## Purpose

This plugin display the Hazard information from Autoware; and output notices when emergencies are from initial localization and route setting.

## Input

| Name                              | Type                                             | Description                                                  |
| --------------------------------- | ------------------------------------------------ | ------------------------------------------------------------ |
| `/system/emergency/hazard_status` | `autoware_system_msgs::msg::HazardStatusStamped` | The topic represents the emergency information from Autoware |
