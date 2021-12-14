# tier4_state_rviz_plugin

## Purpose

This plugin displays the current status of autoware.  
This plugin also can engage from the panel.

## Inputs / Outputs

### Input

| Name                          | Type                                            | Description                                        |
| ----------------------------- | ----------------------------------------------- | -------------------------------------------------- |
| `/control/current_gate_mode`  | `tier4_control_msgs::msg::GateMode`             | The topic represents the state of AUTO or EXTERNAL |
| `/autoware/state`             | `autoware_auto_system_msgs::msg::AutowareState` | The topic represents the state of Autoware         |
| `/vehicle/status/gear_status` | `autoware_auto_vehicle_msgs::msg::GearReport`   | The topic represents the state of Gear             |
| `/api/external/get/engage`    | `tier4_external_api_msgs::msg::EngageStatus`    | The topic represents the state of Engage           |

### Output

| Name                       | Type                                   | Description                    |
| -------------------------- | -------------------------------------- | ------------------------------ |
| `/api/external/set/engage` | `tier4_external_api_msgs::srv::Engage` | The service inputs engage true |

## HowToUse

1. Start rviz and select panels/Add new panel.
   ![select_panel](./images/select_panels.png)
2. Select tier4_state_rviz_plugin/AutowareStatePanel and press OK.
   ![select_state_plugin](./images/select_state_plugin.png)
3. If the AutowareState is WaitingForEngage, can engage by clicking the Engage button.
   ![select_engage](./images/select_engage.png)
