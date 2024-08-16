# autoware_external_cmd_selector

## Purpose

`autoware_external_cmd_selector` is the package to publish `external_control_cmd`, `gear_cmd`, `hazard_lights_cmd`, `heartbeat` and `turn_indicators_cmd`, according to the current mode, which is `remote` or `local`.

The current mode is set via service, `remote` is remotely operated, `local` is to use the values calculated by Autoware.

## Input / Output

### Input topics

| Name                                           | Type | Description                                             |
| ---------------------------------------------- | ---- | ------------------------------------------------------- |
| `/api/external/set/command/local/control`      | TBD  | Local. Calculated control value.                        |
| `/api/external/set/command/local/heartbeat`    | TBD  | Local. Heartbeat.                                       |
| `/api/external/set/command/local/shift`        | TBD  | Local. Gear shift like drive, rear and etc.             |
| `/api/external/set/command/local/turn_signal`  | TBD  | Local. Turn signal like left turn, right turn and etc.  |
| `/api/external/set/command/remote/control`     | TBD  | Remote. Calculated control value.                       |
| `/api/external/set/command/remote/heartbeat`   | TBD  | Remote. Heartbeat.                                      |
| `/api/external/set/command/remote/shift`       | TBD  | Remote. Gear shift like drive, rear and etc.            |
| `/api/external/set/command/remote/turn_signal` | TBD  | Remote. Turn signal like left turn, right turn and etc. |

### Output topics

| Name                                                   | Type                                              | Description                                     |
| ------------------------------------------------------ | ------------------------------------------------- | ----------------------------------------------- |
| `/control/external_cmd_selector/current_selector_mode` | TBD                                               | Current selected mode, remote or local.         |
| `/diagnostics`                                         | diagnostic_msgs::msg::DiagnosticArray             | Check if node is active or not.                 |
| `/external/selected/external_control_cmd`              | TBD                                               | Pass through control command with current mode. |
| `/external/selected/gear_cmd`                          | autoware_vehicle_msgs::msg::GearCommand           | Pass through gear command with current mode.    |
| `/external/selected/hazard_lights_cmd`                 | autoware_vehicle_msgs::msg::HazardLightsCommand   | Pass through hazard light with current mode.    |
| `/external/selected/heartbeat`                         | TBD                                               | Pass through heartbeat with current mode.       |
| `/external/selected/turn_indicators_cmd`               | autoware_vehicle_msgs::msg::TurnIndicatorsCommand | Pass through turn indicator with current mode.  |

## Parameters

{{json_to_markdown("control/autoware_external_cmd_selector/schema/external_cmd_selector.schema.json")}}
