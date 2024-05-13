# autoware_mission_details_overlay_rviz_plugin

This RViz plugin displays the remaining distance and time for the current mission.

## Inputs / Outputs

### Input

| Name                                        | Type                                                        | Description                                          |
| ------------------------------------------- | ----------------------------------------------------------- | ---------------------------------------------------- |
| `/planning/mission_remaining_distance_time` | `autoware_planning_msgs::msg::MissionRemainingDistanceTime` | The topic is for mission remaining distance and time |

## Overlay Parameters

| Name     | Type | Default Value | Description                       |
| -------- | ---- | ------------- | --------------------------------- |
| `Width`  | int  | 170           | Width of the overlay [px]         |
| `Height` | int  | 100           | Height of the overlay [px]        |
| `Right`  | int  | 10            | Margin from the right border [px] |
| `Top`    | int  | 10            | Margin from the top border [px]   |

The mission details display is aligned with top right corner of the screen.

## Usage

Similar to [autoware_overlay_rviz_plugin](../autoware_overlay_rviz_plugin/README.md)

## Credits

Based on the [jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization) package.

### Icons

- <https://fonts.google.com/icons?selected=Material+Symbols+Outlined:conversion_path:FILL@1;wght@400;GRAD@200;opsz@20&icon.size=20&icon.color=%23e8eaed&icon.query=path>
- <https://fonts.google.com/icons?selected=Material+Symbols+Outlined:av_timer:FILL@1;wght@400;GRAD@200;opsz@20&icon.size=20&icon.color=%23e8eaed&icon.query=av+timer>
