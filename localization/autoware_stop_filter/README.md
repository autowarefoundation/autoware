# stop_filter

## Purpose

When this function did not exist, each node used a different criterion to determine whether the vehicle is stopping or not, resulting that some nodes were in operation of stopping the vehicle and some nodes continued running in the drive mode.
This node aims to:

- apply a uniform stopping decision criterion to several nodes.
- suppress the control noise by overwriting the velocity and angular velocity with zero.

## Inputs / Outputs

### Input

| Name         | Type                      | Description           |
| ------------ | ------------------------- | --------------------- |
| `input/odom` | `nav_msgs::msg::Odometry` | localization odometry |

### Output

| Name              | Type                                 | Description                                              |
| ----------------- | ------------------------------------ | -------------------------------------------------------- |
| `output/odom`     | `nav_msgs::msg::Odometry`            | odometry with suppressed longitudinal and yaw twist      |
| `debug/stop_flag` | `tier4_debug_msgs::msg::BoolStamped` | flag to represent whether the vehicle is stopping or not |

## Parameters

{{ json_to_markdown("localization/stop_filter/schema/stop_filter.schema.json") }}
