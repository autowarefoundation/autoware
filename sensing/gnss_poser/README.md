# gnss_poser

## Purpose

The `gnss_poser` is a node that subscribes gnss sensing messages and calculates vehicle pose with covariance.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                           | Type                                                    | Description                                                                                                                    |
| ------------------------------ | ------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `/map/map_projector_info`      | `tier4_map_msgs::msg::MapProjectorInfo`                 | map projection info                                                                                                            |
| `~/input/fix`                  | `sensor_msgs::msg::NavSatFix`                           | gnss status message                                                                                                            |
| `~/input/autoware_orientation` | `autoware_sensing_msgs::msg::GnssInsOrientationStamped` | orientation [click here for more details](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_sensing_msgs) |

### Output

| Name                     | Type                                            | Description                                                    |
| ------------------------ | ----------------------------------------------- | -------------------------------------------------------------- |
| `~/output/pose`          | `geometry_msgs::msg::PoseStamped`               | vehicle pose calculated from gnss sensing data                 |
| `~/output/gnss_pose_cov` | `geometry_msgs::msg::PoseWithCovarianceStamped` | vehicle pose with covariance calculated from gnss sensing data |
| `~/output/gnss_fixed`    | `tier4_debug_msgs::msg::BoolStamped`            | gnss fix status                                                |

## Parameters

### Core Parameters

{{ json_to_markdown("sensing/gnss_poser/schema/gnss_poser.schema.json") }}

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
