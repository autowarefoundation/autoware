# gnss_poser

## Purpose

The `gnss_poser` is a node that subscribes gnss sensing messages and calculates vehicle pose with covariance.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name             | Type                          | Description                                                                                     |
| ---------------- | ----------------------------- | ----------------------------------------------------------------------------------------------- |
| `~/input/fix`    | `sensor_msgs::msg::NavSatFix` | gnss status message                                                                             |
| `~/input/navpvt` | `ublox_msgs::msg::NavPVT`     | position, velocity and time solution (You can see detail description in reference document [1]) |

### Output

| Name                     | Type                                            | Description                                                    |
| ------------------------ | ----------------------------------------------- | -------------------------------------------------------------- |
| `~/output/pose`          | `geometry_msgs::msg::PoseStamped`               | vehicle pose calculated from gnss sensing data                 |
| `~/output/gnss_pose_cov` | `geometry_msgs::msg::PoseWithCovarianceStamped` | vehicle pose with covariance calculated from gnss sensing data |
| `~/output/gnss_fixed`    | `autoware_debug_msgs::msg::BoolStamped`         | gnss fix status                                                |

## Parameters

### Core Parameters

| Name                 | Type   | Default Value    | Description                                                                                     |
| -------------------- | ------ | ---------------- | ----------------------------------------------------------------------------------------------- |
| `base_frame`         | string | "base_link"      | frame d                                                                                         |
| `gnss_frame`         | string | "gnss"           | frame id                                                                                        |
| `gnss_base_frame`    | string | "gnss_base_link" | frame id                                                                                        |
| `map_frame`          | string | "map"            | frame id                                                                                        |
| `use_ublox_receiver` | bool   | false            | flag to use ublox receiver                                                                      |
| `plane_zone`         | int    | 9                | identification number of the plane rectangular coordinate systems (See, reference document [2]) |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

[1] <https://github.com/KumarRobotics/ublox.git>

[2] <https://www.gsi.go.jp/LAW/heimencho.html>

## (Optional) Future extensions / Unimplemented parts
