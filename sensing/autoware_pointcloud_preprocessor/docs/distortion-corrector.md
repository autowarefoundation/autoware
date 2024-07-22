# distortion_corrector

## Purpose

The `distortion_corrector` is a node that compensates for pointcloud distortion caused by the ego-vehicle's movement during one scan.

Since the LiDAR sensor scans by rotating an internal laser, the resulting point cloud will be distorted if the ego-vehicle moves during a single scan (as shown by the figure below). The node corrects this by interpolating sensor data using the odometry of the ego-vehicle.

## Inner-workings / Algorithms

The node uses twist information (linear and angular velocity) from the `~/input/twist` topic to correct each point in the point cloud. If the user sets `use_imu` to true, the node will replace the twist's angular velocity with the angular velocity from IMU.

The node supports two different modes of distortion correction: 2D distortion correction and 3D distortion correction. The main difference is that the 2D distortion corrector only utilizes the x-axis of linear velocity and the z-axis of angular velocity to correct the point positions. On the other hand, the 3D distortion corrector utilizes all linear and angular velocity components to correct the point positions.

Please note that the processing time difference between the two distortion methods is significant; the 3D corrector takes 50% more time than the 2D corrector. Therefore, it is recommended that in general cases, users should set `use_3d_distortion_correction` to `false`. However, in scenarios such as a vehicle going over speed bumps, using the 3D corrector can be beneficial.

![distortion corrector figure](./image/distortion_corrector.jpg)

## Inputs / Outputs

### Input

| Name                 | Type                                             | Description                        |
| -------------------- | ------------------------------------------------ | ---------------------------------- |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2`                  | Topic of the distorted pointcloud. |
| `~/input/twist`      | `geometry_msgs::msg::TwistWithCovarianceStamped` | Topic of the twist information.    |
| `~/input/imu`        | `sensor_msgs::msg::Imu`                          | Topic of the IMU data.             |

### Output

| Name                  | Type                            | Description                         |
| --------------------- | ------------------------------- | ----------------------------------- |
| `~/output/pointcloud` | `sensor_msgs::msg::PointCloud2` | Topic of the undistorted pointcloud |

## Parameters

### Core Parameters

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/distortion_corrector_node.schema.json") }}

## Launch

```bash
ros2 launch autoware_pointcloud_preprocessor distortion_corrector.launch.xml
```

## Assumptions / Known limits

- The node requires time synchronization between the topics from lidars, twist, and IMU.
- If you want to use a 3D distortion corrector without IMU, please check that the linear and angular velocity fields of your twist message are not empty.
