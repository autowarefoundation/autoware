# front_vehicle_velocity_estimator

This package contains a front vehicle velocity estimation for offline perception module analysis.
This package can:

- Attach velocity to 3D detections from velocity estimation with LiDAR pointcloud.

## Algorithm

- Processing flow
  1. Choose front vehicle from front area objects.
  2. Choose nearest neighbor point within front vehicle.
  3. Estimate velocity of front vehicle by using the differentiated value from time series of nearest neighbor point positions.
  4. Compensate ego vehicle twist

## Input

| Name                 | Type                                                 | Description          |
| -------------------- | ---------------------------------------------------- | -------------------- |
| `~/input/objects`    | autoware_auto_perception_msgs/msg/DetectedObject.msg | 3D detected objects. |
| `~/input/pointcloud` | sensor_msgs/msg/PointCloud2.msg                      | LiDAR pointcloud.    |
| `~/input/odometry`   | nav_msgs::msg::Odometry.msg                          | Odometry data.       |

## Output

| Name                                  | Type                                                  | Description                                   |
| ------------------------------------- | ----------------------------------------------------- | --------------------------------------------- |
| `~/output/objects`                    | autoware_auto_perception_msgs/msg/DetectedObjects.msg | 3D detected object with twist.                |
| `~/debug/nearest_neighbor_pointcloud` | sensor_msgs/msg/PointCloud2.msg                       | The pointcloud msg of nearest neighbor point. |

## Node parameter

| Name             | Type   | Description           | Default value |
| :--------------- | :----- | :-------------------- | :------------ |
| `update_rate_hz` | double | The update rate [hz]. | 10.0          |

## Core parameter

| Name                          | Type   | Description                                                                                                                                                                                                                                                                 | Default value |
| :---------------------------- | :----- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `moving_average_num`          | int    | The moving average number for velocity estimation.                                                                                                                                                                                                                          | 1             |
| `threshold_pointcloud_z_high` | float  | The threshold for z position value of point when choosing nearest neighbor point within front vehicle [m]. If z > `threshold_pointcloud_z_high`, the point is considered to noise.                                                                                          | 1.0f          |
| `threshold_pointcloud_z_low`  | float  | The threshold for z position value of point when choosing nearest neighbor point within front vehicle [m]. If z < `threshold_pointcloud_z_low`, the point is considered to noise like ground.                                                                               | 0.6f          |
| `threshold_relative_velocity` | double | The threshold for min and max of estimated relative velocity ($v_{re}$) [m/s]. If $v_{re}$ < - `threshold_relative_velocity` , then $v_{re}$ = - `threshold_relative_velocity`. If $v_{re}$ > `threshold_relative_velocity`, then $v_{re}$ = `threshold_relative_velocity`. | 10.0          |
| `threshold_absolute_velocity` | double | The threshold for max of estimated absolute velocity ($v_{ae}$) [m/s]. If $v_{ae}$ > `threshold_absolute_velocity`, then $v_{ae}$ = `threshold_absolute_velocity`.                                                                                                          | 20.0          |
