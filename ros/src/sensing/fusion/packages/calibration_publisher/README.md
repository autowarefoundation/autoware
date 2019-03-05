# Calibration Publisher

This nodes publishes the camera intrinsics, extrinsics and registers the TF between the camera and LiDAR sensors.
The data is read from an Autoware compatible calibration file.

## Publications

* `sensor_msgs/CameraInfo`, default topic name `/NAMESPACE/camera_info`.

* `autoware_msgs/projection_matrix`, default topic name `/NAMESPACE/camera_info`

`NAMESPACE` if any.

## Subscriptions

* `sensor_msgs/Image`, default topic name `/image_raw`

## Parameters

|         Parameter        | Type  |           Description                                | Default Value |
|--------------------------|-------|------------------------------------------------------|---------------|
|`calibration_file`        | String|Path to the Autoware calibration file                 |               |
|`register_lidar2camera_tf`| Bool  |Whether or not to register TF between camera and lidar| `True`        |
|`publish_extrinsic_mat`   | Bool  |Whether or not to publish extrinsic_matrix topic      | `True`        |
|`publish_camera_info`     | Bool  |Whether or not to publish camera_info topic           | `True`        |
|`camera_frame`            | String|Camera frame to be included in the header             | `camera`      |
|`target_frame`            | String|Target LiDAR frame to register in the TF              | `velodyne`    |
|`image_topic_src`         | String|Image source topic to synchronize CameraInfo          | `/image_raw`  |
|`camera_info_topic`       | String|Camera info topic name to publish intrinsics          | `/camera_info`|
|`projection_matrix_topic` | String|Topic name on which to publish Extrinsics             | `/projection_matrix`|

