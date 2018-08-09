# Lidar fake perception

This node generates fake object and pointcloud message based on the value given manually.
At the sametime, real pointclouds and real detected objects can be merged as sources.

### How to launch

* From Runtime Manager:

Computing -> Detection -> lidar_detector -> lidar_fake_perception

* From CLI:

`$ roslaunch lidar_fake_perception lidar_fake_perception.launch`

### Parameters

Parameters can be set in both Launch file and Runtime manager:

| Parameter in RM | Parameter in Launch | Type | Description | Default |
| --- | --- | --- | --- | --- |
| `initial pose` | `initial_pose_topic` | *String* | Initial fake object pose | `/move_base_simple/goal` |
| `input objects` | `real_objects_topic` | *String* | Input objects | `/detected_objects` |
| `input points` | `real_points_topic` | *String* | Input points | `/points_raw` |
| `input twist` | `fake_twist_topic` | *String* | Input twist command, controlling fake object | `/fake_twist` |
| `output objects` | `fake_objects_topic` | *String* | Output objects, input and fake object are merged | `/fake_objects` |
| `output points` | `fake_points_topic` | *String* | Output points, input and fake points are merged | `/fake_points` |
| `publish object` | `publish_objects` | *Bool* | Enable publishing fake objects | `true` |
| `publish points` | `publish_points` | *Bool* | Enable publishing fake points | `true` |
| `publish rate` | `publish_rate` | *Double* | Publish rate of fake objects/points [Hz] | `10.0` |
| `object length` | `object_length` | *Double* | Length [m] | `4.8` |
| `object width` | `object_width` | *Double* | Width [m] | `1.8` |
| `object height` | `object_height` | *Double* | Height [m] | `1.8` |
| `object z offset` | `object_z_offset` | *Double* | Z Offset from global frame [m] | `0.0` |
| `use input twist` | `use_fake_twist` | *Bool* | Using subscribed twist | `false` |
| `object velocity` | `object_velocity` | *Double* | Constant velocity instead of subscribed twist [m/s] | `3.0` |
| `object angular velocity` | `object_angular_velocity` | *Double* | Constant angular velocity instead of subscribed twist [rad/s] | `0.0` |
| `object points intensity` | `object_intensity` | *Double* | Constant intensity value of fake points, 0-255 [-] | `100.0` |
| `object lifetime` | `object_lifetime` | *Double* | Fake object lifetime (NOTE: when this is negative value, lifetime is inifinity) [s] | `-1` |
| `object points space` | `object_points_space` | *Double* | Fake points space [m] | `0.2` |
| `output label` | `object_label` | *String* | Fake object label (e.g. tracking state) | `Stable` |
| `output frame` | `object_frame` | *String* | Fake object frame_id (NOTE: not affected to input object) | `velodyne` |

### Subscriptions/Publications

```
Node [/lidar_fake_percetion]
Publications:
 * /fake_objects [autoware_msgs/DetectedObjectArray]
 * /fake_points [sensor_msgs/PointCloud2]

Subscriptions:
 * /move_base_simple/goal [geometry_msgs/PoseStamped]
 * /detected_objects [autoware_msgs/DetectedObjectArray]
 * /points_raw [sensor_msgs/PointCloud2]
 * /fake_twist [geometry_msgs/Twist]
 * /tf [tf2_msgs/TFMessage]
```

[![Demo, constant velocity (not using input twist)](https://img.youtube.com/vi/F7yCnQz1FmQ/sddefault.jpg)](https://youtu.be/F7yCnQz1FmQ)

[![Demo, manual control (using input twist)](https://img.youtube.com/vi/1BHi5I5jTGg/sddefault.jpg)](https://youtu.be/1BHi5I5jTGg)
