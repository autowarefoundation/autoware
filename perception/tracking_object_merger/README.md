# Tracking Object Merger

## Purpose

This package try to merge two tracking objects from different sensor.

## Inner-workings / Algorithms

Merging tracking objects from different sensor is a combination of data association and state fusion algorithms.

Detailed process depends on the merger policy.

### decorative_tracker_merger

In decorative_tracker_merger, we assume there are dominant tracking objects and sub tracking objects.
The name `decorative` means that sub tracking objects are used to complement the main objects.

Usually the dominant tracking objects are from LiDAR and sub tracking objects are from Radar or Camera.

Here show the processing pipeline.

![decorative_tracker_merger](./image/decorative_tracker_merger.drawio.svg)

#### time sync

Sub object(Radar or Camera) often has higher frequency than dominant object(LiDAR). So we need to sync the time of sub object to dominant object.

![time sync](image/time_sync.drawio.svg)

#### data association

In the data association, we use the following rules to determine whether two tracking objects are the same object.

- gating
  - `distance gate`: distance between two tracking objects
  - `angle gate`: angle between two tracking objects
  - `mahalanobis_distance_gate`: Mahalanobis distance between two tracking objects
  - `min_iou_gate`: minimum IoU between two tracking objects
  - `max_velocity_gate`: maximum velocity difference between two tracking objects
- score
  - score used in matching is equivalent to the distance between two tracking objects

#### tracklet update

Sub tracking objects are merged into dominant tracking objects.

Depends on the tracklet input sensor state, we update the tracklet state with different rules.

| state\priority             | 1st    | 2nd   | 3rd    |
| -------------------------- | ------ | ----- | ------ |
| Kinematics except velocity | LiDAR  | Radar | Camera |
| Forward velocity           | Radar  | LiDAR | Camera |
| Object classification      | Camera | LiDAR | Radar  |

#### tracklet management

We use the `existence_probability` to manage tracklet.

- When we create a new tracklet, we set the `existence_probability` to $p_{sensor}$ value.
- In each update with specific sensor, we set the `existence_probability` to $p_{sensor}$ value.
- When tracklet does not have update with specific sensor, we reduce the `existence_probability` by `decay_rate`
- Object can be published if `existence_probability` is larger than `publish_probability_threshold`
- Object will be removed if `existence_probability` is smaller than `remove_probability_threshold`

![tracklet_management](./image/tracklet_management.drawio.svg)

These parameter can be set in `config/decorative_tracker_merger.param.yaml`.

```yaml
tracker_state_parameter:
  remove_probability_threshold: 0.3
  publish_probability_threshold: 0.6
  default_lidar_existence_probability: 0.7
  default_radar_existence_probability: 0.6
  default_camera_existence_probability: 0.6
  decay_rate: 0.1
  max_dt: 1.0
```

#### input/parameters

| topic name                      | message type                                    | description                                                                           |
| ------------------------------- | ----------------------------------------------- | ------------------------------------------------------------------------------------- |
| `~input/main_object`            | `autoware_auto_perception_msgs::TrackedObjects` | Dominant tracking objects. Output will be published with this dominant object stamps. |
| `~input/sub_object`             | `autoware_auto_perception_msgs::TrackedObjects` | Sub tracking objects.                                                                 |
| `output/object`                 | `autoware_auto_perception_msgs::TrackedObjects` | Merged tracking objects.                                                              |
| `debug/interpolated_sub_object` | `autoware_auto_perception_msgs::TrackedObjects` | Interpolated sub tracking objects.                                                    |

Default parameters are set in [config/decorative_tracker_merger.param.yaml](./config/decorative_tracker_merger.param.yaml).

| parameter name            | description                                                                                                                                                      | default value |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| `base_link_frame_id`      | base link frame id. This is used to transform the tracking object.                                                                                               | "base_link"   |
| `time_sync_threshold`     | time sync threshold. If the time difference between two tracking objects is smaller than this value, we consider these two tracking objects are the same object. | 0.05          |
| `sub_object_timeout_sec`  | sub object timeout. If the sub object is not updated for this time, we consider this object is not exist.                                                        | 0.5           |
| `main_sensor_type`        | main sensor type. This is used to determine the dominant tracking object.                                                                                        | "lidar"       |
| `sub_sensor_type`         | sub sensor type. This is used to determine the sub tracking object.                                                                                              | "radar"       |
| `tracker_state_parameter` | tracker state parameter. This is used to manage the tracklet.                                                                                                    |               |

- the detail of `tracker_state_parameter` is described in [tracklet management](#tracklet-management)

#### tuning

As explained in [tracklet management](#tracklet-management), this tracker merger tend to maintain the both input tracking objects.

If there are many false positive tracking objects,

- decrease `default_<sensor>_existence_probability` of that sensor
- increase `decay_rate`
- increase `publish_probability_threshold` to publish only reliable tracking objects

### equivalent_tracker_merger

This is future work.
