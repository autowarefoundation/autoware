# Reaction Analyzer

## Description

The main purpose of the reaction analyzer package is to measure the reaction times of various nodes within a ROS-based
autonomous driving simulation environment by subscribing to pre-determined topics. This tool is particularly useful for
evaluating the performance of perception, planning, and control pipelines in response to dynamic changes in the
environment, such as sudden obstacles. To be able to measure both control outputs and perception outputs, it was
necessary to divide the node into two running_mode: `planning_control` and `perception_planning`.

![ReactionAnalyzerDesign.png](media%2FReactionAnalyzerDesign.png)

### Planning Control Mode

In this mode, the reaction analyzer creates a dummy publisher for the PredictedObjects and PointCloud2 topics. In the
beginning of the test, it publishes the initial position of the ego vehicle and the goal position to set the test
environment. Then, it spawns a sudden obstacle in front of the ego vehicle. After the obstacle is spawned, it starts to
search reacted messages of the planning and control nodes in the pre-determined topics. When all the topics are reacted,
it calculates the reaction time of the nodes and statistics by comparing `reacted_times` of each of the nodes
with `spawn_cmd_time`, and it creates a csv file to store the results.

### Perception Planning Mode

In this mode, the reaction analyzer reads the rosbag files which are recorded from AWSIM, and it creates a topic
publisher for each topic inside the rosbag to replay the rosbag. It reads two rosbag files: `path_bag_without_object`
and `path_bag_with_object`. Firstly, it replays the `path_bag_without_object` to set the initial position of the ego
vehicle and the goal position. After `spawn_time_after_init` seconds , it replays the `path_bag_with_object` to spawn a
sudden obstacle in front of the ego vehicle. After the obstacle is spawned, it starts to search the reacted messages of
the perception and planning nodes in the pre-determined topics. When all the topics are reacted, it calculates the
reaction time of the nodes and statistics by comparing `reacted_times` of each of the nodes with `spawn_cmd_time`, and
it creates a csv file to store the results.

#### Point Cloud Publisher Type

To get better analyze for Perception & Sensing pipeline, the reaction analyzer can publish the point cloud messages in 3
different ways: `async_header_sync_publish`, `sync_header_sync_publish` or `async_publish`. (`T` is the period of the
lidar's output)

![PointcloudPublisherType.png](media%2FPointcloudPublisherType.png)

- `async_header_sync_publish`: It publishes the point cloud messages synchronously with asynchronous header times. It
  means that each of the lidar's output will be published at the same time, but the headers of the point cloud messages
  includes different timestamps because of the phase difference.
- `sync_header_sync_publish`: It publishes the point cloud messages synchronously with synchronous header times. It
  means that each of the lidar's output will be published at the same time, and the headers of the point cloud messages
  includes the same timestamps.
- `async_publish`: It publishes the point cloud messages asynchronously. It means that each of the lidar's output will
  be published at different times.

## Usage

The common parameters you need to define for both running modes are `output_file_path`, `test_iteration`,
and `reaction_chain` list. `output_file_path` is the output file path is the path where the results and statistics
will be stored. `test_iteration` defines how many tests will be performed. The `reaction_chain` list is the list of the
pre-defined topics you want to measure their reaction times.

**IMPORTANT:** Ensure the `reaction_chain` list is correctly defined:

- For `perception_planning` mode, **do not** define `Control` nodes.
- For `planning_control` mode, **do not** define `Perception` nodes.

### Prepared Test Environment

- Download the demonstration test map from the
  link [here](https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip). After
  downloading,
  extract the zip file and use its path as `[MAP_PATH]` in the following commands.

#### Planning Control Mode

- You need to define only Planning and Control nodes in the `reaction_chain` list. With the default parameters,
  you can start to test with the following command:

```bash
ros2 launch reaction_analyzer reaction_analyzer.launch.xml running_mode:=planning_control vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit map_path:=[MAP_PATH]
```

After the command, the `simple_planning_simulator` and the `reaction_analyzer` will be launched. It will automatically
start to test. After the test is completed, the results will be stored in the `output_file_path` you defined.

#### Perception Planning Mode

- Download the rosbag files from the Google Drive
  link [here](https://drive.google.com/file/d/1-Qcv7gYfR-usKOjUH8I997w8I4NMhXlX/view?usp=sharing).
- Extract the zip file and set the path of the `.db3` files to parameters `path_bag_without_object`
  and `path_bag_with_object`.
- You can start to test with the following command:

```bash
ros2 launch reaction_analyzer reaction_analyzer.launch.xml running_mode:=perception_planning vehicle_model:=sample_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=[MAP_PATH]
```

After the command, the `e2e_simulator` and the `reaction_analyzer` will be launched. It will automatically start
to test. After the test is completed, the results will be stored in the `output_file_path` you defined.

#### Prepared Test Environment

**Scene without object:**
![sc1-awsim.png](media%2Fsc1-awsim.png)
![sc1-rviz.png](media%2Fsc1-rviz.png)

**Scene object:**
![sc2-awsim.png](media%2Fsc2-awsim.png)
![sc2-rviz.png](media%2Fsc2-rviz.png)

### Custom Test Environment

**If you want to run the reaction analyzer with your custom test environment, you need to redefine some of the
parameters.
The parameters you need to redefine are `initialization_pose`, `entity_params`, `goal_pose`, and `topic_publisher` (
for `perception_planning` mode) parameters.**

- To set `initialization_pose`, `entity_params`, `goal_pose`:
- Run the AWSIM environment. Tutorial for AWSIM can be found
  [here](https://autowarefoundation.github.io/AWSIM/main/GettingStarted/QuickStartDemo/).
- Run the e2e_simulator with the following command:

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=[MAP_PATH]
```

- After EGO is initialized, you can move the ego vehicle to the desired position by using the `SetGoal` button in the
  RViz.
- After the EGO stopped in desired position, please localize the dummy obstacle by using the traffic controller. You can
  control the traffic by pressing `ESC` button.

**After localize EGO and dummy vehicle, we should write the positions of these entities in the map frame
in `reaction_analyzer.param.yaml`. To achieve this:**

- Get initialization pose from `/awsim/ground_truth/vehicle/pose` topic.
- Get entity params from `/perception/object_recognition/objects` topic.
- Get goal pose from `/planning/mission_planning/goal` topic.

**PS: `initialization_pose` is only valid for `planning_control` mode.**

- After the parameters were noted, we should record the rosbags for the test. To record the rosbags, you can use the
  following command:

```bash
ros2 bag record --all
```

- You should record two rosbags: one without the object and one with the object. You can use the traffic controller to
  spawn the object in front of the EGO vehicle or remove it.

**NOTE: You should record the rosbags in the same environment with the same position of the EGO vehicle. You don't need
to run Autoware while recording.**

- After you record the rosbags, you can set the `path_bag_without_object` and `path_bag_with_object` parameters with the
  paths of the recorded rosbags.

## Results

The results will be stored in the `csv` file format and written to the `output_file_path` you defined. It shows each
pipeline of the Autoware by using header timestamp of the messages, and it reports `Node Latency`, `Pipeline Latency`,
and `Total Latency`
for each of the nodes.

- `Node Latency`: The time difference between previous and current node's reaction timestamps. If it is the first node
  in the pipeline, it is same as `Pipeline Latency`.
- `Pipeline Latency`: The time difference between published time of the message and pipeline header time.
- `Total Latency`: The time difference between the message's published timestamp and the spawn obstacle command sent
  timestamp.

## Parameters

| Name                                                                         | Type   | Description                                                                                                                                   |
| ---------------------------------------------------------------------------- | ------ | --------------------------------------------------------------------------------------------------------------------------------------------- |
| `timer_period`                                                               | double | [s] Period for the main processing timer.                                                                                                     |
| `test_iteration`                                                             | int    | Number of iterations for the test.                                                                                                            |
| `output_file_path`                                                           | string | Directory path where test results and statistics will be stored.                                                                              |
| `spawn_time_after_init`                                                      | double | [s] Time delay after initialization before spawning objects. Only valid `perception_planning` mode.                                           |
| `spawn_distance_threshold`                                                   | double | [m] Distance threshold for spawning objects. Only valid `planning_control` mode.                                                              |
| `poses.initialization_pose`                                                  | struct | Initial pose of the vehicle, containing `x`, `y`, `z`, `roll`, `pitch`, and `yaw` fields. Only valid `planning_control` mode.                 |
| `poses.entity_params`                                                        | struct | Parameters for entities (e.g., obstacles), containing `x`, `y`, `z`, `roll`, `pitch`, `yaw`, `x_dimension`, `y_dimension`, and `z_dimension`. |
| `poses.goal_pose`                                                            | struct | Goal pose of the vehicle, containing `x`, `y`, `z`, `roll`, `pitch`, and `yaw` fields.                                                        |
| `topic_publisher.path_bag_without_object`                                    | string | Path to the ROS bag file without objects. Only valid `perception_planning` mode.                                                              |
| `topic_publisher.path_bag_with_object`                                       | string | Path to the ROS bag file with objects. Only valid `perception_planning` mode.                                                                 |
| `topic_publisher.spawned_pointcloud_sampling_distance`                       | double | [m] Sampling distance for point clouds of spawned objects. Only valid `planning_control` mode.                                                |
| `topic_publisher.dummy_perception_publisher_period`                          | double | [s] Publishing period for the dummy perception data. Only valid `planning_control` mode.                                                      |
| `topic_publisher.pointcloud_publisher.pointcloud_publisher_type`             | string | Defines how the PointCloud2 messages are going to be published. Modes explained above.                                                        |
| `topic_publisher.pointcloud_publisher.pointcloud_publisher_period`           | double | [s] Publishing period of the PointCloud2 messages.                                                                                            |
| `topic_publisher.pointcloud_publisher.publish_only_pointcloud_with_object`   | bool   | Default false. Publish only the point cloud messages with the object.                                                                         |
| `reaction_params.first_brake_params.debug_control_commands`                  | bool   | Debug publish flag.                                                                                                                           |
| `reaction_params.first_brake_params.control_cmd_buffer_time_interval`        | double | [s] Time interval for buffering control commands.                                                                                             |
| `reaction_params.first_brake_params.min_number_descending_order_control_cmd` | int    | Minimum number of control commands in descending order for triggering brake.                                                                  |
| `reaction_params.first_brake_params.min_jerk_for_brake_cmd`                  | double | [m/s³] Minimum jerk value for issuing a brake command.                                                                                        |
| `reaction_params.search_zero_vel_params.max_looking_distance`                | double | [m] Maximum looking distance for zero velocity on trajectory                                                                                  |
| `reaction_params.search_entity_params.search_radius`                         | double | [m] Searching radius for spawned entity. Distance between ego pose and entity pose.                                                           |
| `reaction_chain`                                                             | struct | List of the nodes with their topics and topic's message types.                                                                                |

## Limitations

- Reaction analyzer has some limitation like `PublisherMessageType`, `SubscriberMessageType` and `ReactionType`. It is
  currently supporting following list:

- **Publisher Message Types:**

  - `sensor_msgs/msg/PointCloud2`
  - `sensor_msgs/msg/CameraInfo`
  - `sensor_msgs/msg/Image`
  - `geometry_msgs/msg/PoseWithCovarianceStamped`
  - `sensor_msgs/msg/Imu`
  - `autoware_vehicle_msgs/msg/ControlModeReport`
  - `autoware_vehicle_msgs/msg/GearReport`
  - `autoware_vehicle_msgs/msg/HazardLightsReport`
  - `autoware_vehicle_msgs/msg/SteeringReport`
  - `autoware_vehicle_msgs/msg/TurnIndicatorsReport`
  - `autoware_vehicle_msgs/msg/VelocityReport`

- **Subscriber Message Types:**

  - `sensor_msgs/msg/PointCloud2`
  - `autoware_perception_msgs/msg/DetectedObjects`
  - `autoware_perception_msgs/msg/TrackedObjects`
  - `autoware_perception_msgs/msg/PredictedObject`
  - `autoware_planning_msgs/msg/Trajectory`
  - `autoware_control_msgs/msg/Control`

- **Reaction Types:**
  - `FIRST_BRAKE`
  - `SEARCH_ZERO_VEL`
  - `SEARCH_ENTITY`

## Future improvements

- The reaction analyzer can be improved by adding more reaction types. Currently, it is supporting only `FIRST_BRAKE`,
  `SEARCH_ZERO_VEL`, and `SEARCH_ENTITY` reaction types. It can be extended by adding more reaction types for each of
  the message types.
