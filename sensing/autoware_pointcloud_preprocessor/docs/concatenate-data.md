# concatenate_data

## Purpose

Many self-driving cars combine multiple LiDARs to expand the sensing range. Therefore, a function to combine a plurality of point clouds is required.

To combine multiple sensor data with a similar timestamp, the [message_filters](https://github.com/ros2/message_filters) is often used in the ROS-based system, but this requires the assumption that all inputs can be received. Since safety must be strongly considered in autonomous driving, the point clouds concatenate node must be designed so that even if one sensor fails, the remaining sensor information can be output.

## Inner-workings / Algorithms

The figure below represents the reception time of each sensor data and how it is combined in the case.

![concatenate_data_timing_chart](./image/concatenate_data.drawio.svg)

## Inputs / Outputs

### Input

| Name            | Type                                             | Description                                                                   |
| --------------- | ------------------------------------------------ | ----------------------------------------------------------------------------- |
| `~/input/twist` | `geometry_msgs::msg::TwistWithCovarianceStamped` | The vehicle odometry is used to interpolate the timestamp of each sensor data |

### Output

| Name              | Type                            | Description               |
| ----------------- | ------------------------------- | ------------------------- |
| `~/output/points` | `sensor_msgs::msg::Pointcloud2` | concatenated point clouds |

## Parameters

| Name                 | Type             | Default Value | Description                                                         |
| -------------------- | ---------------- | ------------- | ------------------------------------------------------------------- |
| `input/points`       | vector of string | []            | input topic names that type must be `sensor_msgs::msg::Pointcloud2` |
| `input_frame`        | string           | ""            | input frame id                                                      |
| `output_frame`       | string           | ""            | output frame id                                                     |
| `has_static_tf_only` | bool             | false         | flag to listen TF only once                                         |
| `max_queue_size`     | int              | 5             | max queue size of input/output topics                               |

### Core Parameters

| Name                              | Type             | Default Value | Description                                                                                                                                                                                                                                                |
| --------------------------------- | ---------------- | ------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `timeout_sec`                     | double           | 0.1           | tolerance of time to publish next pointcloud [s]<br>When this time limit is exceeded, the filter concatenates and publishes pointcloud, even if not all the point clouds are subscribed.                                                                   |
| `input_offset`                    | vector of double | []            | This parameter can control waiting time for each input sensor pointcloud [s]. You must to set the same length of offsets with input pointclouds numbers. <br> For its tuning, please see [actual usage page](#how-to-tuning-timeout_sec-and-input_offset). |
| `publish_synchronized_pointcloud` | bool             | false         | If true, publish the time synchronized pointclouds. All input pointclouds are transformed and then re-published as message named `<original_msg_name>_synchronized`.                                                                                       |
| `input_twist_topic_type`          | std::string      | twist         | Topic type for twist. Currently support `twist` or `odom`.                                                                                                                                                                                                 |

## Actual Usage

For the example of actual usage of this node, please refer to the [preprocessor.launch.py](../launch/preprocessor.launch.py) file.

### How to tuning timeout_sec and input_offset

The values in `timeout_sec` and `input_offset` are used in the timer_callback to control concatenation timings.

- Assumptions
  - when the timer runs out, we concatenate the pointclouds in the buffer
  - when the first pointcloud comes to buffer, we reset the timer to `timeout_sec`
  - when the second and later pointclouds comes to buffer, we reset the timer to `timeout_sec` - `input_offset`
  - we assume all lidar has same frequency

| Name           | Description                                          | How to tune                                                                                                                                                          |
| -------------- | ---------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `timeout_sec`  | timeout sec for default timer                        | To avoid mis-concatenation, at least this value must be shorter than sampling time.                                                                                  |
| `input_offset` | timeout extension when a pointcloud comes to buffer. | The amount of waiting time will be `timeout_sec` - `input_offset`. So, you will need to set larger value for the last-coming pointcloud and smaller for fore-coming. |

### Node separation options for future

Since the pointcloud concatenation has two process, "time synchronization" and "pointcloud concatenation", it is possible to separate these processes.

In the future, Nodes will be completely separated in order to achieve node loosely coupled nature, but currently both nodes can be selected for backward compatibility ([See this PR](https://github.com/autowarefoundation/autoware.universe/pull/3312)).

## Assumptions / Known limits

It is necessary to assume that the vehicle odometry value exists, the sensor data and odometry timestamp are correct, and the TF from `base_link` to `sensor_frame` is also correct.
