# autoware_simple_object_merger

This package can merge multiple topics of [autoware_perception_msgs/msg/DetectedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/DetectedObject.msg) with low calculation cost.

## Design

### Background

[Object_merger](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/object_merger) is mainly used for merge process with DetectedObjects. There are 2 characteristics in `Object_merger`. First, `object_merger` solve data association algorithm like Hungarian algorithm for matching problem, but it needs computational cost. Second, `object_merger` can handle only 2 DetectedObjects topics and cannot handle more than 2 topics in one node. To merge 6 DetectedObjects topics, 6 `object_merger` nodes need to stand for now.

Therefore, `autoware_simple_object_merger` aim to merge multiple DetectedObjects with low calculation cost.
The package do not use data association algorithm to reduce the computational cost, and it can handle more than 2 topics in one node to prevent launching a large number of nodes.

### Use case

- Multiple radar detection

`autoware_simple_object_merger` can be used for multiple radar detection. By combining them into one topic from multiple radar topics, the pipeline for faraway detection with radar can be simpler.

### Limitation

- Sensor data drops and delay

Merged objects will not be published until all topic data is received when initializing. In addition, to care sensor data drops and delayed, this package has a parameter to judge timeout. When the latest time of the data of a topic is older than the timeout parameter, it is not merged for output objects. For now specification of this package, if all topic data is received at first and after that the data drops, and the merged objects are published without objects which is judged as timeout.The timeout parameter should be determined by sensor cycle time.

- Post-processing

Because this package does not have matching processing, there are overlapping objects depending on the input objects. So output objects can be used only when post-processing is used. For now, [clustering processing](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/autoware_radar_object_clustering) can be used as post-processing.

## Interface

### Input

Input topics is defined by the parameter of `input_topics` (List[string]). The type of input topics is `std::vector<autoware_perception_msgs/msg/DetectedObjects.msg>`.

### Output

- `~/output/objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - Merged objects combined from input topics.

### Parameters

- `update_rate_hz` (double) [hz]
  - Default parameter: 20.0

This parameter is update rate for the `onTimer` function.
This parameter should be same as the frame rate of input topics.

- `new_frame_id` (string)
  - Default parameter: "base_link"

This parameter is the header frame_id of the output topic.
If output topics use for perception module, it should be set for "base_link"

- `timeout_threshold` (double) [s]
  - Default parameter: 0.1

This parameter is the threshold for timeout judgement.
If the time difference between the first topic of `input_topics` and an input topic is exceeded to this parameter, then the objects of topic is not merged to output objects.

```cpp
  for (size_t i = 0; i < input_topic_size; i++) {
    double time_diff = rclcpp::Time(objects_data_.at(i)->header.stamp).seconds() -
                       rclcpp::Time(objects_data_.at(0)->header.stamp).seconds();
    if (std::abs(time_diff) < node_param_.timeout_threshold) {
      // merge objects
    }
  }
```

- `input_topics` (List[string])
  - Default parameter: "[]"

This parameter is the name of input topics.
For example, when this packages use for radar objects,

```yaml
input_topics:
  [
    "/sensing/radar/front_center/detected_objects",
    "/sensing/radar/front_left/detected_objects",
    "/sensing/radar/rear_left/detected_objects",
    "/sensing/radar/rear_center/detected_objects",
    "/sensing/radar/rear_right/detected_objects",
    "/sensing/radar/front_right/detected_objects",
  ]
```

can be set in config yaml file.
For now, the time difference is calculated by the header time between the first topic of `input_topics` and the input topics, so the most important objects to detect should be set in the first of `input_topics` list.
