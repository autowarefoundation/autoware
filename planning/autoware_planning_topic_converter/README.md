# Planning Topic Converter

## Purpose

This package provides tools that convert topic type among types are defined in <https://github.com/autowarefoundation/autoware_msgs>.

## Inner-workings / Algorithms

### Usage example

The tools in this package are provided as composable ROS 2 component nodes, so that they can be spawned into an existing process, launched from launch files, or invoked from the command line.

```xml
<load_composable_node target="container_name">
  <composable_node pkg="planning_topic_converter" plugin="autoware::planning_topic_converter::PathToTrajectory" name="path_to_trajectory_converter" namespace="">
  <!-- params -->
  <param name="input_topic" value="foo"/>
  <param name="output_topic" value="bar"/>
  <!-- composable node config -->
  <extra_arg name="use_intra_process_comms" value="false"/>
  </composable_node>
</load_composable_node>
```

## Parameters

| Name           | Type   | Description        |
| :------------- | :----- | :----------------- |
| `input_topic`  | string | input topic name.  |
| `output_topic` | string | output topic name. |

## Assumptions / Known limits

## Future extensions / Unimplemented parts
