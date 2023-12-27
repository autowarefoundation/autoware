# autoware_auto_msgs_adapter

This package is used to convert `autoware_msgs` to `autoware_auto_msgs`.

## Purpose

As we transition from `autoware_auto_msgs` to `autoware_msgs`, we wanted to provide flexibility and compatibility for
users who are still using `autoware_auto_msgs`.

This adapter package allows users to easily convert messages between the two formats.

## Capabilities

The `autoware_auto_msgs_adapter` package provides the following capabilities:

- Conversion of supported `autoware_msgs` messages to `autoware_auto_msgs` messages.
- Can be extended to support conversion for any message type pairs.
- Each instance is designed to convert from a single source message type to a single target message type.
- Multiple instances can be launched to convert multiple message types.
- Can be launched as a standalone node or as a component.

## Usage

Customize the adapter configuration by replicating and editing the `autoware_auto_msgs_adapter_control.param.yaml` file located
in the `autoware_auto_msgs_adapter/config` directory. Example configuration:

```yaml
/**:
  ros__parameters:
    msg_type_target: "autoware_auto_control_msgs/msg/AckermannControlCommand"
    topic_name_source: "/control/command/control_cmd"
    topic_name_target: "/control/command/control_cmd_auto"
```

Set the `msg_type_target` parameter to the desired target message type from `autoware_auto_msgs`.

Make sure that the `msg_type_target` has the correspondence in either:

- [schema/autoware_auto_msgs_adapter.schema.json](schema/autoware_auto_msgs_adapter.schema.json)
- OR [src/autoware_auto_msgs_adapter_core.cpp](src/autoware_auto_msgs_adapter_core.cpp) `AutowareAutoMsgsAdapterNode::create_adapter_map()` method.

(If this package is maintained correctly, they should match each other.)

Launch the adapter node by any of the following methods:

### `ros2 launch`

```bash
ros2 launch autoware_auto_msgs_adapter autoware_auto_msgs_adapter.launch.xml param_path:='full_path_to_param_file'
```

Make sure to set the `param_path` argument to the full path of the parameter file.

Alternatively,

- You can replicate and edit the launch file to suit to your needs.
- You can make use of the existing launch file in another launch file by providing the parameter file path as an
  argument.

### `ros2 run`

```bash
ros2 run autoware_auto_msgs_adapter autoware_auto_msgs_adapter_exe --ros-args --params-file 'full_path_to_param_file'
```

Make sure to set the `param_path` argument to the full path of the parameter file.

## Contributing

### Current implementation details

The entry point for the adapter executable is created with `RCLCPP_COMPONENTS_REGISTER_NODE` the [autoware_auto_msgs_adapter_core.cpp](src/Fautoware_auto_msgs_adapter_core.cpp).

This allows it to be launched as a component or as a standalone node.

In the `AutowareAutoMsgsAdapterNode` constructor, the adapter is selected by the type string provided in the
configuration file. The adapter is then initialized with the topic names provided.

The constructors of the adapters are responsible for creating the publisher and subscriber (which makes use of the conversion method).

### Adding a new message pair

To add a new message pair,

- Replicate and edit:
  - [adapter_control.hpp](include/autoware_auto_msgs_adapter/adapter_control.hpp).
  - Add the new header file to the [CMakeLists.txt](CMakeLists.txt).
- Add a new entry to the returned map instance in the `AutowareAutoMsgsAdapterNode::create_adapter_map()` method of the adapter node:
  - [autoware_auto_msgs_adapter_core.cpp](src/autoware_auto_msgs_adapter_core.cpp)
- Add a new entry to the [schema/autoware_auto_msgs_adapter.schema.json](schema/autoware_auto_msgs_adapter.schema.json) file in the `definitions:autoware_auto_msgs_adapter:properties:msg_type_target:enum` section.
  - Learn more about JSON schema usage in [here](https://autowarefoundation.github.io/autoware-documentation/main/contributing/coding-guidelines/ros-nodes/parameters/#json-schema).
- Create a new config file by replicating and editing:
  - [autoware_auto_msgs_adapter_control.param.yaml](config/autoware_auto_msgs_adapter_control.param.yaml)
- Add a new test file by replicating and editing:
  - [test_msg_ackermann_control_command.cpp](test/test_msg_ackermann_control_command.cpp)
  - No need to edit the `CMakeLists.txt` file as it will automatically detect the new test file.

Also make sure to test the new adapter with:

```bash
colcon test --event-handlers console_cohesion+ --packages-select autoware_auto_msgs_adapter
```
