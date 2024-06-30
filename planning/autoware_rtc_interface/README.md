# RTC Interface

## Purpose

RTC Interface is an interface to publish the decision status of behavior planning modules and receive execution command from external of an autonomous driving system.

## Inner-workings / Algorithms

### Usage example

```c++
// Generate instance (in this example, "intersection" is selected)
autoware::rtc_interface::RTCInterface rtc_interface(node, "intersection");

// Generate UUID
const unique_identifier_msgs::msg::UUID uuid = generateUUID(getModuleId());

// Repeat while module is running
while (...) {
  // Get safety status of the module corresponding to the module id
  const bool safe = ...

  // Get distance to the object corresponding to the module id
  const double start_distance = ...
  const double finish_distance = ...

  // Get time stamp
  const rclcpp::Time stamp = ...

  // Update status
  rtc_interface.updateCooperateStatus(uuid, safe, start_distance, finish_distance, stamp);

  if (rtc_interface.isActivated(uuid)) {
    // Execute planning
  } else {
    // Stop planning
  }
  // Get time stamp
  const rclcpp::Time stamp = ...

  // Publish status topic
  rtc_interface.publishCooperateStatus(stamp);
}

// Remove the status from array
rtc_interface.removeCooperateStatus(uuid);
```

## Inputs / Outputs

### RTCInterface (Constructor)

```c++
autoware::rtc_interface::RTCInterface(rclcpp::Node & node, const std::string & name);
```

#### Description

A constructor for `autoware::rtc_interface::RTCInterface`.

#### Input

- `node` : Node calling this interface
- `name` : Name of cooperate status array topic and cooperate commands service
  - Cooperate status array topic name : `~/{name}/cooperate_status`
  - Cooperate commands service name : `~/{name}/cooperate_commands`

#### Output

An instance of `RTCInterface`

### publishCooperateStatus

```c++
autoware::rtc_interface::publishCooperateStatus(const rclcpp::Time & stamp)
```

#### Description

Publish registered cooperate status.

#### Input

- `stamp` : Time stamp

#### Output

Nothing

### updateCooperateStatus

```c++
autoware::rtc_interface::updateCooperateStatus(const unique_identifier_msgs::msg::UUID & uuid, const bool safe, const double start_distance, const double finish_distance, const rclcpp::Time & stamp)
```

#### Description

Update cooperate status corresponding to `uuid`.
If cooperate status corresponding to `uuid` is not registered yet, add new cooperate status.

#### Input

- `uuid` : UUID for requesting module
- `safe` : Safety status of requesting module
- `start_distance` : Distance to the start object from ego vehicle
- `finish_distance` : Distance to the finish object from ego vehicle
- `stamp` : Time stamp

#### Output

Nothing

### removeCooperateStatus

```c++
autoware::rtc_interface::removeCooperateStatus(const unique_identifier_msgs::msg::UUID & uuid)
```

#### Description

Remove cooperate status corresponding to `uuid` from registered statuses.

#### Input

- `uuid` : UUID for expired module

#### Output

Nothing

### clearCooperateStatus

```c++
autoware::rtc_interface::clearCooperateStatus()
```

#### Description

Remove all cooperate statuses.

#### Input

Nothing

#### Output

Nothing

### isActivated

```c++
autoware::rtc_interface::isActivated(const unique_identifier_msgs::msg::UUID & uuid)
```

#### Description

Return received command status corresponding to `uuid`.

#### Input

- `uuid` : UUID for checking module

#### Output

If auto mode is enabled, return based on the safety status.
If not, if received command is `ACTIVATED`, return `true`.
If not, return `false`.

### isRegistered

```c++
autoware::rtc_interface::isRegistered(const unique_identifier_msgs::msg::UUID & uuid)
```

#### Description

Return `true` if `uuid` is registered.

#### Input

- `uuid` : UUID for checking module

#### Output

If `uuid` is registered, return `true`.
If not, return `false`.

## Debugging Tools

There is a debugging tool called [RTC Replayer](https://autowarefoundation.github.io/autoware_tools/main/planning/autoware_rtc_replayer/) for the RTC interface.

## Assumptions / Known limits

## Future extensions / Unimplemented parts
