# vehicle utils

Vehicle utils provides a convenient library used to check vehicle status.

## Feature

The library contains following classes.

### vehicle_stop_checker

This class check whether the vehicle is stopped or not based on localization result.

#### Subscribed Topics

| Name                            | Type                      | Description      |
| ------------------------------- | ------------------------- | ---------------- |
| `/localization/kinematic_state` | `nav_msgs::msg::Odometry` | vehicle odometry |

#### Parameters

| Name                       | Type   | Default Value | Explanation                 |
| -------------------------- | ------ | ------------- | --------------------------- |
| `velocity_buffer_time_sec` | double | 10.0          | odometry buffering time [s] |

#### Member functions

```c++
bool isVehicleStopped(const double stop_duration)
```

- Check simply whether the vehicle is stopped based on the localization result.
- Returns `true` if the vehicle is stopped, even if system outputs a non-zero target velocity.

#### Example Usage

Necessary includes:

```c++
#include <autoware/universe_utils/vehicle/vehicle_state_checker.hpp>
```

1.Create a checker instance.

```c++
class SampleNode : public rclcpp::Node
{
public:
  SampleNode() : Node("sample_node")
  {
    vehicle_stop_checker_ = std::make_unique<VehicleStopChecker>(this);
  }

  std::unique_ptr<VehicleStopChecker> vehicle_stop_checker_;

  bool sampleFunc();

  ...
}
```

2.Check the vehicle state.

```c++

bool SampleNode::sampleFunc()
{
  ...

  const auto result_1 = vehicle_stop_checker_->isVehicleStopped();

  ...

  const auto result_2 = vehicle_stop_checker_->isVehicleStopped(3.0);

  ...
}

```

### vehicle_arrival_checker

This class check whether the vehicle arrive at stop point based on localization and planning result.

#### Subscribed Topics

| Name                                     | Type                                      | Description      |
| ---------------------------------------- | ----------------------------------------- | ---------------- |
| `/localization/kinematic_state`          | `nav_msgs::msg::Odometry`                 | vehicle odometry |
| `/planning/scenario_planning/trajectory` | `autoware_planning_msgs::msg::Trajectory` | trajectory       |

#### Parameters

| Name                       | Type   | Default Value | Explanation                                                            |
| -------------------------- | ------ | ------------- | ---------------------------------------------------------------------- |
| `velocity_buffer_time_sec` | double | 10.0          | odometry buffering time [s]                                            |
| `th_arrived_distance_m`    | double | 1.0           | threshold distance to check if vehicle has arrived at target point [m] |

#### Member functions

```c++
bool isVehicleStopped(const double stop_duration)
```

- Check simply whether the vehicle is stopped based on the localization result.
- Returns `true` if the vehicle is stopped, even if system outputs a non-zero target velocity.

```c++
bool isVehicleStoppedAtStopPoint(const double stop_duration)
```

- Check whether the vehicle is stopped at stop point based on the localization and planning result.
- Returns `true` if the vehicle is not only stopped but also arrived at stop point.

#### Example Usage

Necessary includes:

```c++
#include <autoware/universe_utils/vehicle/vehicle_state_checker.hpp>
```

1.Create a checker instance.

```c++
class SampleNode : public rclcpp::Node
{
public:
  SampleNode() : Node("sample_node")
  {
    vehicle_arrival_checker_ = std::make_unique<VehicleArrivalChecker>(this);
  }

  std::unique_ptr<VehicleArrivalChecker> vehicle_arrival_checker_;

  bool sampleFunc();

  ...
}
```

2.Check the vehicle state.

```c++

bool SampleNode::sampleFunc()
{
  ...

  const auto result_1 = vehicle_arrival_checker_->isVehicleStopped();

  ...

  const auto result_2 = vehicle_arrival_checker_->isVehicleStopped(3.0);

  ...

  const auto result_3 = vehicle_arrival_checker_->isVehicleStoppedAtStopPoint();

  ...

  const auto result_4 = vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(3.0);

  ...
}
```

## Assumptions / Known limits

`vehicle_stop_checker` and `vehicle_arrival_checker` cannot check whether the vehicle is stopped more than `velocity_buffer_time_sec` second.
