# autoware_universe_utils

## Purpose

This package contains many common functions used by other packages, so please refer to them as needed.

## For developers

`autoware_universe_utils.hpp` header file was removed because the source files that directly/indirectly include this file took a long time for preprocessing.

## `autoware::universe_utils`

### `systems`

#### `autoware::universe_utils::TimeKeeper`

##### Constructor

```cpp
template <typename... Reporters>
explicit TimeKeeper(Reporters... reporters);
```

- Initializes the `TimeKeeper` with a list of reporters.

##### Methods

- `void add_reporter(std::ostream * os);`

  - Adds a reporter to output processing times to an `ostream`.
  - `os`: Pointer to the `ostream` object.

- `void add_reporter(rclcpp::Publisher<ProcessingTimeDetail>::SharedPtr publisher);`

  - Adds a reporter to publish processing times to an `rclcpp` publisher.
  - `publisher`: Shared pointer to the `rclcpp` publisher.

- `void add_reporter(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher);`

  - Adds a reporter to publish processing times to an `rclcpp` publisher with `std_msgs::msg::String`.
  - `publisher`: Shared pointer to the `rclcpp` publisher.

- `void start_track(const std::string & func_name);`

  - Starts tracking the processing time of a function.
  - `func_name`: Name of the function to be tracked.

- `void end_track(const std::string & func_name);`
  - Ends tracking the processing time of a function.
  - `func_name`: Name of the function to end tracking.

##### Example

```cpp
#include "autoware/universe_utils/system/time_keeper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("time_keeper_example");

  auto time_keeper = std::make_shared<autoware::universe_utils::TimeKeeper>();

  time_keeper->add_reporter(&std::cout);

  auto publisher =
    node->create_publisher<autoware::universe_utils::ProcessingTimeDetail>("processing_time", 10);

  time_keeper->add_reporter(publisher);

  auto publisher_str = node->create_publisher<std_msgs::msg::String>("processing_time_str", 10);

  time_keeper->add_reporter(publisher_str);

  auto funcA = [&time_keeper]() {
    time_keeper->start_track("funcA");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    time_keeper->end_track("funcA");
  };

  auto funcB = [&time_keeper, &funcA]() {
    time_keeper->start_track("funcB");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    funcA();
    time_keeper->end_track("funcB");
  };

  auto funcC = [&time_keeper, &funcB]() {
    time_keeper->start_track("funcC");
    std::this_thread::sleep_for(std::chrono::seconds(3));
    funcB();
    time_keeper->end_track("funcC");
  };

  funcC();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

- Output (console)

  ```text
  ==========================
  funcC (6000.7ms)
      └── funcB (3000.44ms)
          └── funcA (1000.19ms)
  ```

- Output (`ros2 topic echo /processing_time`)

  ```text
  nodes:
  - id: 1
    name: funcC
    processing_time: 6000.659
    parent_id: 0
  - id: 2
    name: funcB
    processing_time: 3000.415
    parent_id: 1
  - id: 3
    name: funcA
    processing_time: 1000.181
    parent_id: 2
  ---
  ```

- Output (`ros2 topic echo /processing_time_str --field data`)

  ```text
  funcC (6000.67ms)
    └── funcB (3000.42ms)
        └── funcA (1000.19ms)

  ---
  ```

#### `autoware::universe_utils::ScopedTimeTrack`

##### Description

Class for automatically tracking the processing time of a function within a scope.

##### Constructor

```cpp
ScopedTimeTrack(const std::string & func_name, TimeKeeper & time_keeper);
```

- `func_name`: Name of the function to be tracked.
- `time_keeper`: Reference to the `TimeKeeper` object.

##### Destructor

```cpp
~ScopedTimeTrack();
```

- Destroys the `ScopedTimeTrack` object, ending the tracking of the function.

##### Example

```cpp
#include "autoware/universe_utils/system/time_keeper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("scoped_time_track_example");

  auto time_keeper = std::make_shared<autoware::universe_utils::TimeKeeper>();

  time_keeper->add_reporter(&std::cout);

  auto publisher =
    node->create_publisher<autoware::universe_utils::ProcessingTimeDetail>("processing_time", 10);

  time_keeper->add_reporter(publisher);

  auto publisher_str = node->create_publisher<std_msgs::msg::String>("processing_time_str", 10);

  time_keeper->add_reporter(publisher_str);

  auto funcA = [&time_keeper]() {
    autoware::universe_utils::ScopedTimeTrack st("funcA", *time_keeper);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  };

  auto funcB = [&time_keeper, &funcA]() {
    autoware::universe_utils::ScopedTimeTrack st("funcB", *time_keeper);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    funcA();
  };

  auto funcC = [&time_keeper, &funcB]() {
    autoware::universe_utils::ScopedTimeTrack st("funcC", *time_keeper);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    funcB();
  };

  funcC();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

- Output (console)

  ```text
  ==========================
  funcC (6000.7ms)
      └── funcB (3000.44ms)
          └── funcA (1000.19ms)
  ```

- Output (`ros2 topic echo /processing_time`)

  ```text
  nodes:
  - id: 1
    name: funcC
    processing_time: 6000.659
    parent_id: 0
  - id: 2
    name: funcB
    processing_time: 3000.415
    parent_id: 1
  - id: 3
    name: funcA
    processing_time: 1000.181
    parent_id: 2
  ---
  ```

- Output (`ros2 topic echo /processing_time_str --field data`)

  ```text
  funcC (6000.67ms)
    └── funcB (3000.42ms)
        └── funcA (1000.19ms)

  ---
  ```
