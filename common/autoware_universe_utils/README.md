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

- `void comment(const std::string & comment);`
  - Adds a comment to the current function being tracked.
  - `comment`: Comment to be added.

##### Note

- It's possible to start and end time measurements using `start_track` and `end_track` as shown below:

  ```cpp
  time_keeper.start_track("example_function");
  // Your function code here
  time_keeper.end_track("example_function");
  ```

- For safety and to ensure proper tracking, it is recommended to use `ScopedTimeTrack`.

##### Example

```cpp
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

class ExampleNode : public rclcpp::Node
{
public:
  ExampleNode() : Node("time_keeper_example")
  {
    publisher_ =
      create_publisher<autoware::universe_utils::ProcessingTimeDetail>("processing_time", 1);

    time_keeper_ = std::make_shared<autoware::universe_utils::TimeKeeper>(publisher_, &std::cerr);
    // You can also add a reporter later by add_reporter.
    // time_keeper_->add_reporter(publisher_);
    // time_keeper_->add_reporter(&std::cerr);

    timer_ =
      create_wall_timer(std::chrono::seconds(1), std::bind(&ExampleNode::func_a, this));
  }

private:
  std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_;
  rclcpp::Publisher<autoware::universe_utils::ProcessingTimeDetail>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_str_;
  rclcpp::TimerBase::SharedPtr timer_;

  void func_a()
  {
    // Start constructing ProcessingTimeTree (because func_a is the root function)
    autoware::universe_utils::ScopedTimeTrack st("func_a", *time_keeper_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    time_keeper_->comment("This is a comment for func_a");
    func_b();
    // End constructing ProcessingTimeTree. After this, the tree will be reported (publishing
    // message and outputting to std::cerr)
  }

  void func_b()
  {
    autoware::universe_utils::ScopedTimeTrack st("func_b", *time_keeper_);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    time_keeper_->comment("This is a comment for func_b");
    func_c();
  }

  void func_c()
  {
    autoware::universe_utils::ScopedTimeTrack st("func_c", *time_keeper_);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    time_keeper_->comment("This is a comment for func_c");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

- Output (console)

  ```text
  ==========================
  func_a (6.243ms) : This is a comment for func_a
      └── func_b (5.116ms) : This is a comment for func_b
          └── func_c (3.055ms) : This is a comment for func_c
  ```

- Output (`ros2 topic echo /processing_time`)

  ```text
  ---
  nodes:
  - id: 1
    name: func_a
    processing_time: 6.366
    parent_id: 0
    comment: This is a comment for func_a
  - id: 2
    name: func_b
    processing_time: 5.237
    parent_id: 1
    comment: This is a comment for func_b
  - id: 3
    name: func_c
    processing_time: 3.156
    parent_id: 2
    comment: This is a comment for func_c
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
