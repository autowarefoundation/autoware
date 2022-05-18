# component_interface_utils

## Features

This is a utility package that provides the following features:

- Logging for service and client

## Usage

1. This package requires interface information in this format.

   ```cpp
   struct SampleService
   {
     using Service = sample_msgs::srv::ServiceType;
     static constexpr char name[] = "/sample/service";
   };
   ```

2. Create a wrapper using the above definition as follows.

   ```cpp
   component_interface_utils::Service<SampleService>::SharedPtr srv_;
   srv_ = component_interface_utils::create_service<SampleService>(node, ...);
   ```

## Design

This package provides the wrappers for the interface classes of rclcpp.
The wrappers limit the usage of the original class to enforce the processing recommended by the component interface.
Do not inherit the class of rclcpp, and forward or wrap the member function that is allowed to be used.
