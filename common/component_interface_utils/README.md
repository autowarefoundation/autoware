# component_interface_utils

## Features

This is a utility package that provides the following features:

- Instantiation of the wrapper class
- Logging for service and client
- Service exception for response
- Relays for topic and service

## Design

This package provides the wrappers for the interface classes of rclcpp.
The wrappers limit the usage of the original class to enforce the processing recommended by the component interface.
Do not inherit the class of rclcpp, and forward or wrap the member function that is allowed to be used.

## Instantiation of the wrapper class

The wrapper class requires interface information in this format.

```cpp
struct SampleService
{
  using Service = sample_msgs::srv::ServiceType;
  static constexpr char name[] = "/sample/service";
};

struct SampleMessage
{
  using Message = sample_msgs::msg::MessageType;
  static constexpr char name[] = "/sample/message";
  static constexpr size_t depth = 3;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};
```

Create the wrapper using the above definition as follows.

```cpp
// header file
component_interface_utils::Service<SampleService>::SharedPtr srv_;
component_interface_utils::Client<SampleService>::SharedPtr cli_;
component_interface_utils::Publisher<SampleMessage>::SharedPtr pub_;
component_interface_utils::Subscription<SampleMessage>::SharedPtr sub_;

// source file
const auto node = component_interface_utils::NodeAdaptor(this);
node.init_srv(srv_, callback);
node.init_cli(cli_);
node.init_pub(pub_);
node.init_sub(sub_, callback);
```

## Logging for service and client

If the wrapper class is used, logging is automatically enabled. The log level is `RCLCPP_INFO`.

## Service exception for response

If the wrapper class is used and the service response has status, throwing `ServiceException` will automatically catch and set it to status.
This is useful when returning an error from a function called from the service callback.

```cpp
void service_callback(Request req, Response res)
{
   function();
   res->status.success = true;
}

void function()
{
   throw ServiceException(ERROR_CODE, "message");
}
```

If the wrapper class is not used or the service response has no status, manually catch the `ServiceException` as follows.

```cpp
void service_callback(Request req, Response res)
{
   try {
      function();
      res->status.success = true;
   } catch (const ServiceException & error) {
      res->status = error.status();
   }
}
```

## Relays for topic and service

There are utilities for relaying services and messages of the same type.

```cpp
const auto node = component_interface_utils::NodeAdaptor(this);
service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
node.relay_message(pub_, sub_);
node.relay_service(cli_, srv_, service_callback_group_);  // group is for avoiding deadlocks
```
