# This is from ros2 common_interfaces.
# https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/MultiArrayDimension.msg

# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
