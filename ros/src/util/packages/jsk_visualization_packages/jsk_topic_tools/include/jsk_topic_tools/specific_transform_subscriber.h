#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

ros::ServiceClient dynamic_tf_publisher_client;

void CallSetDynamicTf(std::string parent_frame_id, std::string frame_id, geometry_msgs::Transform transform);

void transformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);


