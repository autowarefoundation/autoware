#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mode_publisher") ;
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string frame_id;
  private_nh.getParam("frame_id", frame_id);
  ROS_INFO_STREAM("frame_id : " << frame_id);

  std::string topic_name;
  private_nh.getParam("topic_name", topic_name);
  ROS_INFO_STREAM("topic_name : " << topic_name);

  std::string model_path;
  private_nh.getParam("model_path", model_path);
    ROS_INFO_STREAM("model_path : " << model_path);

  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(topic_name,10, true);

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = topic_name;
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = model_path;
  marker.mesh_use_embedded_materials = true;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  double roll = 90 * (M_PI /180);
  double yaw = 180 * (M_PI / 180);
  double pitch = 0;
  marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.frame_locked = true;

  pub.publish(marker);

  ros::spin();

  return 0;
}
