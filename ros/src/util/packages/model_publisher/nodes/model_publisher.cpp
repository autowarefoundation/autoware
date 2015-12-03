#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mode_publisher") ;
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string base_frame;
  private_nh.getParam("base_frame", base_frame);
  ROS_INFO_STREAM("base_frame : " << base_frame);

  std::string topic_name;
  private_nh.getParam("topic_name", topic_name);
  ROS_INFO_STREAM("topic_name : " << topic_name);

  std::string model_path;
  private_nh.getParam("model_path", model_path);
  ROS_INFO_STREAM("model_path : " << model_path);

  double offset_x;
  private_nh.getParam("offset_x", offset_x);
  ROS_INFO_STREAM("offset_x : " << offset_x);

  double offset_y;
  private_nh.getParam("offset_y", offset_y);
  ROS_INFO_STREAM("offset_y : " << offset_y);

  double offset_z;
  private_nh.getParam("offset_z", offset_z);
  ROS_INFO_STREAM("offset_z : " << offset_z);

  double offset_roll;
  private_nh.getParam("offset_roll", offset_roll);
  ROS_INFO_STREAM("offset_roll : " << offset_roll);

  double offset_pitch;
  private_nh.getParam("offset_pitch", offset_pitch);
  ROS_INFO_STREAM("offset_pitch : " << offset_pitch);

  double offset_yaw;
  private_nh.getParam("offset_yaw", offset_yaw);
  ROS_INFO_STREAM("offset_yaw : " << offset_yaw);

  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(topic_name, 10, true);

  visualization_msgs::Marker marker;
  marker.header.frame_id = base_frame;
  marker.header.stamp = ros::Time();
  marker.ns = topic_name;
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = model_path;
  marker.mesh_use_embedded_materials = true;
  marker.pose.position.x = offset_x;
  marker.pose.position.y = offset_y;
  marker.pose.position.z = offset_z;
  double roll = offset_roll * (M_PI / 180.0);
  double yaw = offset_yaw * (M_PI / 180.0);
  double pitch = offset_pitch * (M_PI / 180.0);
  marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
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
