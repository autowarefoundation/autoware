#include "visualize_detected_objects.h"

#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

VisualizeDetectedObjects::VisualizeDetectedObjects()
{
  ros::NodeHandle private_nh_("~");
  private_nh_.param<std::string>("pointcloud_frame", pointcloud_frame_, "velodyne");

  sub_cloud_array_ = node_handle_.subscribe("/detected_objects", 1, &VisualizeDetectedObjects::callBack, this);
  pub_arrow_ = node_handle_.advertise<visualization_msgs::Marker>("/detected_objects/velocity_arrow", 1);
  pub_id_ = node_handle_.advertise<visualization_msgs::Marker>("/detected_objects/target_id", 1);
}

void VisualizeDetectedObjects::callBack(const autoware_msgs::DetectedObjectArray& input)
{
  visMarkers(input);
}

void VisualizeDetectedObjects::visMarkers(const autoware_msgs::DetectedObjectArray& input)
{
  for (size_t i = 0; i < input.objects.size(); i++)
  {
    double tv = input.objects[i].velocity.linear.x;
    double tyaw = input.objects[i].velocity.linear.y;

    visualization_msgs::Marker ids;

    ids.lifetime = ros::Duration(0.15);
    ids.header.frame_id = pointcloud_frame_;
    ids.header.stamp = input.header.stamp;
    ids.ns = "ids";
    ids.action = visualization_msgs::Marker::ADD;
    ids.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // green
    ids.color.g = 1.0f;
    ids.color.a = 1.0;
    ids.id = i;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    ids.pose.position.x = input.objects[i].pose.position.x;
    ids.pose.position.y = input.objects[i].pose.position.y;
    ids.pose.position.z = 1.5;

    // convert from RPY to quartenion
    tf::Matrix3x3 obs_mat;
    obs_mat.setEulerYPR(tyaw, 0, 0);  // yaw, pitch, roll
    tf::Quaternion q_tf;
    obs_mat.getRotation(q_tf);
    ids.pose.orientation.x = q_tf.getX();
    ids.pose.orientation.y = q_tf.getY();
    ids.pose.orientation.z = q_tf.getZ();
    ids.pose.orientation.w = q_tf.getW();

    ids.scale.z = 1.0;

    ids.text = std::to_string(input.objects[i].id);

    pub_id_.publish(ids);

    visualization_msgs::Marker arrows;
    arrows.lifetime = ros::Duration(0.1);

    // visualize velocity arrow only if its status is Stable
    std::string label = input.objects[i].label;
    if (label == "None" || label == "Initialized" || label == "Lost" || label == "Static")
    {
      continue;
    }

    arrows.header.frame_id = pointcloud_frame_;
    arrows.header.stamp = input.header.stamp;
    arrows.ns = "arrows";
    arrows.action = visualization_msgs::Marker::ADD;
    arrows.type = visualization_msgs::Marker::ARROW;
    // green
    arrows.color.g = 1.0f;
    arrows.color.a = 1.0;
    arrows.id = i;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    arrows.pose.position.x = input.objects[i].pose.position.x;
    arrows.pose.position.y = input.objects[i].pose.position.y;
    arrows.pose.position.z = 0.5;

    arrows.pose.orientation.x = q_tf.getX();
    arrows.pose.orientation.y = q_tf.getY();
    arrows.pose.orientation.z = q_tf.getZ();
    arrows.pose.orientation.w = q_tf.getW();

    // Set the scale of the arrows -- 1x1x1 here means 1m on a side
    arrows.scale.x = tv;
    arrows.scale.y = 0.1;
    arrows.scale.z = 0.1;

    pub_arrow_.publish(arrows);
  }
}
