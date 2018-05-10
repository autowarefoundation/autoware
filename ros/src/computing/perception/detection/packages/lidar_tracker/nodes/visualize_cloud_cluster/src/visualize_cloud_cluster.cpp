#include "visualize_cloud_cluster.h"

#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>


VisualizeCloudCluster::VisualizeCloudCluster()
{
  ros::NodeHandle private_nh_("~");
  private_nh_.param<std::string>("pointcloud_frame", pointcloud_frame_, "velodyne");

  sub_cloud_array_  = node_handle_.subscribe ("tracking_cluster_array", 1, &VisualizeCloudCluster::callBack, this);
  pub_jsk_bb_       = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray> ("/tracking_cluster_array/jsk_bb", 1);
  pub_arrow_        = node_handle_.advertise<visualization_msgs::Marker> ("/tracking_cluster_array/velocity_arrow", 1);
  pub_id_           = node_handle_.advertise<visualization_msgs::Marker> ("/tracking_cluster_array/target_id", 1);
}

void VisualizeCloudCluster::callBack(autoware_msgs::CloudClusterArray input)
{
  jsk_recognition_msgs::BoundingBoxArray jsk_bbs;
  visualization_msgs::Marker arrows;

  getJskBBs(input, jsk_bbs);
  pub_jsk_bb_.publish(jsk_bbs);
  visMarkers(input);
}

void VisualizeCloudCluster::getJskBBs(autoware_msgs::CloudClusterArray input,
               jsk_recognition_msgs::BoundingBoxArray& jsk_bbs)
{
  jsk_bbs.header = input.header;

  for(size_t i = 0; i < input.clusters.size(); i++)
  {
    jsk_recognition_msgs::BoundingBox bb;
    bb = input.clusters[i].bounding_box;
    bb.header = input.header;
    std::string label = input.clusters[i].label;

    if(label == "Stable")
    {
      bb.label = 2;
    }
    else if(label == "Static")
    {
      bb.label = 15;
    }

    jsk_bbs.boxes.push_back(bb);
  }
}

void VisualizeCloudCluster::visMarkers(autoware_msgs::CloudClusterArray input)
{
  for(size_t i = 0; i < input.clusters.size(); i++)
  {

    double tv   = input.clusters[i].score;
    double tyaw = input.clusters[i].estimated_angle;
    std::string label = input.clusters[i].label;

    visualization_msgs::Marker ids;

    if(label == "None" || label == "Initialized" || label == "Lost")
    {
      continue;
    }
    ids.lifetime = ros::Duration(0.15);
    ids.header.frame_id = pointcloud_frame_;
    ids.header.stamp = input.header.stamp;
    ids.ns = "ids";
    ids.action = visualization_msgs::Marker::ADD;
    ids.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // green
    ids.color.g = 1.0f;
    ids.color.a = 1.0;
    ids.id = i;


    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    ids.pose.position.x = input.clusters[i].bounding_box.pose.position.x;
    ids.pose.position.y = input.clusters[i].bounding_box.pose.position.y;
    ids.pose.position.z = 1.5;

    // convert from RPY to quartenion
    tf::Matrix3x3 obs_mat;
    obs_mat.setEulerYPR(tyaw, 0, 0); // yaw, pitch, roll
    tf::Quaternion q_tf;
    obs_mat.getRotation(q_tf);
    ids.pose.orientation.x = q_tf.getX();
    ids.pose.orientation.y = q_tf.getY();
    ids.pose.orientation.z = q_tf.getZ();
    ids.pose.orientation.w = q_tf.getW();


    ids.scale.z = 1.0;

    ids.text = std::to_string(input.clusters[i].id);

    pub_id_.publish(ids);


    visualization_msgs::Marker arrows;
    arrows.lifetime = ros::Duration(0.1);

    if(label == "None" || label == "Initialized" || label == "Lost" || label == "Static")
    {
      continue;
    }

    arrows.header.frame_id = pointcloud_frame_;
    arrows.header.stamp = input.header.stamp;
    arrows.ns = "arrows";
    arrows.action = visualization_msgs::Marker::ADD;
    arrows.type   = visualization_msgs::Marker::ARROW;
    // green
    arrows.color.g = 1.0f;
    arrows.color.a = 1.0;
    arrows.id = i;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    arrows.pose.position.x = input.clusters[i].bounding_box.pose.position.x;
    arrows.pose.position.y = input.clusters[i].bounding_box.pose.position.y;
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
