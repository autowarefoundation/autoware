/*
*/

#ifndef LIDAR_FAKE_PERCEPTION_H
#define LIDAR_FAKE_PERCEPTION_H

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

class LidarFakePerception
{
public:
  LidarFakePerception();
  ~LidarFakePerception();

  void run();

private:
  // typedefs
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

  // nodehandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher fake_objects_pub_;  // output objects, input and fake object are merged
  ros::Publisher fake_points_pub_;   // output points, input and fake points are merged

  // subscriber
  ros::Subscriber object_initial_pose_sub_;  // initial fake object pose
  ros::Subscriber real_objects_sub_;         // input objects
  ros::Subscriber real_points_sub_;          // input points
  ros::Subscriber fake_twist_sub_;           // input twist command, controlling fake object

  // tf
  tf::TransformListener tf_listener_;

  // param, publishing
  bool publish_objects_;  // enable publishing fake objects
  bool publish_points_;   // enable publishing fake points
  double publish_rate_;   // publish rate of fake objects/points [Hz]

  // param, object shape
  double object_length_;    // length [m]
  double object_width_;     // width [m]
  double object_height_;    // height [m]
  double object_z_offset_;  // z offset from global frame [m]

  // param, object motion
  bool use_fake_twist_;             // using subscribed twist
  double object_velocity_;          // constant velocity instead of subscribed twist [m/s]
  double object_angular_velocity_;  // constant angular velocity instead of subscribed twist [rad/s]

  // param, object meta-info
  double object_intensity_;  // constant intensity value of fake points, 0-255 [-]
  double object_lifetime_;   // fake object lifetime (NOTE: when this is negative value, lifetime is inifinity) [s]
  double object_points_space_;  // fake points space [m]
  std::string object_label_;    // fake object label (e.g. tracking state)
  std::string object_frame_;    // fake object frame_id (NOTE: not affected to input object)

  // variables
  int fake_object_id_;
  bool fake_object_pose_initialized_;
  ros::Time fake_object_initial_time_;
  std::string global_frame_;
  tf::Transform fake_object_pose_;  // global
  geometry_msgs::Twist fake_object_twist_;
  autoware_msgs::DetectedObject fake_object_;
  autoware_msgs::DetectedObjectArray fake_objects_;
  autoware_msgs::DetectedObjectArray real_objects_;
  PointCloudT fake_points_;
  PointCloudT real_points_;

  // functions, callback
  void objectInitialPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void objectsCallback(const autoware_msgs::DetectedObjectArray& msg);
  void pointsCallback(const sensor_msgs::PointCloud2& msg);
  void twistCallback(const geometry_msgs::Twist& msg);

  // functions, mainloop
  void updateFakes();
  bool updateFakeObject();
  void updatePose(const double& dt, const geometry_msgs::Twist& twist, tf::Transform& tf);
  void updateFakePoints();
  void convertObjectToPoints(const autoware_msgs::DetectedObject& obj, PointCloudT& points);
  void publishFakes();
};

#endif
