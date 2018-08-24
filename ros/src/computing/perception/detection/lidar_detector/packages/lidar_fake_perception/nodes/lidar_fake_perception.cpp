/*
*/

#include "lidar_fake_perception.h"

LidarFakePerception::LidarFakePerception() : nh_(), private_nh_("~")
{
  private_nh_.param<bool>("publish_objects", publish_objects_, true);
  private_nh_.param<bool>("publish_points", publish_points_, true);
  private_nh_.param<double>("publish_rate", publish_rate_, 10.0);

  private_nh_.param<double>("object_length", object_length_, 4.8);
  private_nh_.param<double>("object_width", object_width_, 1.8);
  private_nh_.param<double>("object_height", object_height_, 1.8);
  private_nh_.param<double>("object_z_offset", object_z_offset_, 0.0);

  private_nh_.param<bool>("use_fake_twist", use_fake_twist_, false);
  private_nh_.param<double>("object_velocity", object_velocity_, 3.0);
  private_nh_.param<double>("object_angular_velocity", object_angular_velocity_, 0.0);

  private_nh_.param<double>("object_intensity", object_intensity_, 100.0);
  private_nh_.param<double>("object_lifetime", object_lifetime_, -1);
  private_nh_.param<double>("object_points_space", object_points_space_, 0.2);

  private_nh_.param<std::string>("object_label", object_label_, "Stable");
  private_nh_.param<std::string>("object_frame", object_frame_, "velodyne");

  object_initial_pose_sub_ =
      nh_.subscribe("/move_base_simple/goal", 1, &LidarFakePerception::objectInitialPoseCallback, this);
  real_objects_sub_ = nh_.subscribe("/detected_objects", 1, &LidarFakePerception::objectsCallback, this);
  real_points_sub_ = nh_.subscribe("/points_raw", 1, &LidarFakePerception::pointsCallback, this);
  fake_twist_sub_ = nh_.subscribe("/fake_twist", 1, &LidarFakePerception::twistCallback, this);

  fake_objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/fake_objects", 1);
  fake_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fake_points", 1);

  fake_object_id_ = 0;      // overwritten by real object ids
  global_frame_ = "world";  // overwritten by object initial pose

  fake_object_pose_initialized_ = false;
}

LidarFakePerception::~LidarFakePerception()
{
}

void LidarFakePerception::run()
{
  ros::Rate rate(publish_rate_);

  while (ros::ok())
  {
    ros::spinOnce();
    updateFakes();
    publishFakes();
    rate.sleep();
  }
}

void LidarFakePerception::objectInitialPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  global_frame_ = msg->header.frame_id;
  fake_object_initial_time_ = msg->header.stamp;

  tf::poseMsgToTF(msg->pose, fake_object_pose_);

  fake_object_pose_initialized_ = true;
}

void LidarFakePerception::objectsCallback(const autoware_msgs::DetectedObjectArray& msg)
{
  real_objects_ = msg;
}

void LidarFakePerception::pointsCallback(const sensor_msgs::PointCloud2& msg)
{
  pcl::fromROSMsg(msg, real_points_);
}

void LidarFakePerception::twistCallback(const geometry_msgs::Twist& msg)
{
  fake_object_twist_ = msg;
}

void LidarFakePerception::updateFakes()
{
  // update objects
  fake_objects_.objects.clear();
  fake_points_.clear();

  if (real_objects_.objects.size() != 0)
  {
    fake_objects_.objects.resize(real_objects_.objects.size());
    std::copy(real_objects_.objects.begin(), real_objects_.objects.end(), fake_objects_.objects.begin());
    fake_object_id_ = (real_objects_.objects.end() - 1)->id + 1;  // using incremented id
  }

  if (real_points_.size() != 0)
  {
    fake_points_.resize(real_points_.size());
    std::copy(real_points_.begin(), real_points_.end(), fake_points_.begin());
  }

  if (fake_object_pose_initialized_)
  {
    if (updateFakeObject())
    {
      updateFakePoints();
      fake_objects_.objects.push_back(fake_object_);
    }
  }
}

bool LidarFakePerception::updateFakeObject()
{
  // check lifetime
  if (object_lifetime_ > 0)
  {
    if ((ros::Time::now() - fake_object_initial_time_).toSec() > object_lifetime_)
    {
      return false;
    }
  }

  // obtain world -> sensor frame
  tf::StampedTransform global2local;
  try
  {
    tf_listener_.waitForTransform(global_frame_, object_frame_, ros::Time(0), ros::Duration(1.0));
    tf_listener_.lookupTransform(global_frame_, object_frame_, ros::Time(0), global2local);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  // update pose
  geometry_msgs::Twist twist;
  if (use_fake_twist_)
  {
    // update pose by subscribed twist
    twist = fake_object_twist_;
  }
  else
  {
    // update pose by rosparam
    twist.linear.x = object_velocity_;
    twist.angular.z = object_angular_velocity_;
  }
  updatePose(1. / publish_rate_, twist, fake_object_pose_);

  // fix height
  tf::Vector3 objpos = fake_object_pose_.getOrigin();
  fake_object_pose_.setOrigin(tf::Vector3(objpos.x(), objpos.y(), object_z_offset_ + object_height_ / 2.));

  // update msg
  fake_object_.header.stamp = ros::Time::now();
  fake_object_.header.frame_id = object_frame_;
  fake_object_.id = fake_object_id_;
  fake_object_.label = object_label_;
  fake_object_.dimensions.x = object_length_;
  fake_object_.dimensions.y = object_width_;
  fake_object_.dimensions.z = object_height_;
  fake_object_.velocity = twist;
  fake_object_.acceleration = geometry_msgs::Twist();                              // NOTE: by constant velocity model
  tf::poseTFToMsg(global2local.inverse() * fake_object_pose_, fake_object_.pose);  // local

  return true;
}

void LidarFakePerception::updatePose(const double& dt, const geometry_msgs::Twist& twist, tf::Transform& tf)
{
  // update pose by constant velocity model
  tf::Transform dpose;
  tf::Quaternion dquat;
  dpose.setOrigin(tf::Vector3(twist.linear.x * dt, twist.linear.y * dt, twist.linear.z * dt));
  dquat.setRPY(twist.angular.x * dt, twist.angular.y * dt, twist.angular.z * dt);
  dpose.setRotation(dquat);
  fake_object_pose_ *= dpose;  // transform
}

void LidarFakePerception::updateFakePoints()
{
  PointCloudT points;
  convertObjectToPoints(fake_object_, points);
  fake_points_ += points;
  fake_points_.header = pcl_conversions::toPCL(fake_object_.header);
  pcl::toROSMsg(fake_points_, fake_object_.pointcloud);
}

void LidarFakePerception::convertObjectToPoints(const autoware_msgs::DetectedObject& obj, PointCloudT& points)
{
  points.clear();

  double hl = object_length_ / 2.0;
  double hw = object_width_ / 2.0;
  double hh = object_height_ / 2.0;

  for (double x = -hl; x <= hl; x += object_points_space_)
  {
    for (double y = -hw; y <= hw; y += object_points_space_)
    {
      for (double z = -hh; z <= hh; z += object_points_space_)
      {
        PointT point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = object_intensity_;
        points.push_back(point);
      }
    }
  }

  tf::Transform trans_tf;
  Eigen::Affine3d trans_eigen;
  tf::poseMsgToTF(obj.pose, trans_tf);
  tf::poseTFToEigen(trans_tf, trans_eigen);
  pcl::transformPointCloud(points, points, trans_eigen);
}

void LidarFakePerception::publishFakes()
{
  if (fake_objects_.objects.size() == 0)  // publish empty
  {
    fake_objects_.header.frame_id = global_frame_;
    fake_objects_.header.stamp = ros::Time::now();
  }

  if (fake_points_.size() != 0)  // publish empty
  {
    fake_points_.header = pcl_conversions::toPCL(fake_object_.header);
  }

  if (publish_objects_)
  {
    fake_objects_pub_.publish(fake_objects_);
  }

  if (publish_points_)
  {
    fake_points_pub_.publish(fake_points_);
  }
}
