/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// headers in STL
#include <chrono>
#include <cmath>

// headers in PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>

// headers in ROS
#include <tf/transform_datatypes.h>

// headers in local files
#include "autoware_msgs/DetectedObjectArray.h"
#include "lidar_point_pillars/point_pillars_ros.h"

PointPillarsROS::PointPillarsROS()
  : private_nh_("~")
  , has_subscribed_baselink_(false)
  , NUM_POINT_FEATURE_(4)
  , OUTPUT_NUM_BOX_FEATURE_(7)
  , TRAINED_SENSOR_HEIGHT_(1.73f)
  , NORMALIZING_INTENSITY_VALUE_(255.0f)
  , BASELINK_FRAME_("base_link")
{
  //ros related param
  private_nh_.param<bool>("baselink_support", baselink_support_, true);

  //algorithm related params
  private_nh_.param<bool>("reproduce_result_mode", reproduce_result_mode_, false);
  private_nh_.param<float>("score_threshold", score_threshold_, 0.5f);
  private_nh_.param<float>("nms_overlap_threshold", nms_overlap_threshold_, 0.5f);
  private_nh_.param<std::string>("pfe_onnx_file", pfe_onnx_file_, "");
  private_nh_.param<std::string>("rpn_onnx_file", rpn_onnx_file_, "");

  point_pillars_ptr_.reset(new PointPillars(reproduce_result_mode_, score_threshold_, nms_overlap_threshold_,
                                            pfe_onnx_file_, rpn_onnx_file_));
}

void PointPillarsROS::createROSPubSub()
{
  sub_points_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_raw", 1, &PointPillarsROS::pointsCallback, this);
  pub_objects_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
}

geometry_msgs::Pose PointPillarsROS::getTransformedPose(const geometry_msgs::Pose& in_pose, const tf::Transform& tf)
{
  tf::Transform transform;
  geometry_msgs::PoseStamped out_pose;
  transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
  transform.setRotation(
      tf::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
  geometry_msgs::PoseStamped pose_out;
  tf::poseTFToMsg(tf * transform, out_pose.pose);
  return out_pose.pose;
}

void PointPillarsROS::pubDetectedObject(const std::vector<float>& detections, const std_msgs::Header& in_header)
{
  autoware_msgs::DetectedObjectArray objects;
  objects.header = in_header;
  int num_objects = detections.size() / OUTPUT_NUM_BOX_FEATURE_;
  for (size_t i = 0; i < num_objects; i++)
  {
    autoware_msgs::DetectedObject object;
    object.header = in_header;
    object.valid = true;
    object.pose_reliable = true;

    object.pose.position.x = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 0];
    object.pose.position.y = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 1];
    object.pose.position.z = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 2];

    // Trained this way
    float yaw = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 6];
    yaw += M_PI/2;
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(-yaw);
    object.pose.orientation = q;

    if (baselink_support_)
    {
      object.pose = getTransformedPose(object.pose, angle_transform_inversed_);
    }

    // Again: Trained this way
    object.dimensions.x = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 4];
    object.dimensions.y = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 3];
    object.dimensions.z = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 5];

    //Only detects car in Version 1.0
    object.label = "car";

    objects.objects.push_back(object);
  }
  pub_objects_.publish(objects);
}

void PointPillarsROS::getBaselinkToLidarTF(const std::string& target_frameid)
{
  try
  {
    tf_listener_.waitForTransform(BASELINK_FRAME_, target_frameid, ros::Time(0), ros::Duration(1.0));
    tf_listener_.lookupTransform(BASELINK_FRAME_, target_frameid, ros::Time(0), baselink2lidar_);
    analyzeTFInfo(baselink2lidar_);
    has_subscribed_baselink_ = true;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

void PointPillarsROS::analyzeTFInfo(tf::StampedTransform baselink2lidar)
{
  tf::Vector3 v = baselink2lidar.getOrigin();
  offset_z_from_trained_data_ = v.getZ() - TRAINED_SENSOR_HEIGHT_;

  tf::Quaternion q = baselink2lidar_.getRotation();
  angle_transform_ = tf::Transform(q);
  angle_transform_inversed_ = angle_transform_.inverse();
}

void PointPillarsROS::pclToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, float* out_points_array,
                                 const float offset_z)
{
  for (size_t i = 0; i < in_pcl_pc_ptr->size(); i++)
  {
    pcl::PointXYZI point = in_pcl_pc_ptr->at(i);
    out_points_array[i * NUM_POINT_FEATURE_ + 0] = point.x;
    out_points_array[i * NUM_POINT_FEATURE_ + 1] = point.y;
    out_points_array[i * NUM_POINT_FEATURE_ + 2] = point.z + offset_z;
    out_points_array[i * NUM_POINT_FEATURE_ + 3] = float(point.intensity / NORMALIZING_INTENSITY_VALUE_);
  }
}

void PointPillarsROS::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *pcl_pc_ptr);

  if (baselink_support_)
  {
    if (!has_subscribed_baselink_)
    {
      getBaselinkToLidarTF(msg->header.frame_id);
    }
    pcl_ros::transformPointCloud(*pcl_pc_ptr, *pcl_pc_ptr, angle_transform_);
  }

  float* points_array = new float[pcl_pc_ptr->size() * NUM_POINT_FEATURE_];
  if (baselink_support_ && has_subscribed_baselink_)
  {
    pclToArray(pcl_pc_ptr, points_array, offset_z_from_trained_data_);
  }
  else
  {
    pclToArray(pcl_pc_ptr, points_array);
  }

  std::vector<float> out_detection;
  point_pillars_ptr_->doInference(points_array, pcl_pc_ptr->size(), out_detection);

  delete[] points_array;
  pubDetectedObject(out_detection, msg->header);
}
