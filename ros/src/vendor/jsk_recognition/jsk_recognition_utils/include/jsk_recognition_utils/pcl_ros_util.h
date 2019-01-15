// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


/**
 * This file defines several utilities for pcl <--> ros bridging.
 */

#ifndef JSK_RECOGNITION_UTILS_PCL_ROS_UTIL_H_
#define JSK_RECOGNITION_UTILS_PCL_ROS_UTIL_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <pcl_msgs/PointIndices.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/crop_box.h>
#include <sensor_msgs/PointCloud2.h>

#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_recognition_utils
{
  /**
   * @brief
   * Convert pcl::PointIndices to pcl_msgs::PointIndices
   * and publish it with overriding header.
   */
  void publishPointIndices(ros::Publisher& pub,
                           const pcl::PointIndices& indices,
                           const std_msgs::Header& header);
  
  /**
   * @brief
   * Return true if a and b are the same frame_id
   */
  bool isSameFrameId(const std::string& a, const std::string& b);

  /**
   * @brief
   * Return true if a and b have the same frame_id
   */
  bool isSameFrameId(const std_msgs::Header& a, const std_msgs::Header& b);

  /**
   * @brief
   * Return true if a and b have the same frame_id
   */
  template<class T1, class T2>
  bool isSameFrameId(const T1& a, const T2& b)
  {
    return isSameFrameId(a.header, b.header);
  }
  
  /**
   * @brief
   * check if sensor_msgs/PointCloud2 message has the specified field.
   */
  bool hasField(const std::string& field_name, const sensor_msgs::PointCloud2& msg);

  /**
   * @brief
   * Crop point cloud with jsk_recognition_msgs/BoundingBox.
   */
  template<typename PointT>
  void
  cropPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                 const jsk_recognition_msgs::BoundingBox& bbox_msg,
                 std::vector<int>* indices,
                 bool extract_removed_indices=false)
  {
    if (cloud->header.frame_id != bbox_msg.header.frame_id)
    {
      fprintf(stderr, "Frame id of input cloud and bounding box must be same. Cloud: %s, BoundingBox: %s.",
              cloud->header.frame_id.c_str(), bbox_msg.header.frame_id.c_str());
      return;
    }
    pcl::CropBox<PointT> crop_box(/*extract_removed_indices=*/extract_removed_indices);

    crop_box.setInputCloud(cloud);

    Eigen::Affine3f box_pose;
    tf::poseMsgToEigen(bbox_msg.pose, box_pose);
    crop_box.setTranslation(box_pose.translation());
    float roll, pitch, yaw;
    pcl::getEulerAngles(box_pose, roll, pitch, yaw);
    crop_box.setRotation(Eigen::Vector3f(roll, pitch, yaw));

    Eigen::Vector4f max_point(bbox_msg.dimensions.x / 2,
                              bbox_msg.dimensions.y / 2,
                              bbox_msg.dimensions.z / 2,
                              0);
    Eigen::Vector4f min_point(-bbox_msg.dimensions.x / 2,
                              -bbox_msg.dimensions.y / 2,
                              -bbox_msg.dimensions.z / 2,
                              0);
    crop_box.setMax(max_point);
    crop_box.setMin(min_point);

    crop_box.filter(*indices);
  }

}  // namespace jsk_recognition_utils

#endif  // JSK_RECOGNITION_UTILS_PCL_ROS_UTIL_H_
