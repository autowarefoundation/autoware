/*
 * Copyright (C) 2016, DFKI GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of DFKI GmbH nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Martin Günther <martin.guenther@dfki.de>
 *         Jochen Sprickerhof <ros@jochen.sprickerhof.de>
 *
 */

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sick_ldmrs_msgs/sick_ldmrs_point_type.h>
#include <pcl/point_cloud.h>

typedef sick_ldmrs_msgs::SICK_LDMRS_Point PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

ros::Publisher pub;
ros::Publisher pub_bg;

double min_dist_ratio_;

PointCloudT::Ptr cloud_bg_;

void callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
  PointCloudT::Ptr cloud = boost::make_shared<PointCloudT>();
  pcl::fromROSMsg(*pc, *cloud);

  if (!cloud->isOrganized())
  {
    ROS_ERROR_THROTTLE(15.0, "Input point cloud is not organized!");
    return;
  }

  if (cloud_bg_ && (cloud->width != cloud_bg_->width || cloud->height != cloud_bg_->height))
  {
    ROS_INFO("Dimensions of input cloud different from stored background, resetting background.");
    cloud_bg_.reset();
  }

  if (!cloud_bg_)
  {
    cloud_bg_ = cloud;
    return;
  }

  PointT invalid;
  invalid.x = invalid.y = invalid.z = std::numeric_limits<float>::quiet_NaN();
//  PointCloudT::Ptr cloud_out = boost::make_shared<PointCloudT>(width, height, invalid);
//  cloud_out->is_dense = false;

  for (size_t i = 0; i < cloud->size(); i++)
  {
    const PointT &p_in = cloud->points[i];
    const PointT &p_bg = cloud_bg_->points[i];

    // if input point invalid -> output invalid
    if (!pcl::isFinite<PointT>(p_in))
    {
      continue;
    }

    // if background invalid (and input valid) -> output valid
    if (!pcl::isFinite<PointT>(p_bg))
    {
      (*cloud_bg_)[i] = p_in;     // store new background point
      continue;
    }

    // compare ratio between background and input point to threshold
    float dist_in = p_in.getVector3fMap().norm();
    float dist_bg = p_bg.getVector3fMap().norm();
    float diff = dist_bg - dist_in;

    if (diff < 0)
    {
      // point is further away than previous background
      (*cloud_bg_)[i] = p_in;
      (*cloud)[i] = invalid;
    }
    else if (diff / dist_bg < min_dist_ratio_)
    {
      // point is too close to background
      (*cloud)[i] = invalid;
    }

    // ... otherwise, point is foreground => don't invalidate
  }

  sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cloud, *msg);
  msg->header = pc->header;
  pub.publish(msg);

  sensor_msgs::PointCloud2::Ptr msg_bg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cloud_bg_, *msg_bg);
  msg_bg->header = pc->header;
  pub_bg.publish(msg_bg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_ldmrs_remove_background");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // min_dist_ratio: a point is filtered out if the distance between it and its
  // reference point is less than 5% of the distance between the sensor and the
  // reference point.
  min_dist_ratio_ = private_nh.param("min_dist_ratio", 0.05);

  ros::Subscriber sub = nh.subscribe("cloud", 10, callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("foreground", 10);
  pub_bg = nh.advertise<sensor_msgs::PointCloud2>("background", 10);

  ros::spin();

  return 0;
}
