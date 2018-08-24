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

double start_angle_;
double end_angle_;
double angular_resolution_;

void callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{

  PointCloudT::Ptr cloud = boost::make_shared<PointCloudT>();
  pcl::fromROSMsg(*pc, *cloud);

  size_t height = 8;   // 8 layers
  size_t width = round((start_angle_ - end_angle_) / angular_resolution_) + 1;

  PointT invalid;
  invalid.x = invalid.y = invalid.z = std::numeric_limits<float>::quiet_NaN();
  PointCloudT::Ptr cloud_out = boost::make_shared<PointCloudT>(width, height, invalid);
  cloud_out->is_dense = false;

  for (size_t i = 0; i < cloud->size(); i++)
  {
    const PointT &p = cloud->points[i];
    int col = round((atan2(p.y, p.x) - end_angle_) / angular_resolution_);
    int row = p.layer;

    if (col < 0 || col >= width || row < 0 || row >= height)
    {
      ROS_ERROR("Invalid coordinates: (%d, %d) is outside (0, 0)..(%zu, %zu)!", col, row, width, height);
      continue;
    }
    (*cloud_out)[row * width + col] = p;
  }

  sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cloud_out, *msg);
  msg->header = pc->header;
  pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_ldmrs_make_organized");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  start_angle_ = private_nh.param("start_angle", 0.872664625997);
  end_angle_ = private_nh.param("end_angle", -1.0471975512);
  angular_resolution_ = private_nh.param("angular_resolution", 0.125 * M_PI / 180.0);

  ros::Subscriber sub = nh.subscribe("cloud", 1, callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("organized", 1);

  ros::spin();

  return 0;
}
