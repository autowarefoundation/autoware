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

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sick_ldmrs_msgs/sick_ldmrs_point_type.h>
#include <pcl/point_cloud.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>   // necessary because of custom point type

typedef sick_ldmrs_msgs::SICK_LDMRS_Point PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

tf::TransformListener* tf_;
ros::Publisher pub_;

std::string fixed_frame_;

PointCloudT::Ptr cloud_lower_ = boost::make_shared<PointCloudT>();

void callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
  PointCloudT::Ptr cloud = boost::make_shared<PointCloudT>();
  pcl::fromROSMsg(*pc, *cloud);

  // ----- transform to fixed frame
  try
  {
    PointCloudT::Ptr cloud_fixed = boost::make_shared<PointCloudT>();

    if (!pcl_ros::transformPointCloud(fixed_frame_, *cloud, *cloud_fixed, *tf_))
    {
      ROS_WARN("TF exception in transformPointCloud!");
      cloud_lower_.reset();
      return ;
    }
    cloud = cloud_fixed;
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("TF Exception %s", ex.what());
    cloud_lower_.reset();
    return ;
  }

  // ----- check that we have both clouds (for lower + upper layers)
  if (cloud->size() == 0)
    return;

  if (cloud->at(0).layer < 4)
  {
    cloud_lower_ = cloud;
    return;   // wait for upper 4 layer cloud
  }
  else if (!cloud_lower_)
  {
    return;   // wait for lower 4 layer cloud first
  }

  // ----- concatenate lower + upper clouds
  *cloud_lower_ += *cloud;

  // ----- publish
  sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cloud_lower_, *msg);
  msg->header.stamp = pc->header.stamp;
  msg->header.frame_id = fixed_frame_;
  pub_.publish(msg);

  cloud_lower_.reset();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_ldmrs_all_layer_assembler");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  if (!private_nh.getParam("fixed_frame", fixed_frame_))
  {
    ROS_FATAL("Need to set parameter fixed_frame");
    return 1;
  }

  tf_ = new tf::TransformListener(nh, ros::Duration(3.0));

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub;
  tf::MessageFilter<sensor_msgs::PointCloud2>* tf_filter;

  sub.subscribe(nh, "cloud", 10);
  tf_filter = new tf::MessageFilter<sensor_msgs::PointCloud2>(sub, *tf_, fixed_frame_, 10);
  tf_filter->registerCallback(boost::bind(callback, _1));

  pub_ = nh.advertise<sensor_msgs::PointCloud2>("all_layers", 10);

  ros::spin();

  delete tf_filter;
  delete tf_;

  return 0;
}
