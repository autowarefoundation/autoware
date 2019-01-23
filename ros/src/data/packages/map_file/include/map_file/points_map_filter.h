/*
 *  Copyright (c) 2018, TierIV, Inc
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 * this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef POINTS_MAP_FILTER_H_INCLUDED
#define POINTS_MAP_FILTER_H_INCLUDED

// headers in ROS
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// headers in PCL
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// headers in boost
#include <boost/optional.hpp>

// headers in STL
#include <mutex>
#include <regex>

class points_map_filter {
public:
  points_map_filter(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~points_map_filter();
  void init();
  void run();

private:
  pcl::PassThrough<pcl::PointXYZ> pass_;
  ros::Subscriber map_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher map_pub_;
  std::mutex mtx_;
  ros::NodeHandle nh_, pnh_;
  void map_callback_(const sensor_msgs::PointCloud2::ConstPtr msg);
  void current_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg);
  double load_grid_size_;
  double load_trigger_distance_;
  std::string map_frame_;
  boost::optional<geometry_msgs::PoseStamped> last_load_pose_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  volatile bool map_recieved_;
};

#endif // POINTS_MAP_FILTER_H_INCLUDED