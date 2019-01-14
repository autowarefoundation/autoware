/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

static pcl::PointCloud<pcl::PointXYZ> _vscan;
static visualization_msgs::Marker _linelist;
static ros::Publisher _pub;
const std::string FRAME = "/velodyne";

static void Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::fromROSMsg(*msg, _vscan);
  //  int i = 0;
  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = _vscan.begin(); item != _vscan.end(); item++)
  {
    if ((item->x == 0 && item->y == 0))
      continue;
    // if (i < _loop_limit) {
    // std::cout << "vscan_points : ( " << item->x << " , " << item->y << " , " << item->z << " )" << std::endl;

    geometry_msgs::Point point;
    point.x = item->x;
    point.y = item->y;
    point.z = item->z;
    _linelist.points.push_back(point);

    //}else{
    // break;
    // }
    // i++;
  }
  // std::cout << "i = " << i << std::endl;
  _pub.publish(_linelist);
  _linelist.points.clear();
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vscan2linelist");
  ros::NodeHandle nh;

  _pub = nh.advertise<visualization_msgs::Marker>("vscan_linelist", 10);
  ros::Subscriber sub = nh.subscribe("vscan_points", 100, Callback);

  _linelist.header.frame_id = FRAME;
  _linelist.header.stamp = ros::Time(0);
  _linelist.ns = "vscan_linelist";
  _linelist.id = 0;
  _linelist.type = visualization_msgs::Marker::LINE_LIST;
  _linelist.action = visualization_msgs::Marker::ADD;
  _linelist.scale.x = 0.15;
  _linelist.color.a = 0.5;
  _linelist.color.r = 0.0;
  _linelist.color.g = 1.0;
  _linelist.color.b = 0.0;

  ros::spin();
  return 0;
}
