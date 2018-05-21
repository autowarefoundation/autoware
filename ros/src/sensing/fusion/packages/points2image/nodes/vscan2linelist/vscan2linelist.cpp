/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
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
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
