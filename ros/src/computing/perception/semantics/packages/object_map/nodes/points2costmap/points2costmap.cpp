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
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace
{
constexpr double HEIGHT_LIMIT = 0.1;  // from sensor
constexpr double CAR_LENGTH = 4.5;
constexpr double CAR_WIDTH = 1.75;

ros::Publisher g_costmap_pub;
double g_resolution;
int g_cell_width;
int g_cell_height;

std::vector<int> createCostMap(const pcl::PointCloud<pcl::PointXYZ> &scan)
{
  std::vector<int> cost_map(g_cell_width * g_cell_height, 0);
  double map_center_x = (g_cell_width / 2.0) * g_resolution;
  double map_center_y = (g_cell_height / 2.0) * g_resolution;

  // scan points are in sensor frame
  for (const auto &p : scan.points)
  {
    if (p.z > HEIGHT_LIMIT)
      continue;
    if (std::fabs(p.x) < CAR_LENGTH && std::fabs(p.y) < CAR_WIDTH)
      continue;

    // Calculate grid index
    int grid_x = (p.x + map_center_x) / g_resolution;
    int grid_y = (p.y + map_center_y) / g_resolution;
    if (grid_x < 0 || grid_x >= g_cell_width || grid_y < 0 || grid_y >= g_cell_height)
      continue;

    int index = g_cell_width * grid_y + grid_x;
    cost_map[index] += 15;

    // Max cost value is 100
    if (cost_map[index] > 100)
      cost_map[index] = 100;
  }

  return cost_map;
}

void setOccupancyGrid(nav_msgs::OccupancyGrid *og)
{
  og->info.resolution = g_resolution;
  og->info.width = g_cell_width;
  og->info.height = g_cell_height;
  og->info.origin.position.x = (-1) * (g_cell_width / 2.0) * g_resolution;
  og->info.origin.position.y = (-1) * (g_cell_height / 2.0) * g_resolution;
  og->info.origin.orientation.x = 0.0;
  og->info.origin.orientation.y = 0.0;
  og->info.origin.orientation.z = 0.0;
  og->info.origin.orientation.w = 1.0;
}

void createOccupancyGrid(const sensor_msgs::PointCloud2::ConstPtr &input)
{
  static int count = 0;
  pcl::PointCloud<pcl::PointXYZ> scan;
  pcl::fromROSMsg(*input, scan);

  static nav_msgs::OccupancyGrid og;
  if (!count)
    setOccupancyGrid(&og);

  og.header = input->header;

  // create cost map with pointcloud
  std::vector<int> cost_map = createCostMap(scan);
  og.data.insert(og.data.end(), cost_map.begin(), cost_map.end());
  g_costmap_pub.publish(og);
  og.data.clear();
  count++;
}

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points2costmap");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Subscribing topic of PointCloud2 message
  std::string points_topic;

  private_nh.param<double>("resolution", g_resolution, 1.0);
  private_nh.param<int>("cell_width", g_cell_width, 50);
  private_nh.param<int>("cell_height", g_cell_height, 50);
  private_nh.param<std::string>("points_topic", points_topic, "points_lanes");

  g_costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("realtime_cost_map", 10);
  ros::Subscriber points_sub = nh.subscribe(points_topic, 10, createOccupancyGrid);

  ros::spin();
}
