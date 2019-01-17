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
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <utility>

namespace
{
constexpr double HEIGHT_LIMIT = 0.1;  // from sensor
constexpr double CAR_LENGTH = 4.5;
constexpr double CAR_WIDTH = 1.75;

ros::Publisher g_costmap_pub;
double g_resolution;
int g_cell_width;
int g_cell_height;
double g_offset_x;
double g_offset_y;
double g_offset_z;

pcl::PointCloud<pcl::PointXYZ> g_obstacle_sim_points;
bool g_use_obstacle_sim = false;

void callbackFromObstacleSim(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::fromROSMsg(*msg, g_obstacle_sim_points);

  g_use_obstacle_sim = true;
}

void joinPoints(const pcl::PointCloud<pcl::PointXYZ>& points1, pcl::PointCloud<pcl::PointXYZ>* points2)
{
  for (const auto& p : points1)
  {
    points2->push_back(p);
  }
}

std::vector<int> createCostMap(const pcl::PointCloud<pcl::PointXYZ> &scan)
{
  std::vector<int> cost_map(g_cell_width * g_cell_height, 0);
  double map_center_x = (g_cell_width / 2.0) * g_resolution - g_offset_x;
  double map_center_y = (g_cell_height / 2.0) * g_resolution - g_offset_y;

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
  og->info.origin.position.x = (-1) * (g_cell_width / 2.0) * g_resolution + g_offset_x;
  og->info.origin.position.y = (-1) * (g_cell_height / 2.0) * g_resolution + g_offset_y;
  og->info.origin.position.z = g_offset_z;
  og->info.origin.orientation.x = 0.0;
  og->info.origin.orientation.y = 0.0;
  og->info.origin.orientation.z = 0.0;
  og->info.origin.orientation.w = 1.0;
}

std::vector<int> filterCostMap(std::vector<int>& cost_map)
{
  std::vector<int> filtered_cost_map(cost_map.size(), 0);

  // cells around reference (x, y)
  std::vector<std::pair<int, int>> neighborhood
  {
    std::make_pair(-1, -1), std::make_pair( 0, -1), std::make_pair( 1, -1),
    std::make_pair(-1,  0), std::make_pair( 1,  0),
    std::make_pair(-1,  1), std::make_pair( 0,  1), std::make_pair( 1,  1),
  };

  for (size_t size = cost_map.size(), i = 0; i < size; i++) {
    int ref_cost = cost_map[i];

    int ref_x = i % g_cell_width;
    int ref_y = (i - ref_x) / g_cell_width;

    // we don't have to filter if the cost is 0
    if (ref_cost <= 0)
      continue;

    filtered_cost_map[i] += ref_cost;

    // increase the cost for each neighborhood cell
    for (const auto& n : neighborhood) {
      int neighbor_x = ref_x + n.first;
      int neighbor_y = ref_y + n.second;

      if (neighbor_x < 0 || neighbor_x >= g_cell_width || neighbor_y < 0 || neighbor_y >= g_cell_height)
        continue;

      int neighbor_index = neighbor_x + neighbor_y * g_cell_width;
      filtered_cost_map[neighbor_index] += ref_cost;
    }

  }

  // handle the cost over 100
  for (auto &cost : filtered_cost_map) {
    if (cost > 100)
      cost = 100;
  }

  return filtered_cost_map;
}

void createOccupancyGrid(const sensor_msgs::PointCloud2::ConstPtr &input)
{
  static int count = 0;
  pcl::PointCloud<pcl::PointXYZ> scan;
  pcl::fromROSMsg(*input, scan);

  // use simulated obstacle
  if (g_use_obstacle_sim)
  {
    joinPoints(g_obstacle_sim_points, &scan);
    g_obstacle_sim_points.clear();
  }
  // ---

  static nav_msgs::OccupancyGrid og;
  if (!count)
    setOccupancyGrid(&og);

  og.header = input->header;

  // create cost map with pointcloud
  std::vector<int> cost_map = createCostMap(scan);

  /*
  bool filter = false;
  if (filter)
    cost_map = filterCostMap(cost_map);
  */

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
  private_nh.param<double>("offset_x", g_offset_x, 30.0);
  private_nh.param<double>("offset_y", g_offset_y, 0.0);
  private_nh.param<double>("offset_z", g_offset_z, -2.0);

  g_costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("realtime_cost_map", 10);
  ros::Subscriber points_sub = nh.subscribe(points_topic, 10, createOccupancyGrid);
  ros::Subscriber obstacle_sim_points_sub = nh.subscribe("obstacle_sim_pointcloud", 1, callbackFromObstacleSim);

  ros::spin();
}
