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

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

namespace
{
constexpr auto OGM_FRAME = "/map";
constexpr int OCCUPIED_MAX = 8;
constexpr int OCCUPIED_MIN = -8;
constexpr int OCCUPIED_INCREMENT = 2;
constexpr int FREE_INCREMENT = 1;
std::string g_sensor_frame = "/velodyne";  // sensor which publihes lasescan message
std::string g_scan_topic = "/scan";        // laser scan topic
double g_resolution = 0.1;                 // [m]
int g_scan_size_x = 1000;                  // actual scanning size
int g_scan_size_y = 1000;
int g_map_size_x = 500;  // publishing occupancy grid map size
int g_map_size_y = 500;

struct Grid
{
  int index;
  int index_x, index_y;
  double x, y;
  double weight;
  double range;

  void calcCoordinate();
  void calcRange();
  int calcGlobalIndex(const tf::StampedTransform& transform) const;
};

struct Cost
{
  int occupied = 0;
  int free     = 0;
  bool unknown = true;

  void accumulateCost(int occ, int free);
};

ros::Publisher g_map_pub;
tf::TransformListener* g_tf_listenerp;

double calcYawFromQuaternion(const tf::Quaternion& q)
{
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

// Accumulate cost value
void Cost::accumulateCost(int occupied_inc, int free_inc)
{
  occupied += occupied_inc - free_inc;
  free     += free_inc;
  unknown   = false;

  if (occupied > OCCUPIED_MAX)
    occupied = OCCUPIED_MAX;

  if (occupied < OCCUPIED_MIN)
    occupied = OCCUPIED_MIN;
}

// Calcurate grid's coordinate in sensor frame
void Grid::calcCoordinate()
{
  // index coordinate
  index_x = index % g_scan_size_x;
  index_y = (index - index_x) / g_scan_size_x;

  // actual coordinate
  x = (index_x - g_scan_size_x / 2.0) * g_resolution;
  y = (index_y - g_scan_size_y / 2.0) * g_resolution;
}

// Calculate a range from sensor in a certain grid
void Grid::calcRange()
{
  double distance_x = g_resolution * fabs(g_scan_size_x / 2 - index_x);
  double distance_y = g_resolution * fabs(g_scan_size_y / 2 - index_y);

  range = sqrt(distance_x * distance_x + distance_y * distance_y);
}

// Change local index into global index
int Grid::calcGlobalIndex(const tf::StampedTransform& transform) const
{
  // Calculate index in global frame
  int ix = (x + transform.getOrigin().x()) / g_resolution;
  int iy = (y + transform.getOrigin().y()) / g_resolution;

  // Make indexes positive value
  int global_grid_x = (ix + 100000 * g_scan_size_x) % g_scan_size_x;
  int global_grid_y = (iy + 100000 * g_scan_size_y) % g_scan_size_y;

  int global_index = global_grid_x + global_grid_y * g_scan_size_x;

  return global_index;
}

int calcGlobalIndex(int grid_x, int grid_y, const tf::StampedTransform& transform)
{
  // Point coordinate in velodyne(local) frame
  double local_x = (grid_x - g_scan_size_x / 2.0) * g_resolution;
  double local_y = (grid_y - g_scan_size_y / 2.0) * g_resolution;

  // Calculate index in global coordinate
  int ix = (local_x + transform.getOrigin().x()) / g_resolution;
  int iy = (local_y + transform.getOrigin().y()) / g_resolution;

  // Make indexes positive value
  int global_grid_x = (ix + 100000 * g_scan_size_x) % g_scan_size_x;
  int global_grid_y = (iy + 100000 * g_scan_size_y) % g_scan_size_y;

  int global_index = global_grid_x + global_grid_y * g_scan_size_x;

  return global_index;
}

void preCasting(const sensor_msgs::LaserScan& scan, std::vector<std::vector<Grid>>* precasted_grids)
{
  int iangle_size = 2 * M_PI / scan.angle_increment;
  // We decide if a grid is passed by laser by this search_step
  double search_step = g_resolution / 10.0;

  // Grid indexes for each laser
  std::vector<std::vector<int>> grid_indexes(iangle_size);

  for (int iangle = 0; iangle < iangle_size; iangle++)
  {
    // Angle of each laser from Lidar
    double angle = scan.angle_increment * iangle;  // + scan.angle_min;
    double step = 0;

    while (1)
    {
      step += search_step;

      // Calculate a grid index passed by laser scan
      int grid_x = step * cos(angle) / g_resolution + g_scan_size_x / 2.0;
      int grid_y = step * sin(angle) / g_resolution + g_scan_size_y / 2.0;

      if (grid_x >= g_scan_size_x || grid_x < 0 || grid_y >= g_scan_size_y || grid_y < 0)
        break;

      grid_indexes[iangle].push_back(grid_x + grid_y * g_scan_size_x);
    }
  }

  // Erase overlapping values
  auto unique_grid_indexes = grid_indexes;
  for (auto& vec : unique_grid_indexes)
    vec.erase(std::unique(vec.begin(), vec.end()), vec.end());

  // Max size of step counts
  // Precated laser steps for each grid are less than this value
  // int max_count = g_resolution / search_step * sqrt(2) + 1;

  for (int i = 0; i < iangle_size; i++)
  {
    // Push grid indexes for each laser
    // Each index has weight which is decided by passing laser rate
    for (const auto& unique_index : unique_grid_indexes[i])
    {
      Grid g;
      g.index = unique_index;
      // g.weight = 1.0 * std::count(grid_indexes[i].begin(), grid_indexes[i].end(), unique_index) / max_count;
      g.calcCoordinate();
      g.calcRange();
      precasted_grids->at(i).push_back(g);
    }
  }
}


// Delete old cost values
void deleteOldData(std::vector<Cost>* cost_map, double current_x, double current_y, double prev_x, double prev_y)
{
  double begin_x = current_x;
  double begin_y = current_y;
  double end_x = prev_x;
  double end_y = prev_y;

  if (begin_x > end_x)
    std::swap(begin_x, end_x);
  if (begin_y > end_y)
    std::swap(begin_y, end_y);

  int ibegin_x = begin_x / g_resolution + g_scan_size_x / 2;
  int ibegin_y = begin_y / g_resolution + g_scan_size_y / 2;
  int iend_x = end_x / g_resolution + g_scan_size_x / 2;
  int iend_y = end_y / g_resolution + g_scan_size_y / 2;

  for (int i = ibegin_x; i < iend_x; i++)
  {
    for (int j = 0; j < g_scan_size_y; j++)
    {
      int global_grid_x = (i + 100000 * g_scan_size_x) % g_scan_size_x;
      cost_map->at(global_grid_x + j * g_scan_size_x).occupied = 0;
      cost_map->at(global_grid_x + j * g_scan_size_x).free     = 0;
      cost_map->at(global_grid_x + j * g_scan_size_x).unknown  = true;
    }
  }

  for (int i = 0; i < g_scan_size_x; i++)
  {
    for (int j = ibegin_y; j < iend_y; j++)
    {
      int global_grid_y = (j + 100000 * g_scan_size_y) % g_scan_size_y;
      cost_map->at(i + global_grid_y * g_scan_size_x).occupied = 0;
      cost_map->at(i + global_grid_y * g_scan_size_x).free     = 0;
      cost_map->at(i + global_grid_y * g_scan_size_x).unknown  = true;
    }
  }
}

void setOccupancyGridMap(nav_msgs::OccupancyGrid* map, const std_msgs::Header& header,
                         const tf::StampedTransform& transform)
{
  map->header.stamp = header.stamp;
  map->header.frame_id = OGM_FRAME;
  map->info.map_load_time = header.stamp;
  map->info.resolution = g_resolution;
  map->info.height = g_map_size_y;
  map->info.width = g_map_size_x;
  map->info.origin.position.x = transform.getOrigin().x() - (g_map_size_x / 2) * g_resolution;
  map->info.origin.position.y = transform.getOrigin().y() - (g_map_size_y / 2) * g_resolution;
  map->info.origin.position.z = transform.getOrigin().z() - 5;
  map->info.origin.orientation.x = 0;
  map->info.origin.orientation.y = 0;
  map->info.origin.orientation.z = 0;
  map->info.origin.orientation.w = 1;
}

void createCostMap(const sensor_msgs::LaserScan& scan, const std::vector<std::vector<Grid>>& precasted_grids)
{
  tf::StampedTransform transform;

  try
  {
    // What time should we use?
    g_tf_listenerp->lookupTransform(OGM_FRAME, g_sensor_frame, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Save costs in this variable
  static std::vector<Cost> cost_map(g_scan_size_x * g_scan_size_y);

  static bool initialized_map = false;
  static double prev_x = transform.getOrigin().x();
  static double prev_y = transform.getOrigin().y();
  static nav_msgs::OccupancyGrid map;
  setOccupancyGridMap(&map, scan.header, transform);
  if (!initialized_map)
  {
    map.data.resize(g_map_size_x * g_map_size_y, -1);
    initialized_map = true;
  }

  // Since we implement as ring buffer, we have to delete old data regularly
  deleteOldData(&cost_map, transform.getOrigin().x(), transform.getOrigin().y(), prev_x, prev_y);
  prev_x = transform.getOrigin().x();
  prev_y = transform.getOrigin().y();

  // Vehicle's orientation
  double yaw = calcYawFromQuaternion(transform.getRotation());

  // Original yaw is -PI ~ PI, so make its range 0 ~ 2PI
  if (yaw < 0)
    yaw += 2 * M_PI;
  static double laser_offset = fabs(scan.angle_min);

  // For tough_urg
  // static double manual_offset = -10.0 / 180 * M_PI + M_PI;
  // int index_offset = (yaw + laser_offset + manual_offset) / scan.angle_increment;

  int index_offset = (yaw + laser_offset) / scan.angle_increment;
  static int iangle_size = 2 * M_PI / scan.angle_increment;

  //----------------- RING OCCUPANCY GRID MAPPING --------------
  // Accumulate grid costs for each laser scan
  for (size_t i = 0; i < scan.ranges.size(); i++)
  {
    double range = scan.ranges[i];

    // Ignore 0 ranges
    if (range == 0)
      continue;

    // If laserscan does not reach objects, make a range max
    if (scan.ranges[i] > scan.range_max)
      range = scan.range_max;

    int precasted_index = (i + index_offset) % iangle_size;
    int obstacle_index = -1;
    for (const auto& g : precasted_grids[precasted_index])
    {
      obstacle_index++;

      if (g.range > range)
        break;

      // Free range
      int global_index = g.calcGlobalIndex(transform);
      cost_map[global_index].accumulateCost(0, FREE_INCREMENT);
    }

    // Obstacle
    int global_index = precasted_grids[precasted_index][obstacle_index].calcGlobalIndex(transform);
    cost_map[global_index].accumulateCost(OCCUPIED_INCREMENT, 0);

  }


  // Begining of grid index of publishing OGM
  static int origin_index_x = (g_scan_size_x - g_map_size_x) / 2.0;
  static int origin_index_y = (g_scan_size_y - g_map_size_y) / 2.0;
  // static int origin_index = origin_index_x + origin_index_y * g_scan_size_x;

  // Set cost values for publishing OccuppancyGridMap
  for (int i = 0; i < g_map_size_y; i++)
  {
    for (int j = 0; j < g_map_size_x; j++)
    {
      int scanmap_index_x = origin_index_x + j;
      int scanmap_index_y = origin_index_y + i;

      int global_index = calcGlobalIndex(scanmap_index_x, scanmap_index_y, transform);
      if (cost_map[global_index].unknown)
        map.data[j + i * g_map_size_x] = -1;
      else
        map.data[j + i * g_map_size_x] = (cost_map[global_index].occupied + 8) * 6;
    }
  }

  g_map_pub.publish(map);
}

// Make CostMap from LaserScan message
void laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  static bool precasted = false;
  static int iangle_size = 2 * M_PI / msg->angle_increment;
  static std::vector<std::vector<Grid>> precasted_grids(iangle_size);

  if (!precasted)
  {
    preCasting(*msg, &precasted_grids);
    precasted = true;
  }

  // Create costmap and publish
  createCostMap(*msg, precasted_grids);

  return;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserscan2costmap");

  tf::TransformListener tf_listener;
  g_tf_listenerp = &tf_listener;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<double>("resolution", g_resolution, 0.1);
  private_nh.param<int>("scan_size_x", g_scan_size_x, 1000);
  private_nh.param<int>("scan_size_y", g_scan_size_y, 1000);
  private_nh.param<int>("map_size_x", g_map_size_x, 500);
  private_nh.param<int>("map_size_y", g_map_size_y, 500);
  private_nh.param<std::string>("scan_topic", g_scan_topic, "/scan");
  private_nh.param<std::string>("sensor_frame", g_sensor_frame, "/velodyne");

  ros::Subscriber laserscan_sub = nh.subscribe(g_scan_topic, 1, laserScanCallback);

  g_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/ring_ogm", 1);

  ros::spin();

  return 0;
}
