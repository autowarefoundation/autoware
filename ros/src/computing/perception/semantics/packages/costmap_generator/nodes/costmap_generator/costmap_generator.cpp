/*
 *  Copyright (c) 2018, Nagoya University
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
 ********************/

// headers in local directory
#include "object_map/object_map_utils.hpp"
#include "costmap_generator/costmap_generator.h"

// Constructor
CostmapGenerator::CostmapGenerator()
  : private_nh_("~")
  , has_subscribed_wayarea_(false)
  , OBJECTS_BOX_COSTMAP_LAYER_("objects_box")
  , OBJECTS_CONVEX_HULL_COSTMAP_LAYER_("objects_convex_hull")
  , SENSOR_POINTS_COSTMAP_LAYER_("sensor_points")
  , VECTORMAP_COSTMAP_LAYER_("vectormap")
  , COMBINED_COSTMAP_LAYER_("costmap")
{
}

CostmapGenerator::~CostmapGenerator()
{
}

void CostmapGenerator::init()
{
  private_nh_.param<std::string>("lidar_frame", lidar_frame_, "velodyne");
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<double>("grid_min_value", grid_min_value_, 0.0);
  private_nh_.param<double>("grid_max_value", grid_max_value_, 1.0);
  private_nh_.param<double>("grid_resolution", grid_resolution_, 0.2);
  private_nh_.param<double>("grid_length_x", grid_length_x_, 50);
  private_nh_.param<double>("grid_length_y", grid_length_y_, 30);
  private_nh_.param<double>("grid_position_x", grid_position_x_, 20);
  private_nh_.param<double>("grid_position_y", grid_position_y_, 0);
  private_nh_.param<double>("maximum_lidar_height_thres", maximum_lidar_height_thres_, 0.3);
  private_nh_.param<double>("minimum_lidar_height_thres", minimum_lidar_height_thres_, -2.2);
  private_nh_.param<bool>("use_objects_box", use_objects_box_, false);
  private_nh_.param<bool>("use_objects_convex_hull", use_objects_convex_hull_, true);
  private_nh_.param<bool>("use_points", use_points_, true);
  private_nh_.param<bool>("use_wayarea", use_wayarea_, true);
  private_nh_.param<double>("expand_polygon_size", expand_polygon_size_, 1.0);
  private_nh_.param<int>("size_of_expansion_kernel", size_of_expansion_kernel_, 9);

  initGridmap();
}

void CostmapGenerator::run()
{
  pub_costmap_ = nh_.advertise<grid_map_msgs::GridMap>("/semantics/costmap", 1);
  pub_occupancy_grid_ = nh_.advertise<nav_msgs::OccupancyGrid>("/semantics/costmap_generator/occupancy_grid", 1);

  if (use_objects_box_ || use_objects_convex_hull_)
  {
    sub_objects_ = nh_.subscribe("/prediction/motion_predictor/objects", 1, &CostmapGenerator::objectsCallback, this);
  }

  if (use_points_)
  {
    sub_points_ = nh_.subscribe("/points_no_ground", 1, &CostmapGenerator::sensorPointsCallback, this);
  }
}

void CostmapGenerator::objectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects)
{
  if(use_objects_box_)
  {
    const bool use_convex_hull = !use_objects_box_;
    costmap_[OBJECTS_BOX_COSTMAP_LAYER_] = generateObjectsCostmap(in_objects, use_convex_hull);
  }

  if(use_objects_convex_hull_)
  {
    costmap_[OBJECTS_CONVEX_HULL_COSTMAP_LAYER_] = generateObjectsCostmap(in_objects, use_objects_convex_hull_);
  }
  costmap_[VECTORMAP_COSTMAP_LAYER_] = generateVectormapCostmap();
  costmap_[COMBINED_COSTMAP_LAYER_] = generateCombinedCostmap();

  std_msgs::Header in_header = in_objects->header;
  publishRosMsg(costmap_, in_header);
}

void CostmapGenerator::sensorPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_points_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_sensor_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*in_sensor_points_msg, *in_sensor_points);
  costmap_[SENSOR_POINTS_COSTMAP_LAYER_] = generateSensorPointsCostmap(in_sensor_points);
  costmap_[VECTORMAP_COSTMAP_LAYER_] = generateVectormapCostmap();
  costmap_[COMBINED_COSTMAP_LAYER_] = generateCombinedCostmap();

  std_msgs::Header in_header = in_sensor_points_msg->header;
  publishRosMsg(costmap_, in_header);
}

void CostmapGenerator::initGridmap()
{
  costmap_.setFrameId(lidar_frame_);
  costmap_.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_), grid_resolution_,
                       grid_map::Position(grid_position_x_, grid_position_y_));

  costmap_.add(SENSOR_POINTS_COSTMAP_LAYER_, grid_min_value_);
  costmap_.add(OBJECTS_BOX_COSTMAP_LAYER_, grid_min_value_);
  costmap_.add(OBJECTS_CONVEX_HULL_COSTMAP_LAYER_, grid_min_value_);
  costmap_.add(VECTORMAP_COSTMAP_LAYER_, grid_min_value_);
  costmap_.add(COMBINED_COSTMAP_LAYER_, grid_min_value_);
}

grid_map::Matrix
CostmapGenerator::generateSensorPointsCostmap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_sensor_points)
{
  grid_map::Matrix sensor_points_costmap = points2costmap_.makeCostmapFromSensorPoints(
      maximum_lidar_height_thres_, minimum_lidar_height_thres_, grid_min_value_, grid_max_value_, costmap_,
      SENSOR_POINTS_COSTMAP_LAYER_, in_sensor_points);
  return sensor_points_costmap;
}

grid_map::Matrix
CostmapGenerator::generateObjectsCostmap(const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects,
                                         const bool use_convex_hull)
{
  grid_map::Matrix objects_costmap = objects2costmap_.makeCostmapFromObjects(
    costmap_, expand_polygon_size_, size_of_expansion_kernel_, in_objects, use_convex_hull);
  return objects_costmap;
}

// Only this funstion depends on object_map_utils
grid_map::Matrix CostmapGenerator::generateVectormapCostmap()
{
  grid_map::GridMap vectormap_costmap = costmap_;
  if (use_wayarea_)
  {
    if (!has_subscribed_wayarea_)
    {
      object_map::LoadRoadAreasFromVectorMap(private_nh_, area_points_);
    }
    if (!area_points_.empty())
    {
      has_subscribed_wayarea_ = true;
      object_map::FillPolygonAreas(vectormap_costmap, area_points_, VECTORMAP_COSTMAP_LAYER_, grid_max_value_,
                                   grid_min_value_, grid_min_value_, grid_max_value_, lidar_frame_, map_frame_,
                                   tf_listener_);
    }
  }
  return vectormap_costmap[VECTORMAP_COSTMAP_LAYER_];
}

grid_map::Matrix CostmapGenerator::generateCombinedCostmap()
{
  // assuming combined_costmap is calculated by element wise max operation
  grid_map::GridMap combined_costmap = costmap_;
  combined_costmap[COMBINED_COSTMAP_LAYER_].setConstant(grid_min_value_);
  combined_costmap[COMBINED_COSTMAP_LAYER_] =
      combined_costmap[COMBINED_COSTMAP_LAYER_].cwiseMax(combined_costmap[SENSOR_POINTS_COSTMAP_LAYER_]);
  combined_costmap[COMBINED_COSTMAP_LAYER_] =
      combined_costmap[COMBINED_COSTMAP_LAYER_].cwiseMax(combined_costmap[VECTORMAP_COSTMAP_LAYER_]);
  combined_costmap[COMBINED_COSTMAP_LAYER_] =
      combined_costmap[COMBINED_COSTMAP_LAYER_].cwiseMax(combined_costmap[OBJECTS_BOX_COSTMAP_LAYER_]);
  combined_costmap[COMBINED_COSTMAP_LAYER_] =
      combined_costmap[COMBINED_COSTMAP_LAYER_].cwiseMax(combined_costmap[OBJECTS_CONVEX_HULL_COSTMAP_LAYER_]);
  return combined_costmap[COMBINED_COSTMAP_LAYER_];
}

void CostmapGenerator::publishRosMsg(const grid_map::GridMap& costmap, const std_msgs::Header& in_header)
{
  nav_msgs::OccupancyGrid out_occupancy_grid;
  grid_map::GridMapRosConverter::toOccupancyGrid(costmap, COMBINED_COSTMAP_LAYER_, grid_min_value_, grid_max_value_,
                                                 out_occupancy_grid);
  out_occupancy_grid.header = in_header;
  pub_occupancy_grid_.publish(out_occupancy_grid);

  grid_map_msgs::GridMap out_gridmap_msg;
  grid_map::GridMapRosConverter::toMessage(costmap, out_gridmap_msg);
  out_gridmap_msg.info.header = in_header;
  pub_costmap_.publish(out_gridmap_msg);
}
