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
#include "object_map_utils.hpp"
#include "costmap_generator.h"


// Constructor
CostmapGenerator::CostmapGenerator() :
  private_nh_("~"),
  has_subscribed_wayarea_(false),
  OBJECTS_COSTMAP_LAYER_("objects"),
  SENSOR_POINTS_COSTMAP_LAYER_("sensor_points"),
  VECTORMAP_COSTMAP_LAYER_("vectormap"),
  WAYPOINTS_COSTMAP_LAYER_("waypoints"),
  COMBINED_COSTMAP_LAYER_("costmap")
{
}

CostmapGenerator::~CostmapGenerator() {}

void CostmapGenerator::init()
{
  //TODO sample sompo rosbga has weird velodyne coordinate. so be carefult about default grid length and position
  private_nh_.param<std::string>("velodyne_frame", velodyne_frame_, "velodyne");
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<double>("grid_min_value", grid_min_value_, 0.0);
  private_nh_.param<double>("grid_max_value", grid_max_value_, 1.0);
  private_nh_.param<double>("grid_resolution", grid_resolution_, 0.2);
  private_nh_.param<double>("grid_length_x", grid_length_x_, 30);
  private_nh_.param<double>("grid_length_y", grid_length_y_, 50);
  private_nh_.param<double>("grid_position_x", grid_position_x_, 0);
  private_nh_.param<double>("grid_position_y", grid_position_y_, 20);
  private_nh_.param<double>("maximum_lidar_height_thres", maximum_lidar_height_thres_, 0.3);
  private_nh_.param<double>("minimum_lidar_height_thres", minimum_lidar_height_thres_, -2.2);
  private_nh_.param<bool>("use_objects", use_objects_, true);
  private_nh_.param<bool>("use_points", use_points_, true);
  private_nh_.param<bool>("use_wayarea", use_wayarea_, true);
  private_nh_.param<double>("expand_rectangle_size", expand_rectangle_size_, 1.5);
  private_nh_.param<int>("size_of_expansion_kernel", size_of_expansion_kernel_, 9);
  // private_nh_.param<bool>("use_waypoints_", use_waypoints_, true);

  initGridmap();
}

void CostmapGenerator::run()
{
  pub_costmap_ = nh_.advertise<grid_map_msgs::GridMap>("/semantics/costmap", 1);
  // pub_sensor_points_cost_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/semantics/costmap_generator/sensor_points_cost_cloud", 1);
  // pub_objects_cost_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/semantics/costmap_generator/objects_cost_cloud", 1);
  // pub_vectormap_cost_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/semantics/costmap_generator/vectormap_cost_cloud", 1);
  // pub_occupancy_grid_ = nh_.advertise<nav_msgs::OccupancyGrid>("/semantics/costmap_generator/occupancy_grid", 1);

  if(use_objects_)
  {
    sub_objects_ = nh_.subscribe("/prediction/motion_predictor/objects", 1, &CostmapGenerator::objectsCallback, this);
  }

  if(use_points_)
  {
    sub_points_ = nh_.subscribe("/points_no_ground", 1, &CostmapGenerator::sensorPointsCallback, this);
  }

  // if(use_waypoints_)
  // {
  //   sub_waypoint_ = nh_.subscribe("/based_waypoints", 1, &CostmapGenerator::waypointsCallback, this);
  // }
}

void CostmapGenerator::objectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects)
{
  costmap_[OBJECTS_COSTMAP_LAYER_] = generateObjectsCostmap(in_objects);
  costmap_[VECTORMAP_COSTMAP_LAYER_] = generateVectormapCostmap();
  costmap_[COMBINED_COSTMAP_LAYER_] = generateCombinedCostmap();

  publishRosMsg(costmap_);
}

void CostmapGenerator::sensorPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_points_msg)
{
  costmap_[SENSOR_POINTS_COSTMAP_LAYER_] = generateSensorPointsCostmap(in_sensor_points_msg);
  costmap_[VECTORMAP_COSTMAP_LAYER_] = generateVectormapCostmap();
  costmap_[COMBINED_COSTMAP_LAYER_] = generateCombinedCostmap();

  publishRosMsg(costmap_);
}

// void CostmapGenerator::waypointsCallback(const autoware_msgs::LaneArray::ConstPtr& in_waypoint)
// {
  // costmap_ = generateWaypointsCostmap();
  // costmap_ = generateCombinedCostmap();
  // publishRosMsg(costmap_);
// }

void CostmapGenerator::initGridmap()
{
  costmap_.setFrameId(velodyne_frame_);
  costmap_.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_),
                  grid_resolution_,
                  grid_map::Position(grid_position_x_, grid_position_y_));

  costmap_.add(SENSOR_POINTS_COSTMAP_LAYER_, grid_min_value_);
  costmap_.add(OBJECTS_COSTMAP_LAYER_, grid_min_value_);
  costmap_.add(VECTORMAP_COSTMAP_LAYER_, grid_min_value_);
  // costmap_.add("waypoint_cost", 0);
  costmap_.add(COMBINED_COSTMAP_LAYER_, grid_min_value_);
}

grid_map::Matrix CostmapGenerator::generateSensorPointsCostmap(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_points_msg)
{
  // TODO: check tf coordinate. If assuming that make costmap in velodyne coordinate, it is not necessary
  // TODO: rename makeCostmapFromSensorPoints
  grid_map::Matrix sensor_points_costmap =
                        points2costmap_.makeCostmapFromSensorPoints(maximum_lidar_height_thres_, minimum_lidar_height_thres_,
                                                                    grid_min_value_, grid_max_value_,
                                                          costmap_, SENSOR_POINTS_COSTMAP_LAYER_, in_sensor_points_msg);
  return sensor_points_costmap;
}

grid_map::Matrix CostmapGenerator::generateObjectsCostmap(const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects)
{
  // TODO: check tf coordinate. If assuming that make costmap in velodyne coordinate, it is not necessary
  grid_map::Matrix objects_costmap =
                        objects2costmap_.makeCostmapFromObjects(costmap_,
                                                                OBJECTS_COSTMAP_LAYER_,
                                                                expand_rectangle_size_,
                                                                size_of_expansion_kernel_,
                                                                in_objects);
  return objects_costmap;
}

// grid_map::Matrix CostmapGenerator::generateWaypointsCostmap(const autoware_msgs::LaneArray::ConstPtr& in_waypoints)
// {
//   grid_map::Matrix waypoints_costmap =
//                         waypoints2costmap_.makeCostmapFromWaypoints(costmap_, WAYPOINTS_COSTMAP_LAYER_, in_waypoints);
//   return waypoints_costmap;
// }


// only this funstion depends on object_map_utils
grid_map::Matrix CostmapGenerator::generateVectormapCostmap()
{
  grid_map::GridMap vectormap_costmap = costmap_;
  if(use_wayarea_)
  {
    if(!has_subscribed_wayarea_)
    {
      object_map::LoadRoadAreasFromVectorMap(private_nh_, area_points_);
    }
    if(!area_points_.empty())
    {
      has_subscribed_wayarea_ = true;
      object_map::FillPolygonAreas(vectormap_costmap, area_points_, VECTORMAP_COSTMAP_LAYER_,
                        grid_max_value_, grid_min_value_, grid_min_value_,
                       grid_max_value_, velodyne_frame_, map_frame_,
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
  combined_costmap[COMBINED_COSTMAP_LAYER_] = combined_costmap[COMBINED_COSTMAP_LAYER_].cwiseMax(
                                              combined_costmap[SENSOR_POINTS_COSTMAP_LAYER_]);
  combined_costmap[COMBINED_COSTMAP_LAYER_] = combined_costmap[COMBINED_COSTMAP_LAYER_].cwiseMax(
                                              combined_costmap[VECTORMAP_COSTMAP_LAYER_]);
  combined_costmap[COMBINED_COSTMAP_LAYER_] = combined_costmap[COMBINED_COSTMAP_LAYER_].cwiseMax(
                                              combined_costmap[OBJECTS_COSTMAP_LAYER_]);
  return combined_costmap[COMBINED_COSTMAP_LAYER_];
}

void CostmapGenerator::publishRosMsg(const grid_map::GridMap& costmap)
{
  std::cout << "pub?" << std::endl;
  // if(use_points_)
  // {
  //   sensor_msgs::PointCloud2 out_sensor_points_cost_cloud_msg;
  //   grid_map::GridMapRosConverter::toPointCloud(costmap, SENSOR_POINTS_COSTMAP_LAYER_, out_sensor_points_cost_cloud_msg);
  //   pub_sensor_points_cost_cloud_.publish(out_sensor_points_cost_cloud_msg);
  // }
  //
  // if(use_objects_)
  // {
  //   sensor_msgs::PointCloud2 out_objects_cost_cloud_msg;
  //   grid_map::GridMapRosConverter::toPointCloud(costmap, OBJECTS_COSTMAP_LAYER_, out_objects_cost_cloud_msg);
  //   pub_objects_cost_cloud_.publish(out_objects_cost_cloud_msg);
  // }
  //
  // if(use_wayarea_)
  // {
  //   sensor_msgs::PointCloud2 out_vectormap_cost_cloud_msg;
  //   grid_map::GridMapRosConverter::toPointCloud(costmap, VECTORMAP_COSTMAP_LAYER_, out_vectormap_cost_cloud_msg);
  //   pub_vectormap_cost_cloud_.publish(out_vectormap_cost_cloud_msg);
  // }

  // nav_msgs::OccupancyGrid out_occupancy_grid;
  // grid_map::GridMapRosConverter::toOccupancyGrid(costmap, COMBINED_COSTMAP_LAYER_, grid_min_value_, grid_max_value_,
  //                                         out_occupancy_grid);
  // pub_occupancy_grid_.publish(out_occupancy_grid);

  grid_map_msgs::GridMap out_gridmap_msg;
  grid_map::GridMapRosConverter::toMessage(costmap, out_gridmap_msg);
  pub_costmap_.publish(out_gridmap_msg);
}
