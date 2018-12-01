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

#include "costmap_generator.h"


// Constructor
CostmapGenerator::CostmapGenerator() :
  use_vectormap_(true),
  use_wayarea_(true),
  use_waypoint_(true),
  use_objects_(true),
  private_nh_("~")
{
}

CostmapGenerator::~CostmapGenerator() {}

void CostmapGenerator::init()
{
  private_nh_.param<std::string>("sensor_frame", sensor_frame_, "velodyne");
  private_nh_.param<double>("grid_resolution", grid_resolution_, 0.2);
  private_nh_.param<double>("grid_length_x", grid_length_x_, 50);
  private_nh_.param<double>("grid_length_y", grid_length_y_, 30);
  private_nh_.param<double>("grid_position_x", grid_position_x_, 20);
  private_nh_.param<double>("grid_position_x", grid_position_y_, 0);
  initGridmap();
}

void CostmapGenerator::run()
{
  pub_costmap_ = nh_.advertise<grid_map_msgs::GridMap>("/semantics/costmap", 1);

  if(use_vectormap_)
  {
    registerVectormapSubscriber();
  }

  if(use_waypoint_)
  {
    sub_waypoint_ = nh_.subscribe("/based_waypoints", 1, &CostmapGenerator::waypointCallback, this);
  }

  if(!use_objects_)
  {
    sub_points_ = nh_.subscribe("/poitns_no_ground", 1, &CostmapGenerator::sensorPointsCallback, this);
  }
  else
  {
    registerSyncedSubscriber();
  }
}

void CostmapGenerator::syncedCallback(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_points,
                                const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects)
{
  // if(checkSubscripton())
  // generateSensorPointsCostmap()
  // generateObjectsCostmap()
  // generateMapPoints()
  // generateVectormapCostmap()
  // generateWaypointCostmap()
  // generateCombinedCostmap()
  // pub(costmap)
}

void CostmapGenerator::sensorPointsCallback(const sensor_msgs::PointCloud2& in_sensor_points_msg)
{
  // if(checkSubscripton())
  // gridmap_ = generateSensorPointsCostmap(in_sensor_points_msg);
  // gridmap_ = generateMapPointsCostmap()
  // gridmap_ = generateVectormapCostmap()
  // gridmap_ = generateWaypointCostmap()
  // gridmap_ = generateCombinedCostmap()
  // pub(gridmap_);
}

// void CostmapGenerator::mapPointsCallback(const sensor_msgs::PointCloud2& in_map_points)
// {
// }

void CostmapGenerator::waypointCallback(const autoware_msgs::LaneArray& in_waypoint)
{

}


void CostmapGenerator::registerSyncedSubscriber()
{
  sub_sync_points_ptr_.reset(
    new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/points_no_ground", 1));
  sub_sync_objects_ptr_.reset(
    new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(nh_, "/detection/lidar_tracker/objects", 1));
  sync_ptr_.reset(
    new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,
    autoware_msgs::DetectedObjectArray>
    (*sub_sync_points_ptr_, *sub_sync_objects_ptr_, 10));

  sync_ptr_->registerCallback(boost::bind(&CostmapGenerator::syncedCallback, this, _1, _2));
}

void CostmapGenerator::registerVectormapSubscriber()
{
  if(use_wayarea_)
  {
    vmap_.subscribe(private_nh_, vector_map::Category::POINT |
                                vector_map::Category::LINE |
                                vector_map::Category::AREA |
                                vector_map::Category::WAY_AREA, 1);
  }
  else
  {
    //TODO need to subscribe appropriate vectormap info
    vmap_.subscribe(private_nh_, vector_map::Category::POINT |
                                 vector_map::Category::NODE  |
                                 vector_map::Category::LANE, 1);

  }
}

void CostmapGenerator::initGridmap()
{
  gridmap_.add("sensor_points_cost", 0);
  gridmap_.add("vectormap_cost", 0);
  gridmap_.add("waypoint_cost", 0);
  gridmap_.add("objects_cost", 0);
  gridmap_.setFrameId(sensor_frame_);
  gridmap_.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_),
                  grid_resolution_,
                  grid_map::Position(grid_position_x_, grid_position_y_));
}

grid_map::GridMap CostmapGenerator::generateSensorPointsCostmap(const sensor_msgs::PointCloud2& in_sensor_points_msg)
{
  std::string sensor_points_costmap_layer_name = "sensor_points_cost";
  grid_map::GridMap gridmap_with_points_cost = points2costmap_.makeSensorPointsCostmap(gridmap_,
                                                                sensor_points_costmap_layer_name, in_sensor_points_msg);
  return gridmap_with_points_cost;
}
