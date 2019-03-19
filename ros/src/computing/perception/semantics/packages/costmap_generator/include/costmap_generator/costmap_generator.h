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

#ifndef COSTMAP_GENERATOR_H
#define COSTMAP_GENERATOR_H

// headers in ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>

// headers in local directory
#include "vector_map/vector_map.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "points_to_costmap.h"
#include "objects_to_costmap.h"

// headers in STL
#include <memory>

class CostmapGenerator
{
public:
  CostmapGenerator();
  ~CostmapGenerator();

  void init();
  void run();

private:
  friend class TestClass;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  bool use_objects_box_;
  bool use_objects_convex_hull_;
  bool use_points_;
  bool use_wayarea_;

  bool has_subscribed_wayarea_;

  std::string lidar_frame_;
  std::string map_frame_;
  double grid_min_value_;
  double grid_max_value_;
  double grid_resolution_;
  double grid_length_x_;
  double grid_length_y_;
  double grid_position_x_;
  double grid_position_y_;

  double maximum_lidar_height_thres_;
  double minimum_lidar_height_thres_;

  double expand_polygon_size_;
  int size_of_expansion_kernel_;

  vector_map::VectorMap vmap_;

  grid_map::GridMap costmap_;

  ros::Publisher pub_costmap_;
  ros::Publisher pub_occupancy_grid_;
  ros::Subscriber sub_waypoint_;
  ros::Subscriber sub_points_;
  ros::Subscriber sub_objects_;

  tf::TransformListener tf_listener_;

  std::vector<std::vector<geometry_msgs::Point>> area_points_;

  PointsToCostmap points2costmap_;
  ObjectsToCostmap objects2costmap_;

  const std::string OBJECTS_BOX_COSTMAP_LAYER_;
  const std::string OBJECTS_CONVEX_HULL_COSTMAP_LAYER_;
  const std::string SENSOR_POINTS_COSTMAP_LAYER_;
  const std::string VECTORMAP_COSTMAP_LAYER_;
  const std::string COMBINED_COSTMAP_LAYER_;

  /// \brief callback for DetectedObjectArray
  /// \param[in] in_objects input DetectedObjectArray usually from prediction or perception component
  void objectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& in_ojects);

  /// \brief callback for sensor_msgs::PointCloud2
  /// \param[in] in_sensor_points input sensot_msgs::PointCloud2. Assuming groud-fitered pointcloud by default
  void sensorPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_points);

  /// \brief initialize gridmap parameters based on rosparam
  void initGridmap();

  /// \brief publish ros msg: /semantics/costmap (grid_map::GridMap),
  ///                         /semantics/costmap_generator/occupancy_grid(nav_msgs::OccupancyGrid)
  /// \param[in] gridmap with calculated cost
  /// \param[in] input ros header
  void publishRosMsg(const grid_map::GridMap& gridmap, const std_msgs::Header& in_header);

  /// \brief calculate cost from pointcloud data
  /// \param[in] in_sensor_points: subscribed pointcloud data
  grid_map::Matrix generateSensorPointsCostmap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_sensor_points);

  /// \brief calculate cost from DetectedObjectArray
  /// \param[in] in_objects: subscribed DetectedObjectArray
  grid_map::Matrix generateObjectsCostmap(const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects,
                                          const bool use_objects_convex_hull);

  /// \brief calculate cost from vectormap
  grid_map::Matrix generateVectormapCostmap();

  /// \brief calculate cost for final output
  grid_map::Matrix generateCombinedCostmap();
};

#endif  // COSTMAP_GENERATOR_H
