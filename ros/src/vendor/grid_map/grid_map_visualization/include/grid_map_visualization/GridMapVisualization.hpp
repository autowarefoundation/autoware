/*
 * GridMapVisualization.hpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#pragma once

#include <grid_map_msgs/GridMap.h>
#include <grid_map_visualization/visualizations/VisualizationFactory.hpp>
#include <grid_map_visualization/visualizations/MapRegionVisualization.hpp>
#include <grid_map_visualization/visualizations/PointCloudVisualization.hpp>
#include <grid_map_visualization/visualizations/VectorVisualization.hpp>
#include <grid_map_visualization/visualizations/OccupancyGridVisualization.hpp>

// ROS
#include <ros/ros.h>

// STD
#include <vector>
#include <memory>

namespace grid_map_visualization {

/*!
 * Visualizes a grid map by publishing different topics that can be viewed in Rviz.
 */
class GridMapVisualization
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  GridMapVisualization(ros::NodeHandle& nodeHandle, const std::string& parameterName);

  /*!
   * Destructor.
   */
  virtual ~GridMapVisualization();

  /*!
   * Callback function for the grid map.
   * @param message the grid map message to be visualized.
   */
  void callback(const grid_map_msgs::GridMap& message);

 private:

  /*!
   * Read parameters from ROS.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initialization.
   * @return true if successful.
   */
  bool initialize();

  /*!
   * Check if visualizations are active (subscribed to),
   * and accordingly cancels/activates the subscription to the
   * grid map to safe bandwidth.
   * @param timerEvent the timer event.
   */
  void updateSubscriptionCallback(const ros::TimerEvent& timerEvent);

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Parameter name of the visualizer configuration list.
  std::string visualizationsParameter_;

  //! ROS subscriber to the grid map.
  ros::Subscriber mapSubscriber_;

  //! Topic name of the grid map to be visualized.
  std::string mapTopic_;

  //! List of visualizations.
  std::vector<std::shared_ptr<VisualizationBase>> visualizations_;

  //! Visualization factory.
  VisualizationFactory factory_;

  //! Timer to check the activity of the visualizations.
  ros::Timer activityCheckTimer_;

  //! Duration of checking the activity of the visualizations.
  ros::Duration activityCheckDuration_;

  //! If the grid map visualization is subscribed to the grid map.
  bool isSubscribed_;
};

} /* namespace */
