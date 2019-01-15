/*
 * VectorVisualization.hpp
 *
 *  Created on: Sep 16, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map_visualization/visualizations/VisualizationBase.hpp>
#include <grid_map_core/GridMap.hpp>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

// STD
#include <vector>

namespace grid_map_visualization {

/*!
 * Visualization a combination of three layers of the grid map as a vector field.
 */
class VectorVisualization : public VisualizationBase
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param name the name of the visualization.
   */
  VectorVisualization(ros::NodeHandle& nodeHandle, const std::string& name);

  /*!
   * Destructor.
   */
  virtual ~VectorVisualization();

  /*!
   * Read parameters from ROS.
   * @param config the parameters as XML.
   * @return true if successful.
   */
  bool readParameters(XmlRpc::XmlRpcValue& config);

  /*!
   * Initialization.
   */
  bool initialize();

  /*!
   * Generates the visualization.
   * @param map the grid map to visualize.
   * @return true if successful.
   */
  bool visualize(const grid_map::GridMap& map);

 private:

  //! Marker to be published.
  visualization_msgs::Marker marker_;

  //! Types that are transformed to vectors.
  std::vector<std::string> types_;

  //! Type that is the position of the vectors.
  std::string positionLayer_;

  //! Scaling of the vectors.
  double scale_;

  //! Width of the line markers [m].
  double lineWidth_;

  //! Color of the vectors.
  std_msgs::ColorRGBA color_;
};

} /* namespace */
