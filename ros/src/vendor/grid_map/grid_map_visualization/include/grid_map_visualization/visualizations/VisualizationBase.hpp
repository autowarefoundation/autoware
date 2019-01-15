/*
 * VisualizationBase.hpp
 *
 *  Created on: Mar 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map_core/GridMap.hpp>

// ROS
#include <ros/ros.h>

namespace grid_map_visualization {

typedef std::map<std::string, XmlRpc::XmlRpcValue> StringMap;

class VisualizationBase
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param name the name of the visualization.
   */
  VisualizationBase(ros::NodeHandle& nodeHandle, const std::string& name);

  /*!
   * Destructor.
   */
  virtual ~VisualizationBase();

  /*!
   * Read parameters from ROS.
   * @param config the parameters as XML.
   * @return true if successful.
   */
  virtual bool readParameters(XmlRpc::XmlRpcValue& config);

  /*!
   * Initialization.
   */
  virtual bool initialize() = 0;

  /*!
   * Generates the visualization.
   * @param map the grid map to visualize.
   * @return true if successful.
   */
  virtual bool visualize(const grid_map::GridMap& map) = 0;

  /*!
   * Checks if visualization is active (if somebody has actually subscribed).
   * @return true if active, false otherwise.
   */
  bool isActive() const;

 protected:

  /*!
   * Get a visualization parameter as a string.
   * @param[in] name the name of the parameter
   * @param[out] value the string to set with the value.
   * @return true if parameter was found, false otherwise.
   */
  bool getParam(const std::string& name, std::string& value);

  /*!
   * Get a visualization parameter as a double.
   * @param[in] name the name of the parameter
   * @param[out] value the double to set with the value.
   * @return true if parameter was found, false otherwise.
   */
  bool getParam(const std::string& name, double& value);

  /*!
   * Get a visualization parameter as a float.
   * @param[in] name the name of the parameter
   * @param[out] value the float to set with the value.
   * @return true if parameter was found, false otherwise.
   */
  bool getParam(const std::string& name, float& value);

  /*!
   * Get a visualization parameter as an integer.
   * @param[in] name the name of the parameter
   * @param[out] value the int to set with the value.
   * @return true if parameter was found, false otherwise.
   */
  bool getParam(const std::string&name, int& value);

  /*!
   * Get a visualization parameter as a boolean.
   * @param[in] name the name of the parameter
   * @param[out] value the boolean to set with the value.
   * @return true if parameter was found, false otherwise.
   */
  bool getParam(const std::string& name, bool& value);

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Name of the visualization.
  std::string name_;

  //! Storage of the parsed XML parameters.
  StringMap parameters_;

  //! ROS publisher of the occupancy grid.
  ros::Publisher publisher_;
};

} /* namespace */
