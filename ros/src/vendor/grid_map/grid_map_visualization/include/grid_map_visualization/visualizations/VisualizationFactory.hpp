/*
 * VisualizationFactory.hpp
 *
 *  Created on: Mar 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map_visualization/visualizations/VisualizationBase.hpp>
#include <vector>
#include <string>
#include <memory>

namespace grid_map_visualization {

class VisualizationFactory
{
 public:
  VisualizationFactory(ros::NodeHandle& nodeHandle);
  virtual ~VisualizationFactory();

  bool isValidType(const std::string& type);
  std::shared_ptr<VisualizationBase> getInstance(const std::string& type, const std::string& name);

 private:
  ros::NodeHandle& nodeHandle_;
  std::vector<std::string> types_;
};

} /* namespace */
