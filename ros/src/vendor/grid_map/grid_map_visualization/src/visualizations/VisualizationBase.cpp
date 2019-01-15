/*
 * VisualizationBase.cpp
 *
 *  Created on: Mar 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_visualization/visualizations/VisualizationBase.hpp"

namespace grid_map_visualization {

VisualizationBase::VisualizationBase(ros::NodeHandle& nodeHandle, const std::string& name)
    : nodeHandle_(nodeHandle),
      name_(name)
{
}

VisualizationBase::~VisualizationBase()
{
}

bool VisualizationBase::isActive() const
{
  if (publisher_.getNumSubscribers() > 0) return true;
  return false;
}

bool VisualizationBase::readParameters(XmlRpc::XmlRpcValue& config)
{
  if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR("A filter configuration must be a map with fields name, type, and params.");
    return false;
  }

  // Check to see if we have parameters in our list.
  if (config.hasMember("params")) {
    XmlRpc::XmlRpcValue params = config["params"];
    if (params.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR("Params must be a map.");
      return false;
    } else {
      for (XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it) {
        ROS_DEBUG("Loading param %s\n", it->first.c_str());
        parameters_[it->first] = it->second;
      }
    }
  }

  return true;
}

bool VisualizationBase::getParam(const std::string& name, std::string& value)
{
  StringMap::iterator it = parameters_.find(name);
  if (it == parameters_.end()) return false;
  if (it->second.getType() != XmlRpc::XmlRpcValue::TypeString) return false;
  value = std::string(it->second);
  return true;
}

bool VisualizationBase::getParam(const std::string& name, double& value)
{
  StringMap::iterator it = parameters_.find(name);
  if (it == parameters_.end()) return false;
  if (it->second.getType() != XmlRpc::XmlRpcValue::TypeDouble
      && it->second.getType() != XmlRpc::XmlRpcValue::TypeInt) return false;
  value = it->second.getType() == XmlRpc::XmlRpcValue::TypeInt ?
          (int) (it->second) : (double) (it->second);
  return true;
}

bool VisualizationBase::getParam(const std::string& name, float& value)
{
  double valueDouble;
  bool isSuccess = getParam(name, valueDouble);
  if (isSuccess) value = static_cast<float>(valueDouble);
  return isSuccess;
}

bool VisualizationBase::getParam(const std::string& name, bool& value)
{
  StringMap::iterator it = parameters_.find(name);
  if (it == parameters_.end()) return false;
  if (it->second.getType() != XmlRpc::XmlRpcValue::TypeBoolean) return false;
  value = it->second;
  return true;
}

bool VisualizationBase::getParam(const std::string&name, int& value)
{
  StringMap::iterator it = parameters_.find(name);
  if (it == parameters_.end()) return false;
  if(it->second.getType() != XmlRpc::XmlRpcValue::TypeInt) return false;
  value = it->second;
  return true;
}

} /* namespace */
