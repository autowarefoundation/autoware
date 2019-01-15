/*
 * GridMapVisualization.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_visualization/GridMapVisualization.hpp"
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

using namespace std;
using namespace ros;

namespace grid_map_visualization {

GridMapVisualization::GridMapVisualization(ros::NodeHandle& nodeHandle,
                                           const std::string& parameterName)
    : nodeHandle_(nodeHandle),
      visualizationsParameter_(parameterName),
      factory_(nodeHandle_),
      isSubscribed_(false)
{
  ROS_INFO("Grid map visualization node started.");
  readParameters();
  activityCheckTimer_ = nodeHandle_.createTimer(activityCheckDuration_,
                                                &GridMapVisualization::updateSubscriptionCallback,
                                                this);
  initialize();
}

GridMapVisualization::~GridMapVisualization()
{
}

bool GridMapVisualization::readParameters()
{
  nodeHandle_.param("grid_map_topic", mapTopic_, string("/grid_map"));

  double activityCheckRate;
  nodeHandle_.param("activity_check_rate", activityCheckRate, 2.0);
  activityCheckDuration_.fromSec(1.0 / activityCheckRate);
  ROS_ASSERT(!activityCheckDuration_.isZero());

  // Configure the visualizations from a configuration stored on the parameter server.
  XmlRpc::XmlRpcValue config;
  if (!nodeHandle_.getParam(visualizationsParameter_, config)) {
    ROS_WARN(
        "Could not load the visualizations configuration from parameter %s,are you sure it"
        "was pushed to the parameter server? Assuming that you meant to leave it empty.",
        visualizationsParameter_.c_str());
    return false;
  }

  // Verify proper naming and structure,
  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("%s: The visualization specification must be a list, but it is of XmlRpcType %d",
              visualizationsParameter_.c_str(), config.getType());
    ROS_ERROR("The XML passed in is formatted as follows:\n %s", config.toXml().c_str());
    return false;
  }

  // Iterate over all visualizations (may be just one),
  for (unsigned int i = 0; i < config.size(); ++i) {
    if (config[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR("%s: Visualizations must be specified as maps, but they are XmlRpcType:%d",
                visualizationsParameter_.c_str(), config[i].getType());
      return false;
    } else if (!config[i].hasMember("type")) {
      ROS_ERROR("%s: Could not add a visualization because no type was given",
                visualizationsParameter_.c_str());
      return false;
    } else if (!config[i].hasMember("name")) {
      ROS_ERROR("%s: Could not add a visualization because no name was given",
                visualizationsParameter_.c_str());
      return false;
    } else {
      //Check for name collisions within the list itself.
      for (int j = i + 1; j < config.size(); ++j) {
        if (config[j].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
          ROS_ERROR("%s: Visualizations must be specified as maps, but they are XmlRpcType:%d",
                    visualizationsParameter_.c_str(), config[j].getType());
          return false;
        }

        if (!config[j].hasMember("name")
            || config[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString
            || config[j]["name"].getType() != XmlRpc::XmlRpcValue::TypeString) {
          ROS_ERROR("%s: Visualizations names must be strings, but they are XmlRpcTypes:%d and %d",
                    visualizationsParameter_.c_str(), config[i].getType(), config[j].getType());
          return false;
        }

        std::string namei = config[i]["name"];
        std::string namej = config[j]["name"];
        if (namei == namej) {
          ROS_ERROR("%s: A visualization with the name '%s' already exists.",
                    visualizationsParameter_.c_str(), namei.c_str());
          return false;
        }
      }
    }

    // Make sure the visualization has a valid type.
    if (!factory_.isValidType(config[i]["type"])) {
      ROS_ERROR("Could not find visualization of type '%s'.", std::string(config[i]["type"]).c_str());
      return false;
    }
  }

  for (int i = 0; i < config.size(); ++i) {
    std::string type = config[i]["type"];
    std::string name = config[i]["name"];
    auto visualization = factory_.getInstance(type, name);
    visualization->readParameters(config[i]);
    visualizations_.push_back(visualization);
    ROS_INFO("%s: Configured visualization of type '%s' with name '%s'.",
             visualizationsParameter_.c_str(), type.c_str(), name.c_str());
  }

  return true;
}

bool GridMapVisualization::initialize()
{
  for (auto& visualization : visualizations_) {
    visualization->initialize();
  }
  updateSubscriptionCallback(ros::TimerEvent());
  ROS_INFO("Grid map visualization initialized.");
  return true;
}

void GridMapVisualization::updateSubscriptionCallback(const ros::TimerEvent&)
{
  bool isActive = false;

  for (auto& visualization : visualizations_) {
    if (visualization->isActive()) {
      isActive = true;
      break;
    }
  }

  if (!isSubscribed_ && isActive) {
    mapSubscriber_ = nodeHandle_.subscribe(mapTopic_, 1, &GridMapVisualization::callback, this);
    isSubscribed_ = true;
    ROS_DEBUG("Subscribed to grid map at '%s'.", mapTopic_.c_str());
  }
  if (isSubscribed_ && !isActive) {
    mapSubscriber_.shutdown();
    isSubscribed_ = false;
    ROS_DEBUG("Cancelled subscription to grid map.");
  }
}

void GridMapVisualization::callback(const grid_map_msgs::GridMap& message)
{
  ROS_DEBUG("Grid map visualization received a map (timestamp %f) for visualization.",
            message.info.header.stamp.toSec());
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(message, map);

  for (auto& visualization : visualizations_) {
    visualization->visualize(map);
  }
}

} /* namespace */
