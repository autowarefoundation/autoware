#ifndef __PLANNER_SELECTOR_HPP__
#define __PLANNER_SELECTOR_HPP__

#include <autoware_msgs/ConfigPlannerSelector.h>
#include <autoware_msgs/lane.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <mutex>
#include <thread>
#include <unordered_map>

#include <amathutils_lib/amathutils.hpp>

namespace decision_maker
{
#define DEFAULT_LATENCY_NUM 5
#define DEFAULT_WAYPOINTS_NUM 3
#define DEFAULT_CONVERGENCE_NUM 2.0
class PlannerSelector
{
private:
  ros::NodeHandle nh_;

  std::unordered_map<std::string, autoware_msgs::lane> waypoints_;
  std::unordered_map<std::string, ros::Publisher> Pubs;
  std::unordered_map<std::string, ros::Subscriber> Subs;

  autoware_msgs::lane final_waypoints_dp_;
  autoware_msgs::lane final_waypoints_astar_;

  std::unordered_map<std::string, int> closest_waypoints_;
  int closest_waypoint_astar_;
  int closest_waypoint_dp_;

  unsigned int way_offset;

  std::mutex _mutex;

  int enableLattice_;
  bool pastWaypoint;
  unsigned int config_latency_num_;
  unsigned int config_waypoints_num_;
  double config_convergence_num_;
  double current_velocity_;

  bool existWaypoints(const int _config_waypoints_num);

public:
  PlannerSelector()
  {
    this->initROS();
    enableLattice_ = 0;
    pastWaypoint = false;
    config_latency_num_ = DEFAULT_LATENCY_NUM;
    config_waypoints_num_ = DEFAULT_WAYPOINTS_NUM;
    config_convergence_num_ = DEFAULT_CONVERGENCE_NUM;
  }

  void initROS();

  void callbackFromClosest(const ros::MessageEvent<std_msgs::Int32> &event);
  void callbackFromWaypoints(const ros::MessageEvent<autoware_msgs::lane const> &event);
  void callbackFromLattice(const std_msgs::Int32 &msg);
  void callbackFromConfig(const autoware_msgs::ConfigPlannerSelector &msg);
  void callbackFromCurrentVelocity(const geometry_msgs::TwistStamped &msg);
};
}
#endif
