#include <autoware_msgs/ConfigPlannerSelector.h>
#include <autoware_msgs/lane.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <mutex>
#include <thread>
#include <unordered_map>

#include <amathutils_lib/amathutils.hpp>
#include <planner_selector.hpp>

namespace decision_maker
{
void PlannerSelector::initROS()
{
  Subs["/dp/final_waypoints"] = nh_.subscribe("/dp/final_waypoints", 1, &PlannerSelector::callbackFromWaypoints, this);
  Subs["/astar/final_waypoints"] =
      nh_.subscribe("/astar/final_waypoints", 1, &PlannerSelector::callbackFromWaypoints, this);
  Subs["/enableLattice"] = nh_.subscribe("/enableLattice", 10, &PlannerSelector::callbackFromLattice, this);

  Subs["/dp/closest_waypoint"] = nh_.subscribe("/dp/closest_waypoint", 1, &PlannerSelector::callbackFromClosest, this);
  Subs["/astar/closest_waypoint"] =
      nh_.subscribe("/astar/closest_waypoint", 1, &PlannerSelector::callbackFromClosest, this);

  Subs["/config/planner_selector"] =
      nh_.subscribe("/config/PlannerSelector", 1, &PlannerSelector::callbackFromConfig, this);

  Subs["current_velocity"] = nh_.subscribe("current_velocity", 3, &PlannerSelector::callbackFromCurrentVelocity, this);

  Pubs["final_waypoints"] = nh_.advertise<autoware_msgs::lane>("/final_waypoints", 1);
  Pubs["closest_waypoint"] = nh_.advertise<std_msgs::Int32>("/closest_waypoint", 1);
}

inline bool PlannerSelector::existWaypoints(const int _config_waypoints_num)
{
  bool ret;
  ret = (_config_waypoints_num < final_waypoints_dp_.waypoints.size()) &&
        (_config_waypoints_num < final_waypoints_dp_.waypoints.size());
  return ret;
}

void PlannerSelector::callbackFromLattice(const std_msgs::Int32 &msg)
{
  // static int prev[LATENCY_NUM] = { 1 };
  static int counter = 0;
  double _distance = 100.0;

  try
  {
    autoware_msgs::waypoint dp_point = final_waypoints_dp_.waypoints.at(config_waypoints_num_);
    autoware_msgs::waypoint astar_point = final_waypoints_astar_.waypoints.at(config_waypoints_num_);

    amathutils::point p_dp, p_astar;
    p_dp.x = dp_point.pose.pose.position.x;
    p_dp.x = dp_point.pose.pose.position.y;
    p_dp.z = 0.0;

    p_astar.x = astar_point.pose.pose.position.x;
    p_astar.x = astar_point.pose.pose.position.y;
    p_astar.z = 0.0;

    _distance = amathutils::find_distance(&p_dp, &p_astar);
    //  ROS_INFO("distance=%f. %d:%d", _distance, dp_point.dtlane.dist,astar_point.dtlane.dist);
  }
  catch (const std::out_of_range &ex)
  {
    ROS_ERROR("Out of Range:%s", ex.what());
  }

#if 0  // delay switch
  _mutex.lock();
  if(enableLattice_){
	  if (msg.data == 0){
		  if(counter++ >= config_latency_num_){
			  enableLattice_ = 0;
			  counter = 0;
		  }
	  }
  }else{
	  enableLattice_ = msg.data;
  }
  ROS_INFO("msg.data=%d, enableLattice_ = %d", msg.data, enableLattice_);
  _mutex.unlock();
#else
  _mutex.lock();
  if (msg.data != enableLattice_)
  {
    if (enableLattice_ == 1 && msg.data == 0)
    {
      if (pastWaypoint == false)
      {
        pastWaypoint = true;
        counter = 0;
        way_offset = config_waypoints_num_;
      }
    }
    enableLattice_ = msg.data;
  }
  if (counter++ >= config_latency_num_)
  {
    counter = 0;
    if (way_offset > 0)
    {
      way_offset--;
    }
    else
    {
      pastWaypoint = false;
    }
  }

  // ROS_INFO("msg.data=%d, enableLattice_ = %d", msg.data, enableLattice_);
  _mutex.unlock();
#endif
  // for debug
  //	ROS_INFO("\n***** EnableLattice = %d  **** \n",enableLattice_,msg.data);
}

void PlannerSelector::callbackFromWaypoints(const ros::MessageEvent<autoware_msgs::lane const> &event)
{
  const ros::M_string &header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  const autoware_msgs::lane *waypoints = event.getMessage().get();

  _mutex.lock();

  if (this->enableLattice_ && topic.find("dp") != std::string::npos)
  {
    Pubs["final_waypoints"].publish(*waypoints);
    final_waypoints_dp_ = *waypoints;
  }
  else if (!this->enableLattice_ && topic.find("astar") != std::string::npos)
  {
    if (pastWaypoint)
    {
      for (int i = 0; i < config_waypoints_num_; i++)
      {
        if (!final_waypoints_astar_.waypoints.empty())
          final_waypoints_astar_.waypoints.erase(final_waypoints_astar_.waypoints.begin());
        else
        {
          pastWaypoint = false;
          way_offset = 0;
        }
      }
#if 0
	    if(final_waypoints_astar_.waypoints.empty()){
		    int _size = final_waypoints_astar_.waypoints.size();
		    _size = _size>=5?5:_size;
		    auto itr = final_waypoints_astar_.waypoints.begin();
		    for(int i=0; i < 5;  i++){
			    itr->twist.twist.linear.x =  (current_velocity_*2 + itr->twist.twist.linear.x) / 3;
			    std::cout << "set linear velocity:" <<  mps2kmph(itr->twist.twist.linear.x)  << std::endl; 
			    itr++;
		    }
	    }
#endif
      Pubs["final_waypoints"].publish(final_waypoints_astar_);
    }
    else
    {
      Pubs["final_waypoints"].publish(*waypoints);
    }
    final_waypoints_astar_ = *waypoints;
  }

  // for debug
  // ROS_INFO("%s, %d-%d-%d", topic.c_str(), closest_waypoint_dp_, closest_waypoint_astar_, this->enableLattice_);
  _mutex.unlock();
}

void PlannerSelector::callbackFromCurrentVelocity(const geometry_msgs::TwistStamped &msg)
{
  current_velocity_ = msg.twist.linear.x;
}

void PlannerSelector::callbackFromConfig(const autoware_msgs::ConfigPlannerSelector &msg)
{
  config_latency_num_ = msg.latency_num;
  config_waypoints_num_ = msg.waypoints_num;
  config_convergence_num_ = msg.convergence_num;

  ROS_INFO("PARAM_SET-latency:%d, waypoints:%d, convergence:%f", config_latency_num_, config_waypoints_num_,
           config_convergence_num_);
}

void PlannerSelector::callbackFromClosest(const ros::MessageEvent<std_msgs::Int32> &event)
{
  const ros::M_string &header = event.getConnectionHeader();
  std::string topic = header.at("topic");

  int temp = event.getMessage().get()->data;
  std_msgs::Int32 msg;

#if 1
  if (topic.find("/dp") == 0)
  {
    closest_waypoints_["dp"] = temp;

    if (closest_waypoints_["astar"])
    {
      msg.data = closest_waypoint_astar_;
    }
    closest_waypoint_dp_ = temp;
  }
  else if (topic.find("/astar") == 0)
  {
    closest_waypoints_["astar"] = temp;
    closest_waypoint_astar_ = temp;
  }

  //  ROS_INFO("PastWaypoint:%s-offset:%d, latency:%d, closest:%d", pastWaypoint?"true":"false", way_offset,
  //  config_latency_num_,closest_waypoint_astar);
  if (pastWaypoint && final_waypoints_astar_.waypoints.size() > closest_waypoint_astar_ + way_offset)
  {
    msg.data = closest_waypoint_astar_ + way_offset;
  }
  else
  {
    msg.data = closest_waypoint_astar_;
  }
#endif
  Pubs["closest_waypoint"].publish(msg);
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_selector");

  decision_maker::PlannerSelector _psn;
  ros::spin();
}
