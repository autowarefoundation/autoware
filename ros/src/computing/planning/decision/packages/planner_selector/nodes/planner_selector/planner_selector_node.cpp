#include <autoware_msgs/lane.h>
#include <autoware_msgs/ConfigPlannerSelector.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <mutex>
#include <thread>
#include <unordered_map>

#include <euclidean_space.hpp>

namespace decision_maker
{
#define DEFAULT_LATENCY_NUM 5
#define DEFAULT_WAYPOINTS_NUM 3
#define DEFAULT_CONVERGENCE_NUM 2.0
class PlannerSelectorNode
{
private:
  ros::NodeHandle nh_;

  std::unordered_map<std::string, autoware_msgs::lane> waypoints_;
  std::unordered_map<std::string, ros::Publisher> Pubs;
  std::unordered_map<std::string, ros::Subscriber> Subs;


  autoware_msgs::lane final_waypoints_dp;
  autoware_msgs::lane final_waypoints_astar;

  std::unordered_map<std::string, int> closest_waypoints_;
  int __closest_waypoint_astar;
  int __closest_waypoint_dp;

  unsigned int way_offset;

  std::mutex _mutex;

  int enableLattice_;
  bool pastWaypoint;
  unsigned int config_latency_num;
  unsigned int config_waypoints_num;
  double config_convergence_num;

public:
  PlannerSelectorNode()
  {
    this->initROS();
    enableLattice_ = 0;
    pastWaypoint = false;
    config_latency_num = DEFAULT_LATENCY_NUM;
    config_waypoints_num = DEFAULT_WAYPOINTS_NUM;
    config_convergence_num = DEFAULT_CONVERGENCE_NUM;
  }

  void initROS();

  void callbackFromClosest(const ros::MessageEvent<std_msgs::Int32> &event);
  void callbackFromWaypoints(const ros::MessageEvent<autoware_msgs::lane const> &event);
  void callbackFromLattice(const std_msgs::Int32 &msg);
  void callbackFromConfig(const autoware_msgs::ConfigPlannerSelector &msg);
};

void PlannerSelectorNode::initROS()
{
  Subs["/dp/final_waypoints"] =
      nh_.subscribe("/dp/final_waypoints", 1, &PlannerSelectorNode::callbackFromWaypoints, this);
  Subs["/astar/final_waypoints"] =
      nh_.subscribe("/astar/final_waypoints", 1, &PlannerSelectorNode::callbackFromWaypoints, this);
  Subs["/enableLattice"] = nh_.subscribe("/enableLattice", 10, &PlannerSelectorNode::callbackFromLattice, this);

  Subs["/dp/closest_waypoint"] =
      nh_.subscribe("/dp/closest_waypoint", 1, &PlannerSelectorNode::callbackFromClosest, this);
  Subs["/astar/closest_waypoint"] =
      nh_.subscribe("/astar/closest_waypoint", 1, &PlannerSelectorNode::callbackFromClosest, this);

  Subs["/config/planner_selector"] = nh_.subscribe("/config/PlannerSelector", 1, &PlannerSelectorNode::callbackFromConfig, this);

  Pubs["final_waypoints"] = nh_.advertise<autoware_msgs::lane>("/final_waypoints", 1);
  Pubs["closest_waypoint"] = nh_.advertise<std_msgs::Int32>("/closest_waypoint", 1);
}

void PlannerSelectorNode::callbackFromLattice(const std_msgs::Int32 &msg)
{
  //static int prev[LATENCY_NUM] = { 1 };
  static int counter = 0;
  
  double _distance = 100.0;

  try{
	  autoware_msgs::waypoint dp_point = final_waypoints_dp.waypoints.at(config_waypoints_num);
	  autoware_msgs::waypoint astar_point = final_waypoints_astar.waypoints.at(config_waypoints_num);

	  euclidean_space::point p_dp, p_astar;
	  p_dp.x = dp_point.pose.pose.position.x;
	  p_dp.x = dp_point.pose.pose.position.y;
	  p_dp.z = 0.0;

	  p_astar.x = astar_point.pose.pose.position.x;
	  p_astar.x = astar_point.pose.pose.position.y;
	  p_astar.z = 0.0;

	  _distance = euclidean_space::EuclideanSpace::find_distance(&p_dp, &p_astar);
	//  ROS_INFO("distance=%f. %d:%d", _distance, dp_point.dtlane.dist,astar_point.dtlane.dist);
  }catch(const std::out_of_range &ex){
  }

#if 0
  _mutex.lock();
  if(enableLattice_){
	  if (msg.data == 0){
		  if(counter++ >= config_latency_num){
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
  if (msg.data != enableLattice_){
	  if(enableLattice_ == 1 && msg.data == 0){
		  if(pastWaypoint == false){
			  pastWaypoint  = true;
			  counter = 0;
			  way_offset = config_waypoints_num;
		  }
	  }
	  enableLattice_ = msg.data;
  }
  if(counter++ >= config_latency_num)
  {
	  counter = 0;
	  if(way_offset>0)
		  way_offset--;
	  else
		  pastWaypoint = false;
  }



  //ROS_INFO("msg.data=%d, enableLattice_ = %d", msg.data, enableLattice_);
  _mutex.unlock();
#endif
  // for debug
  //	ROS_INFO("\n***** EnableLattice = %d  **** \n",enableLattice_,msg.data);
}

void PlannerSelectorNode::callbackFromWaypoints(const ros::MessageEvent<autoware_msgs::lane const> &event)
{
  const ros::M_string &header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  const autoware_msgs::lane *waypoints = event.getMessage().get();

  /// ROS_INFO("%s", enableLattice_?"flag = True":"flag = False");

  _mutex.lock();
  if (this->enableLattice_ && topic.find("dp") != std::string::npos)
  {
    Pubs["final_waypoints"].publish(*waypoints);
    final_waypoints_dp = *waypoints;
  }
  else if (!this->enableLattice_ && topic.find("astar") != std::string::npos)
  {

    if(pastWaypoint){
	    static bool initflag = true;
	    for(int i=0; i<config_waypoints_num; i++){
		    if(!final_waypoints_astar.waypoints.empty())
			    final_waypoints_astar.waypoints.erase(final_waypoints_astar.waypoints.begin());
		    else{
			    pastWaypoint = false;
			    way_offset=0;

		    }
	    }
	    Pubs["final_waypoints"].publish(final_waypoints_astar);
    }else{
	    Pubs["final_waypoints"].publish(*waypoints);
    }
    final_waypoints_astar = *waypoints;
  }
  //for debug
  //ROS_INFO("%s, %d-%d-%d", topic.c_str(), __closest_waypoint_dp, __closest_waypoint_astar, this->enableLattice_);
  _mutex.unlock();
}

void PlannerSelectorNode::callbackFromConfig(const autoware_msgs::ConfigPlannerSelector &msg)
{
	config_latency_num = msg.latency_num;
	config_waypoints_num = msg.waypoints_num;
	config_convergence_num = msg.convergence_num;

	ROS_INFO("PARAM_SET-latency:%d, waypoints:%d, convergence:%f", config_latency_num, config_waypoints_num, config_convergence_num);
}


void PlannerSelectorNode::callbackFromClosest(const ros::MessageEvent<std_msgs::Int32> &event)
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
      msg.data = __closest_waypoint_astar;
    }
    __closest_waypoint_dp = temp;
  }
  else if (topic.find("/astar") == 0)
  {
    closest_waypoints_["astar"] = temp;
    __closest_waypoint_astar = temp;
  }

//  ROS_INFO("PastWaypoint:%s-offset:%d, latency:%d, closest:%d", pastWaypoint?"true":"false", way_offset, config_latency_num,__closest_waypoint_astar);
  if(pastWaypoint &&  final_waypoints_astar.waypoints.size() > __closest_waypoint_astar + way_offset){
	  msg.data = __closest_waypoint_astar + way_offset;
  }else{
	  msg.data = __closest_waypoint_astar;
  }
#endif
  Pubs["closest_waypoint"].publish(msg);
}
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_selector");

  decision_maker::PlannerSelectorNode _psn;
  ros::spin();
}
