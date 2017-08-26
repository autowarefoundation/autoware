#include <autoware_msgs/lane.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <mutex>
#include <thread>
#include <unordered_map>

namespace decision_maker
{
class PlannerSelectorNode
{
private:
  ros::NodeHandle nh_;

  std::unordered_map<std::string, autoware_msgs::lane> waypoints_;
  std::unordered_map<std::string, ros::Publisher> Pubs;
  std::unordered_map<std::string, ros::Subscriber> Subs;

  std::unordered_map<std::string, int> closest_waypoints_;
  int __closest_waypoint_astar;
  int __closest_waypoint_dp;

  std::mutex _mutex;

  int enableLattice_;

public:
  PlannerSelectorNode()
  {
    this->initROS();
    enableLattice_ = 0;
  }

  void initROS();

  void callbackFromClosest(const ros::MessageEvent<std_msgs::Int32> &event);
  void callbackFromWaypoints(const ros::MessageEvent<autoware_msgs::lane const> &event);
  void callbackFromLattice(const std_msgs::Int32 &msg);
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

  Pubs["final_waypoints"] = nh_.advertise<autoware_msgs::lane>("/final_waypoints", 1);
  Pubs["closest_waypoint"] = nh_.advertise<std_msgs::Int32>("/closest_waypoint", 1);
}

#define LATENCY_NUM 3
void PlannerSelectorNode::callbackFromLattice(const std_msgs::Int32 &msg)
{
  static int prev[LATENCY_NUM] = { 1 };
#if 0
	if(enableLattice_ == 1)	{
		for(int i = 1; i<LATENCY_NUM; i++)
			prev[i] = prev[i-1];
		prev[0] = msg.data;
		enableLattice_ = prev[LATENCY_NUM];
	}else{
		enableLattice_ = msg.data;
	}
#else
  _mutex.lock();
  if (msg.data != enableLattice_)
    enableLattice_ = msg.data;
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
  }
  else if (!this->enableLattice_ && topic.find("astar") != std::string::npos)
  {
    Pubs["final_waypoints"].publish(*waypoints);
  }
//for debug
//ROS_INFO("%s, %d-%d-%d", topic.c_str(), __closest_waypoint_dp, __closest_waypoint_astar, this->enableLattice_);
  _mutex.unlock();
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
  msg.data = __closest_waypoint_astar;
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
