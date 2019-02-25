#ifndef __EMERGENCY_PLAN_CLIENT_H__
#define __EMERGENCY_PLAN_CLIENT_H__
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <autoware_system_msgs/EmergencyAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

typedef actionlib::SimpleActionClient<autoware_system_msgs::EmergencyAction> ActionClient;
typedef actionlib::SimpleClientGoalState ActionState;
typedef autoware_system_msgs::EmergencyFeedback Feedback;
class EmergencyPlanClient
{
public:
  EmergencyPlanClient(const std::pair<int, std::string>& param);
  void initNextPriority();
  static void setupPublisher(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  static void reserveOrder(int priority);
  static void resetOrder();
private:
  void fbCallback(const Feedback::ConstPtr& feedback);
  void mainThread();
  void getSimpleState(const ActionState& st, bool& fail, bool& success, bool& pending);
  void startOrder();
  bool connectServer();

  boost::shared_ptr<boost::thread> thread_;
  ActionClient client_;
  const int client_priority_;
  int next_priority_, order_id_;
  bool is_running_;

  static ros::Publisher statecmd_pub_, recordcmd_pub_, emlane_pub_, emvel_pub_;
  static boost::mutex priority_mutex_;
  static const int normal_behavior_;
  static int required_priority_, min_priority_, record_priority_thresh_;
  static std::set<int> priority_list_;
};

#endif
