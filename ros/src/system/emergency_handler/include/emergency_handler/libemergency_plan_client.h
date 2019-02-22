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
  void initNextHandlingLevel();
  static void setupPublisher(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  static void reserveOrder(int handling_level);
private:
  void fbCallback(const Feedback::ConstPtr& feedback);
  void mainThread();
  void getSimpleState(const ActionState& st, bool& fail, bool& success, bool& pending);
  void startOrder();
  bool connectServer();

  boost::shared_ptr<boost::thread> thread_;
  ActionClient client_;
  const int client_handling_level_;
  int next_handling_level_, order_id_;
  bool is_running_;

  static ros::Publisher statecmd_pub_, recordcmd_pub_, emlane_pub_, emvel_pub_;
  static boost::mutex level_mutex_;
  static int required_level_, max_handling_level_, record_level_thresh_;
  static std::set<int> level_list_;
};

#endif
