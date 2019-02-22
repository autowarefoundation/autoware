#include <emergency_handler/libemergency_plan_client.h>

int EmergencyPlanClient::required_level_ = 0;
int EmergencyPlanClient::max_handling_level_ = 0;
int EmergencyPlanClient::record_level_thresh_;
boost::mutex EmergencyPlanClient::level_mutex_;
std::set<int> EmergencyPlanClient::level_list_;
ros::Publisher EmergencyPlanClient::statecmd_pub_;
ros::Publisher EmergencyPlanClient::recordcmd_pub_;
ros::Publisher EmergencyPlanClient::emlane_pub_;
ros::Publisher EmergencyPlanClient::emvel_pub_;

EmergencyPlanClient::EmergencyPlanClient(const std::pair<int, std::string>& param) :
  client_(param.second), order_id_(0), client_handling_level_(param.first), next_handling_level_(-1)
{
  max_handling_level_ = std::max(max_handling_level_, param.first);
  level_list_.insert(param.first);
  auto func = boost::bind(&EmergencyPlanClient::mainThread, this);
  thread_ = boost::shared_ptr<boost::thread>(new boost::thread(func));
}

void EmergencyPlanClient::initNextHandlingLevel()
{
  auto itr = std::find(level_list_.begin(), level_list_.end(), client_handling_level_);
  if (itr != level_list_.end() && std::next(itr, 1) != level_list_.end())
  {
    next_handling_level_ = *std::next(itr, 1);
  }
}

void EmergencyPlanClient::setupPublisher(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  pnh.param<int>("record_level_thresh", record_level_thresh_, 1);
  statecmd_pub_ = nh.advertise<std_msgs::String>("/state_cmd", 1, true);
  recordcmd_pub_ = nh.advertise<std_msgs::Header>("/record_cmd", 1, true);
  emlane_pub_ = nh.advertise<autoware_msgs::Lane>("/emergency_waypoints", 1);
  emvel_pub_ = nh.advertise<autoware_msgs::VehicleCmd>("/emergency_velocity", 1);
}

void EmergencyPlanClient::reserveOrder(int handling_level)
{
  boost::lock_guard<boost::mutex> lock(level_mutex_);
  required_level_ = std::max(required_level_, handling_level);
}

void EmergencyPlanClient::mainThread()
{
  ros::Rate r(50);
  while(ros::ok())
  {
    const bool is_connected = connectServer();
    const bool is_reserved = (client_handling_level_ == required_level_);
    bool increase_urgency = false;
    if (!is_connected)
    {
      increase_urgency = is_reserved;
    }
    else if (is_running_)
    {
      bool is_failed, is_succeeded, is_pending;
      const bool is_canceled = !is_reserved;
      client_.waitForResult(ros::Duration(0.01));
      getSimpleState(client_.getState(), is_failed, is_succeeded, is_pending);
      increase_urgency = (is_failed && !is_canceled);
      is_running_ = !(is_failed || is_succeeded || is_pending);
      if (is_running_ && is_canceled)
      {
        client_.cancelAllGoals();
      }
    }
    else if (is_reserved)
    {
      startOrder();
      is_running_ = true;
    }

    if (increase_urgency && required_level_ < max_handling_level_)
    {
      reserveOrder(next_handling_level_);
    }
    r.sleep();
  }
}

void EmergencyPlanClient::getSimpleState(const ActionState& st, bool& fail, bool& success, bool& pending)
{
  fail = (st == ActionState::REJECTED) || (st == ActionState::ABORTED) || (st == ActionState::LOST);
  success = (st == ActionState::SUCCEEDED);
  pending = (st == ActionState::PENDING);
}

void EmergencyPlanClient::startOrder()
{
  const ros::Time t = ros::Time::now();
  autoware_system_msgs::EmergencyGoal goal;
  goal.handling_level = client_handling_level_;
  client_.sendGoal(goal,
    ActionClient::SimpleDoneCallback(),
    ActionClient::SimpleActiveCallback(),
    boost::bind(&EmergencyPlanClient::fbCallback, this, _1));
  std_msgs::String str_msg;
  str_msg.data = "emergency";
  statecmd_pub_.publish(str_msg);
  if (client_handling_level_ >= record_level_thresh_)
  {
    std_msgs::Header header_msg;
    header_msg.stamp = t;
    recordcmd_pub_.publish(header_msg);
  }
}

void EmergencyPlanClient::fbCallback(const Feedback::ConstPtr& feedback)
{
  const bool is_canceled = (client_handling_level_ != required_level_);
  if (is_canceled || !is_running_)
  {
    return;
  }
  if (feedback->handling_type == Feedback::TYPE_CONTROL)
  {
    emvel_pub_.publish(feedback->control);
  }
  else if (feedback->handling_type == Feedback::TYPE_PLAN)
  {
    emlane_pub_.publish(feedback->plan);
  }
}

bool EmergencyPlanClient::connectServer()
{
  if (!client_.isServerConnected())
  {
    client_.waitForServer(ros::Duration(0.1));
    return false;
  }
  return true;
}
