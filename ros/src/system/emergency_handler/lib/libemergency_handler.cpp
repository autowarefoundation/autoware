#include <emergency_handler/libemergency_handler.h>

EmergencyHandler::EmergencyHandler(ros::NodeHandle &nh, ros::NodeHandle &pnh) :
  nh_(nh), pnh_(pnh), status_sub_(nh_, pnh_), handling_level_(0)
{
  pnh_.param<int>("record_level_thresh", record_level_thresh_, 1);
  statecmd_pub_ = nh_.advertise<std_msgs::String>("/state_cmd", 1, true);
  recordcmd_pub_ = nh_.advertise<std_msgs::Header>("/record_cmd", 1, true);
}

EmergencyHandler::~EmergencyHandler(){}

void EmergencyHandler::addPublisher(const std::map<int, std::string>& behavior)
{
  for (const auto& el : behavior)
  {
    emergency_pub_[el.first] = nh_.advertise<std_msgs::Header>(el.second, 1, true);
  }
}

void EmergencyHandler::addFilter(const SystemStatusFilter& filter)
{
  status_sub_.addCallback(std::bind(&EmergencyHandler::wrapFunc, this, filter.getFunc(), std::placeholders::_1));
}

void EmergencyHandler::run()
{
  status_sub_.addCallback(std::bind(&EmergencyHandler::publishCallback, this, std::placeholders::_1));
  status_sub_.enable();
}

void EmergencyHandler::wrapFunc(std::function<int(const SystemStatus&)> func, SystemStatus status)
{
  handling_level_ = func(status);
}

void EmergencyHandler::publishCallback(SystemStatus status)
{
  if (handling_level_ == 0)
  {
    return;
  }
  std_msgs::Header header_msg;
  header_msg.stamp = ros::Time::now();
  emergency_pub_[handling_level_].publish(header_msg);
  std_msgs::String str_msg;
  str_msg.data = "emergency";
  statecmd_pub_.publish(str_msg);
  if (handling_level_ >= record_level_thresh_)
  {
    recordcmd_pub_.publish(header_msg);
  }
  handling_level_ = 0;
}
