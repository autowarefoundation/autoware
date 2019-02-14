#include <emergency_handler/libemergency_handler.h>

EmergencyHandler::EmergencyHandler(ros::NodeHandle &nh, ros::NodeHandle &pnh) :
  autoware_health_checker::SystemStatusSubscriber(nh, pnh),
  is_emergency_(false), start_record_(false)
{
  emergency_pub_ = nh.advertise<std_msgs::Header>("/emergency_cmd", 1, true);
  statecmd_pub_ = nh.advertise<std_msgs::String>("/state_cmd", 1, true);
  recordcmd_pub_ = nh.advertise<std_msgs::Header>("/record_cmd", 1, true);
}

EmergencyHandler::~EmergencyHandler(){}

void EmergencyHandler::addFilter(const SystemStatusFilter& filter)
{
  addCallback(std::bind(&EmergencyHandler::wrapFunc, this, filter.getFunc(), std::placeholders::_1));
}

void EmergencyHandler::addPublishCallback()
{
  addCallback(std::bind(&EmergencyHandler::publishCallback, this, std::placeholders::_1));
}

void EmergencyHandler::changePublisherFlag(std::string behavior)
{
  if (behavior == "None")
  {
    return;
  }
  if (behavior.find("Emergency") != std::string::npos)
  {
    is_emergency_ = true;
  }
  else if (!is_emergency_)
  {
    const std::string key = "Cmd=";
    if (behavior.find(key) != std::string::npos)
    {
      const auto len = key.length();
      statecmd_str_ = behavior;
      statecmd_str_.replace(0, len, "");
    }
  }
  start_record_ = (is_emergency_ || !statecmd_str_.empty());
}

void EmergencyHandler::wrapFunc(std::function<std::string(const SystemStatus&)> func, SystemStatus status)
{
  const std::string ret = func(status);
  changePublisherFlag(ret);
}

void EmergencyHandler::publishCallback(SystemStatus status)
{
  std_msgs::Header header_msg;
  header_msg.stamp = ros::Time::now();

  if (is_emergency_)
  {
    emergency_pub_.publish(header_msg);
  }
  else if (!statecmd_str_.empty())
  {
    std_msgs::String str_msg;
    str_msg.data = statecmd_str_;
    statecmd_pub_.publish(str_msg);
  }
  if (start_record_)
  {
    recordcmd_pub_.publish(header_msg);
  }
  is_emergency_ = start_record_ = false;
  statecmd_str_.clear();
}
