#include <emergency_handler/libemergency_handler.h>

EmergencyHandler::EmergencyHandler(ros::NodeHandle &nh, ros::NodeHandle &pnh) :
  status_sub_(nh, pnh), priority_(0), spinner_(1)
{
  EmergencyPlanClient::setupPublisher(nh, pnh);
  spinner_.start();
}

EmergencyHandler::~EmergencyHandler()
{
  spinner_.stop();
}

void EmergencyHandler::addPublisher(const std::map<int, std::string>& behavior)
{
  for (const auto& el : behavior)
  {
    emplan_client_.emplace(el.first, boost::shared_ptr<EmergencyPlanClient>(new EmergencyPlanClient(el)));
  }
}

void EmergencyHandler::addFilter(const SystemStatusFilter& filter)
{
  status_sub_.addCallback(boost::bind(&EmergencyHandler::wrapFunc, this, filter.getFunc(), _1));
}

void EmergencyHandler::run()
{
  for (auto& el : emplan_client_)
  {
    el.second->initNextPriority();
  }
  status_sub_.addCallback(boost::bind(&EmergencyHandler::reserveCallback, this, _1));
  status_sub_.enable();
}

void EmergencyHandler::wrapFunc(FilterFunc func, SystemStatus status)
{
  priority_ = func(status);
}

void EmergencyHandler::reserveCallback(SystemStatus status)
{
  EmergencyPlanClient::reserveOrder(priority_);
}
