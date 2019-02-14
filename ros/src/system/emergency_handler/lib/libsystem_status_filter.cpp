#include <emergency_handler/libsystem_status_filter.h>

SystemStatusFilter::SystemStatusFilter() :
  callback_(std::bind(&SystemStatusFilter::selectBehavior, this, std::placeholders::_1)){}

std::string SystemStatusFilter::selectBehavior(const SystemStatus& status)
{
  return "None";
}

const std::function<std::string(const SystemStatus&)>& SystemStatusFilter::getFunc() const
{
  return callback_;
}

StatusType SystemStatusFilter::getStatus(const DiagnosticStatusArray& st_array, int level_th) const
{
  const auto found = find_if(st_array.status.begin(), st_array.status.end(),
    [=](const DiagnosticStatus& s){return s.level >= level_th;});
  if (found != st_array.status.end())
  {
    return StatusType::ERROR;
  }
  return StatusType::OK;
}

StatusType SystemStatusFilter::getStatus(const NodeStatus& node_status, int level_th) const
{
  if (!node_status.node_activated)
  {
    return StatusType::NOT_READY;
  }
  for(const auto& st_array : node_status.status)
  {
    if (getStatus(st_array, level_th) == StatusType::ERROR)
    {
      return StatusType::ERROR;
    }
  }
  return StatusType::OK;
}

StatusType SystemStatusFilter::getStatus(const HardwareStatus& hw_status, int level_th) const
{
  for(const auto& st_array : hw_status.status)
  {
    if (getStatus(st_array, level_th) == StatusType::ERROR)
    {
      return StatusType::ERROR;
    }
  }
  return StatusType::OK;
}

StatusType SystemStatusFilter::getStatus(std::string node_name, const std::vector<NodeStatus>& array, int level_th) const
{
  const auto found = find_if(array.begin(), array.end(),
    [=](const NodeStatus& s){return s.node_name == node_name;});
  if (found != array.end())
  {
    return getStatus(*found, level_th);
  }
  return StatusType::NONE;
}

bool SystemStatusFilter::checkAllNodeSimplly(const std::vector<NodeStatus>& array, int level_th) const
{
  return checkAllSimplly<NodeStatus>(array, level_th);
}

bool SystemStatusFilter::checkAllHardwareSimplly(const std::vector<HardwareStatus>& array, int level_th) const
{
  return checkAllSimplly<HardwareStatus>(array, level_th);
}

template<typename T> bool SystemStatusFilter::checkAllSimplly(const std::vector<T>& array, int level_th) const
{
  for (const auto& st : array)
  {
    const StatusType status = getStatus(st, level_th);
    if (status == StatusType::NOT_READY)
    {
      continue;
    }
    else if (status == StatusType::ERROR)
    {
      return false;
    }
  }
  return true;
}
