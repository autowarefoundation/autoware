/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <emergency_handler/libemergency_handler.h>

EmergencyHandler::EmergencyHandler(ros::NodeHandle &nh, ros::NodeHandle &pnh) :
  status_sub_(nh, pnh), priority_(INT_MAX), spinner_(1)
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
  priority_ = std::min(func(status), priority_);
}

void EmergencyHandler::reserveCallback(SystemStatus status)
{
  EmergencyPlanClient::reserveOrder(priority_);
  EmergencyPlanClient::getErrorStatusSummary(SystemStatusFilter::getFactorStatusArray());
  SystemStatusFilter::resetFactorStatusArray();
  priority_ = INT_MAX;
}
