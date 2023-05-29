// Copyright 2023 LeoDrive, A.Ş.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "moderate_stop_interface.hpp"

namespace vehicle_cmd_gate
{

ModerateStopInterface::ModerateStopInterface(rclcpp::Node * node) : node_(node)
{
  const auto adaptor = component_interface_utils::NodeAdaptor(node);
  adaptor.init_srv(srv_set_stop_, this, &ModerateStopInterface::on_stop_request);
  adaptor.init_pub(pub_is_stopped_);

  stop_state_.data = false;
  stop_state_.requested_sources.clear();
  publish();
}

bool ModerateStopInterface::is_stop_requested() const
{
  return stop_state_.data;
}

void ModerateStopInterface::publish()
{
  if (prev_stop_map_ != stop_map_) {
    pub_is_stopped_->publish(stop_state_);
    prev_stop_map_ = stop_map_;
  }
}

void ModerateStopInterface::on_stop_request(
  const SetStop::Service::Request::SharedPtr req, const SetStop::Service::Response::SharedPtr res)
{
  stop_map_[req->request_source] = req->stop;
  update_stop_state();
  res->status.success = true;
}

void ModerateStopInterface::update_stop_state()
{
  stop_state_.stamp = node_->now();
  stop_state_.requested_sources.clear();
  stop_state_.data = false;

  if (stop_map_.empty()) {
    return;
  }
  for (auto & itr : stop_map_) {
    if (itr.second) {
      stop_state_.data = true;
      stop_state_.requested_sources.push_back(itr.first);
    }
  }
}

}  // namespace vehicle_cmd_gate
