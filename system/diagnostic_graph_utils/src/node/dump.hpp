// Copyright 2024 The Autoware Contributors
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

#ifndef NODE__DUMP_HPP_
#define NODE__DUMP_HPP_

#include "diagnostic_graph_utils/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>

namespace diagnostic_graph_utils
{

class DumpNode : public rclcpp::Node
{
public:
  explicit DumpNode(const rclcpp::NodeOptions & options);

private:
  void on_create(DiagGraph::ConstSharedPtr graph);
  void on_update(DiagGraph::ConstSharedPtr graph);
  DiagGraphSubscription sub_graph_;

  struct TableLine
  {
    int index;
    std::string text1;
    std::string text2;
  };

  std::unordered_map<DiagUnit *, TableLine> table_;
  std::string header_;
  std::string border_;
};

}  // namespace diagnostic_graph_utils

#endif  // NODE__DUMP_HPP_
