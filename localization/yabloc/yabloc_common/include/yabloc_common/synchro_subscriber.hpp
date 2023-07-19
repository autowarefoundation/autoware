// Copyright 2023 TIER IV, Inc.
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

#ifndef YABLOC_COMMON__SYNCHRO_SUBSCRIBER_HPP_
#define YABLOC_COMMON__SYNCHRO_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <memory>
#include <string>

namespace yabloc::common
{
template <typename Msg1, typename Msg2>
class SynchroSubscriber
{
public:
  using SharedPtr = std::shared_ptr<SynchroSubscriber>;
  using Sub1 = message_filters::Subscriber<Msg1>;
  using Sub2 = message_filters::Subscriber<Msg2>;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Msg1, Msg2>;
  using UserCallback = std::function<void(const Msg1 &, const Msg2 &)>;

  SynchroSubscriber(rclcpp::Node * n, const std::string & topic1, const std::string & topic2)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    sub1_ = std::make_shared<Sub1>(n, topic1);
    sub2_ = std::make_shared<Sub2>(n, topic2);
    synchronizer_ = std::make_shared<Synchronizer>(SyncPolicy(80), *sub1_, *sub2_);
    synchronizer_->registerCallback(std::bind(&SynchroSubscriber::raw_callback, this, _1, _2));
  }

  void set_callback(const UserCallback & callback) { user_callback_ = callback; }

private:
  std::shared_ptr<Sub1> sub1_;
  std::shared_ptr<Sub2> sub2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  std::optional<UserCallback> user_callback_{std::nullopt};

  void raw_callback(const typename Msg1::ConstPtr & msg1, const typename Msg2::ConstPtr & msg2)
  {
    if (user_callback_.has_value()) user_callback_.value()(*msg1, *msg2);
  }
};

}  // namespace yabloc::common

#endif  // YABLOC_COMMON__SYNCHRO_SUBSCRIBER_HPP_
