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

#ifndef YABLOC_COMMON__GROUND_SERVER__FILTER__MOVING_AVERAGING_HPP_
#define YABLOC_COMMON__GROUND_SERVER__FILTER__MOVING_AVERAGING_HPP_

#include <Eigen/Core>

#include <boost/circular_buffer.hpp>

namespace yabloc::ground_server
{
class MovingAveraging
{
public:
  MovingAveraging() : buffer_(50) {}

  Eigen::Vector3f update(const Eigen::Vector3f & normal)
  {
    buffer_.push_back(normal);

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const Eigen::Vector3f & v : buffer_) mean += v;
    mean /= buffer_.size();

    return mean.normalized();
  }

private:
  boost::circular_buffer<Eigen::Vector3f> buffer_;
};
}  // namespace yabloc::ground_server

#endif  // YABLOC_COMMON__GROUND_SERVER__FILTER__MOVING_AVERAGING_HPP_
