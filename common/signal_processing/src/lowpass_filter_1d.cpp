// Copyright 2021 Tier IV, Inc.
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

#include "signal_processing/lowpass_filter_1d.hpp"

namespace signal_processing
{
double lowpassFilter(const double current_val, const double prev_val, const double gain)
{
  return gain * prev_val + (1.0 - gain) * current_val;
}
}  // namespace signal_processing

LowpassFilter1d::LowpassFilter1d(const double gain) : gain_(gain)
{
}

void LowpassFilter1d::reset()
{
  x_ = {};
}

void LowpassFilter1d::reset(const double x)
{
  x_ = x;
}

boost::optional<double> LowpassFilter1d::getValue() const
{
  return x_;
}

double LowpassFilter1d::filter(const double u)
{
  if (x_) {
    const double ret = gain_ * x_.get() + (1.0 - gain_) * u;
    x_ = ret;
    return x_.get();
  }

  x_ = u;
  return x_.get();
}
