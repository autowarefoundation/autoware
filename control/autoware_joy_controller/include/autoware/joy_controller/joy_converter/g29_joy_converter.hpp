// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__JOY_CONTROLLER__JOY_CONVERTER__G29_JOY_CONVERTER_HPP_
#define AUTOWARE__JOY_CONTROLLER__JOY_CONVERTER__G29_JOY_CONVERTER_HPP_

#include "autoware/joy_controller/joy_converter/joy_converter_base.hpp"

#include <cstdlib>

namespace autoware::joy_controller
{
class G29JoyConverter : public JoyConverterBase
{
public:
  explicit G29JoyConverter(const sensor_msgs::msg::Joy & j) : j_(j) {}

  float accel() const
  {
    constexpr float eps = 0.0000001;
    if (std::abs(AccelPedal()) < eps) {
      return 0.0f;
    }
    return (AccelPedal() + 1.0f) / 2;
  }

  float brake() const
  {
    constexpr float eps = 0.0000001;
    if (std::abs(BrakePedal()) < eps) {
      return 0.0f;
    }
    return (BrakePedal() + 1.0f) / 2;
  }

  float steer() const { return Steer(); }

  bool shift_up() const { return CursorUpDown() == 1.0f; }
  bool shift_down() const { return CursorUpDown() == -1.0f; }
  bool shift_drive() const { return CursorLeftRight() == 1.0f; }
  bool shift_reverse() const { return CursorLeftRight() == -1.0f; }

  bool turn_signal_left() const { return L1(); }
  bool turn_signal_right() const { return R1(); }
  bool clear_turn_signal() const { return Share(); }

  bool gate_mode() const { return Options(); }

  bool emergency_stop() const { return !reverse() && PS(); }
  bool clear_emergency_stop() const { return reverse() && PS(); }

  bool autoware_engage() const { return !reverse() && Circle(); }
  bool autoware_disengage() const { return reverse() && Circle(); }

  bool vehicle_engage() const { return !reverse() && Triangle(); }
  bool vehicle_disengage() const { return reverse() && Triangle(); }

private:
  float Steer() const { return j_.axes.at(0); }
  float AccelPedal() const { return j_.axes.at(2); }
  float BrakePedal() const { return j_.axes.at(3); }

  float CursorLeftRight() const { return j_.axes.at(4); }
  float CursorUpDown() const { return j_.axes.at(5); }

  bool Cross() const { return j_.buttons.at(0); }
  bool Circle() const { return j_.buttons.at(1); }
  bool Triangle() const { return j_.buttons.at(2); }
  bool Square() const { return j_.buttons.at(3); }
  bool L1() const { return j_.buttons.at(5); }
  bool R1() const { return j_.buttons.at(4); }
  bool L2() const { return j_.buttons.at(7); }
  bool R2() const { return j_.buttons.at(6); }
  bool Share() const { return j_.buttons.at(8); }
  bool Options() const { return j_.buttons.at(9); }
  bool PS() const { return j_.buttons.at(24); }

  const sensor_msgs::msg::Joy j_;

  bool reverse() const { return Share(); }
};
}  // namespace autoware::joy_controller

#endif  // AUTOWARE__JOY_CONTROLLER__JOY_CONVERTER__G29_JOY_CONVERTER_HPP_
