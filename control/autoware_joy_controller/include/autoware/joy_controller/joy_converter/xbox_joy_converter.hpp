// Copyright 2023 The Autoware Contributors
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

#ifndef AUTOWARE__JOY_CONTROLLER__JOY_CONVERTER__XBOX_JOY_CONVERTER_HPP_
#define AUTOWARE__JOY_CONTROLLER__JOY_CONVERTER__XBOX_JOY_CONVERTER_HPP_

#include "autoware/joy_controller/joy_converter/joy_converter_base.hpp"

#include <algorithm>

namespace autoware::joy_controller
{
class XBOXJoyConverter : public JoyConverterBase
{
public:
  explicit XBOXJoyConverter(const sensor_msgs::msg::Joy & j) : j_(j) {}

  float accel() const { return std::max(0.0f, -((RT() - 1.0f) / 2.0f)); }

  float brake() const { return std::max(0.0f, -((LT() - 1.0f) / 2.0f)); }

  float steer() const { return LStickLeftRight(); }

  bool shift_up() const { return CursorUpDown() == 1.0f; }
  bool shift_down() const { return CursorUpDown() == -1.0f; }
  bool shift_drive() const { return CursorLeftRight() == 1.0f; }
  bool shift_reverse() const { return CursorLeftRight() == -1.0f; }

  bool turn_signal_left() const { return LB(); }
  bool turn_signal_right() const { return RB(); }
  bool clear_turn_signal() const { return A(); }

  bool gate_mode() const { return B(); }

  bool emergency_stop() const { return ChangeView(); }
  bool clear_emergency_stop() const { return Menu(); }

  bool autoware_engage() const { return X(); }
  bool autoware_disengage() const { return Y(); }

  bool vehicle_engage() const { return LStickButton(); }
  bool vehicle_disengage() const { return RStickButton(); }

private:
  float LStickLeftRight() const { return j_.axes.at(0); }
  float LStickUpDown() const { return j_.axes.at(1); }
  float RStickLeftRight() const { return j_.axes.at(2); }
  float RStickUpDown() const { return j_.axes.at(3); }
  float RT() const { return j_.axes.at(4); }
  float LT() const { return j_.axes.at(5); }
  float CursorLeftRight() const { return j_.axes.at(6); }
  float CursorUpDown() const { return j_.axes.at(7); }

  bool A() const { return j_.buttons.at(0); }
  bool B() const { return j_.buttons.at(1); }
  bool X() const { return j_.buttons.at(3); }
  bool Y() const { return j_.buttons.at(4); }
  bool LB() const { return j_.buttons.at(6); }
  bool RB() const { return j_.buttons.at(7); }
  bool Menu() const { return j_.buttons.at(11); }
  bool LStickButton() const { return j_.buttons.at(13); }
  bool RStickButton() const { return j_.buttons.at(14); }
  bool ChangeView() const { return j_.buttons.at(15); }
  bool Xbox() const { return j_.buttons.at(16); }

  const sensor_msgs::msg::Joy j_;
};
}  // namespace autoware::joy_controller

#endif  // AUTOWARE__JOY_CONTROLLER__JOY_CONVERTER__XBOX_JOY_CONVERTER_HPP_
