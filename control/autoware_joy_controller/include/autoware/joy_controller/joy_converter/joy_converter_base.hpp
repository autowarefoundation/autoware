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

#ifndef AUTOWARE__JOY_CONTROLLER__JOY_CONVERTER__JOY_CONVERTER_BASE_HPP_
#define AUTOWARE__JOY_CONTROLLER__JOY_CONVERTER__JOY_CONVERTER_BASE_HPP_

#include <sensor_msgs/msg/joy.hpp>

namespace autoware::joy_controller
{
class JoyConverterBase
{
public:
  virtual float accel() const = 0;

  virtual float brake() const = 0;

  virtual float steer() const = 0;

  virtual bool shift_up() const = 0;
  virtual bool shift_down() const = 0;
  virtual bool shift_drive() const = 0;
  virtual bool shift_reverse() const = 0;

  virtual bool turn_signal_left() const = 0;
  virtual bool turn_signal_right() const = 0;
  virtual bool clear_turn_signal() const = 0;

  virtual bool gate_mode() const = 0;

  virtual bool emergency_stop() const = 0;
  virtual bool clear_emergency_stop() const = 0;

  virtual bool autoware_engage() const = 0;
  virtual bool autoware_disengage() const = 0;

  virtual bool vehicle_engage() const = 0;
  virtual bool vehicle_disengage() const = 0;

  virtual ~JoyConverterBase() = default;
};
}  // namespace autoware::joy_controller

#endif  // AUTOWARE__JOY_CONTROLLER__JOY_CONVERTER__JOY_CONVERTER_BASE_HPP_
