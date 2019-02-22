/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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

#ifndef G30ESLI_H
#define G30ESLI_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdint>
#include <cstring>

#include <cansend.h>

#define G30ESLI_WHEEL_BASE      2.14
#define G30ESLI_MODE_MANUAL     3
#define G30ESLI_MODE_AUTO       8
#define G30ESLI_BRAKE_NONE      0
#define G30ESLI_BRAKE_SMOOTH    1
#define G30ESLI_BRAKE_SEMIEMG   2
#define G30ESLI_BRAKE_EMERGENCY 3
#define G30ESLI_SHIFT_DRIVE     0
#define G30ESLI_SHIFT_REVERSE   1
#define G30ESLI_SHIFT_NEUTRAL   2
#define G30ESLI_FLASHER_NONE    0
#define G30ESLI_FLASHER_RIGHT   1
#define G30ESLI_FLASHER_LEFT    2
#define G30ESLI_FLASHER_HAZARD  3
#define G30ESLI_FLASHER_CLEAR   4

namespace ymc
{
class G30esli
{
private:
  FILE* dev_;
  CanSender sender_;

public:
  struct Command
  {
    float         speed   = 0.0;
    float         steer   = 0.0;
    unsigned char mode    = 3;
    unsigned char brake   = 0;
    unsigned char shift   = 0;
    unsigned char alive   = 0;
    unsigned char turn    = 0;
    unsigned char flasher = 0;
  };

  struct SpeedStatus
  {
    float target;
    float actual;
    float desired;
  };

  struct SteerStatus
  {
    float target;
    float actual;
  };

  struct BatteryStatus
  {
    float charge;
    float voltage;
    float amperage;
  };

  struct WarningStatus
  {
    bool warning[21];
  };

  struct OverrideStatus
  {
    unsigned short  steer;
    unsigned char   accel;
    unsigned char   brake;
    unsigned char   flasher;
  };

  struct Status
  {
    SpeedStatus     speed;
    SteerStatus     steer;
    BatteryStatus   battery;
    WarningStatus   warning;
    OverrideStatus  override;
    unsigned char   mode;
    unsigned char   shift;
    unsigned char   brake;
    unsigned char   turn;
  };

  G30esli();
  ~G30esli();

  bool openDevice(const std::string& device);
  void sendCommand(const Command& command);
  void readStatus(Status& status);

  static std::string dumpCommand(const Command& command);
  static std::string dumpStatus(const Status& status);
};

} // namespace

#endif  // G30ESLI_H
