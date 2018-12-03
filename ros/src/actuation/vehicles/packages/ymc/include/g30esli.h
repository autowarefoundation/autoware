/*
 *  Copyright (c) 2017, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#define G30ESLI_BRAKE_EMERGENCY 2
#define G30ESLI_SHIFT_DRIVE     0
#define G30ESLI_SHIFT_REVERSE   1
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
