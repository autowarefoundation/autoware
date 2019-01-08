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

#ifndef G30ESLI_INTERFACE_UTIL_H
#define G30ESLI_INTERFACE_UTIL_H

#include <vector>
#include <string>
#include <sstream>
#include <cstring>
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>

namespace ymc
{
void reverse_byteorder_memcpy(unsigned char* target, int16_t* source, size_t size)
{
  for (unsigned int i = 0; i < size; i++)
  {
    unsigned char* address = (unsigned char*)source + (i * sizeof(unsigned char));
    std::memcpy(&target[size - i - 1], address, sizeof(unsigned char));
  }
}

// split string with whitespace
std::vector<std::string> splitString(const std::string& s)
{
  std::stringstream ss(s.c_str());
  std::string each_s;

  std::vector<std::string> result;
  while (getline(ss, each_s, ' '))
  {
    result.emplace_back(each_s);
  }

  // remove empty elements
  result.erase(std::remove_if(result.begin(), result.end(), [](const std::string& s) { return s.empty(); }),
               result.end());

  return result;
}

// detect hit of keyboard
int kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

double computeTargetSteeringAngleDegree(double angular_z, double velocity_mps, double wheel_base)
{
  double steering_angle_rad;
  static double prev_steering_angle_rad = 0.0;

  if (velocity_mps == 0)
  {
    // steering_angle_rad = 0;
    steering_angle_rad = prev_steering_angle_rad;
  }
  else
  {
    steering_angle_rad = std::asin(wheel_base * angular_z / velocity_mps);  // radian
  }

  prev_steering_angle_rad = steering_angle_rad;

  double steering_angle_degree = steering_angle_rad / M_PI * 180.0;

  return steering_angle_degree;
}

}  // namespace ymc

#endif
