#ifndef CONSTANTS_H_INCLUDED
#define CONSTANTS_H_INCLUDED

/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 *
 * v1.0 Masaya Kataoka
 */

#include <autoware_system_msgs/DiagnosticStatus.h>

namespace autoware_health_checker {
constexpr uint8_t LEVEL_UNDEFINED =
    autoware_system_msgs::DiagnosticStatus::UNDEFINED;
constexpr uint8_t LEVEL_OK = autoware_system_msgs::DiagnosticStatus::OK;
constexpr uint8_t LEVEL_WARN = autoware_system_msgs::DiagnosticStatus::WARN;
constexpr uint8_t LEVEL_ERROR = autoware_system_msgs::DiagnosticStatus::ERROR;
constexpr uint8_t LEVEL_FATAL = autoware_system_msgs::DiagnosticStatus::FATAL;

constexpr uint8_t TYPE_UNDEFINED =
    autoware_system_msgs::DiagnosticStatus::UNDEFINED;
constexpr uint8_t TYPE_OUT_OF_RANGE =
    autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE;
constexpr uint8_t TYPE_RATE_IS_SLOW =
    autoware_system_msgs::DiagnosticStatus::RATE_IS_SLOW;

constexpr double BUFFER_LENGTH = 5.0;
constexpr double UPDATE_RATE = 10.0;
}

#endif // CONSTANTS_H_INCLUDED