#ifndef CONSTANTS_H_INCLUDED
#define CONSTANTS_H_INCLUDED

#include <autoware_system_msgs/DiagnosticStatus.h>

namespace autoware_health_checker
{
    constexpr uint8_t LEVEL_UNDEFINED = autoware_system_msgs::DiagnosticStatus::UNDEFINED;
    constexpr uint8_t LEVEL_OK = autoware_system_msgs::DiagnosticStatus::OK;
    constexpr uint8_t LEVEL_WARN = autoware_system_msgs::DiagnosticStatus::WARN;
    constexpr uint8_t LEVEL_ERROR = autoware_system_msgs::DiagnosticStatus::ERROR;
    constexpr uint8_t LEVEL_FATAL = autoware_system_msgs::DiagnosticStatus::FATAL;

    constexpr uint8_t TYPE_UNDEFINED = autoware_system_msgs::DiagnosticStatus::UNDEFINED;
    constexpr uint8_t TYPE_OUT_OF_RANGE = autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE;
    constexpr uint8_t TYPE_RATE_IS_SLOW = autoware_system_msgs::DiagnosticStatus::RATE_IS_SLOW;

    constexpr double BUFFER_LENGTH = 5.0;
    constexpr double UPDATE_RATE = 10.0;
}

#endif //CONSTANTS_H_INCLUDED