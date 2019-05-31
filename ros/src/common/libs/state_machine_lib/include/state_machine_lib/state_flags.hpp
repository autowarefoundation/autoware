#ifndef __STATE_FLAGS_HPP__
#define __STATE_FLAGS_HPP__

namespace state_machine
{
enum class CallbackType
{
  UPDATE,
  ENTRY,
  EXIT
};

enum TrafficLightColors
{
  E_RED = 0,
  E_YELLOW = 0,
  E_GREEN = 1,
  E_COLOR_ERROR = 2
};
}

#endif
