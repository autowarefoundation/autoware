#ifndef __STATE_HPP__
#define __STATE_HPP__

#include <functional>
#include <iostream>
#include <memory>
#include <state_machine_lib/state_flags.hpp>
#include <vector>

namespace state_machine
{
class StartState;
class InitialState;
class LocateVehicleState;

class DriveState;
// Accel/Brake subState
class DriveAccAccelerationState;
class DriveAccDecelerationState;
class DriveAccKeepState;
class DriveAccStopState;
// Steering subState
class DriveStrStraightState;
class DriveStrLeftTurnState;
class DriveStrRightTurnState;

// Behavior subState
class DriveBehaviorLaneChangeLeftState;
class DriveBehaviorLaneChangeRightState;
class DriveBehaviorObstacleAvoidanceState;

// Perception subState
class DriveDetectObstacleState;
class DriveDetectStoplineState;
class DriveDetectTrafficlightRedState;

class MissionCompleteState;
class EmergencyState;

// base class
class BaseState
{
protected:
  BaseState()
  {
  }

public:
  virtual void update(void) = 0;
  virtual void inState(void) = 0;
  virtual void outState(void) = 0;
  virtual void showStateName(void) = 0;
  virtual uint64_t getStateTransMask(void) = 0;
  virtual uint64_t getStateNum(void) = 0;
  virtual std::string getStateName(void) = 0;
  virtual uint8_t getStateKind(void) = 0;
  virtual void setCallbackUpdateFunc(std::function<void(void)> _f) = 0;
  virtual void setCallbackInFunc(std::function<void(void)> _f) = 0;
  virtual void setCallbackOutFunc(std::function<void(void)> _f) = 0;
};

// Interface
template <class T>
class State : public BaseState
{
protected:
  std::string StateName = "Base";
  uint64_t StateNum;
  uint64_t StateTransMask;
  uint8_t StateKind;

  std::function<void(void)> StateCallbackUpdateFunc;
  std::function<void(void)> StateCallbackInFunc;
  std::function<void(void)> StateCallbackOutFunc;

  State()
  {
    StateNum = 0;
    StateTransMask = (uint64_t)STATE_END - 1;
    StateKind = UNKNOWN_STATE;
  }

public:
  virtual void update(void)
  {
    if (StateCallbackUpdateFunc)
      StateCallbackUpdateFunc();
  }

  virtual void inState(void)
  {
    if (StateCallbackInFunc)
      StateCallbackInFunc();
  }
  virtual void outState(void)
  {
    if (StateCallbackOutFunc)
      StateCallbackOutFunc();
  }
  virtual void setCallbackUpdateFunc(std::function<void(void)> _f)
  {
    StateCallbackUpdateFunc = _f;
  }

  virtual void setCallbackOutFunc(std::function<void(void)> _f)
  {
    StateCallbackOutFunc = _f;
  }

  virtual void setCallbackInFunc(std::function<void(void)> _f)
  {
    StateCallbackInFunc = _f;
  }

  void showStateName(void)
  {
    std::cout << StateName << "-";
  }

  static T* getInstance(void)
  {
    static T singleton;
    return &singleton;
  }

  std::string getStateName(void)
  {
    return std::string(StateName);
  }

  uint8_t getStateKind(void)
  {
    return StateKind;
  }

  uint64_t getStateTransMask(void)
  {
    return StateTransMask;
  }
  uint64_t getStateNum(void)
  {
    return StateNum;
  }
};
}

#endif
