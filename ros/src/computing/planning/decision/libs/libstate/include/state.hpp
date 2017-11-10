#ifndef __STATE_HPP__
#define __STATE_HPP__

#include <iostream>
#include <memory>
#include <vector>
#include <functional>
#include <state_flags.hpp>

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
  virtual void showStateName(void) = 0;
  virtual unsigned long long getStateTransMask(void) = 0;
  virtual unsigned long long getStateNum(void) = 0;
  virtual std::string getStateName(void) = 0;
  virtual unsigned char getStateKind(void) = 0;
  virtual void setCallbackFunc(std::function<void(void)> _f)=0;
};

// Interface
template <class T>
class State : public BaseState
{
protected:
  std::string StateName = "Base";
  unsigned long long StateNum;
  unsigned long long StateTransMask;
  unsigned char StateKind;

  std::function<void(void)> StateCallbackFunc;
  
  State()
  {
    StateNum = 0;
    StateTransMask = (unsigned long long)STATE_END - 1;
    StateKind = UNKNOWN_STATE;
  }

public:
  virtual void update(void){
	std::cout << "update:"<< StateNum << std::endl;
	if(StateCallbackFunc)
		StateCallbackFunc();
  }

  virtual void setCallbackFunc(std::function<void(void)> _f){
	StateCallbackFunc = _f;
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

  unsigned char getStateKind(void)
  {
    return StateKind;
  }

  unsigned long long getStateTransMask(void)
  {
    return StateTransMask;
  }
  unsigned long long getStateNum(void)
  {
    return StateNum;
  }
};
}

#endif
