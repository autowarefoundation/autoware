#ifndef __STATE_EMG_HPP__
#define __STATE_EMG_HPP__

#include "state.hpp"

namespace state_machine
{
class EmergencyHWState : public State<EmergencyHWState>
{
private:
  friend class State<EmergencyHWState>;
  EmergencyHWState(void)
  {
    StateName = "EmergencyHW";
    StateTransMask = 0;
  }

public:
};
class EmergencyHWVehicleState : public State<EmergencyHWVehicleState>
{
private:
  friend class State<EmergencyHWVehicleState>;
  EmergencyHWVehicleState(void)
  {
    StateName = "EmergencyHWVehicle";
    StateTransMask = 0;
  }

public:
};
class EmergencyHWControllerState : public State<EmergencyHWControllerState>
{
private:
  friend class State<EmergencyHWControllerState>;
  EmergencyHWControllerState(void)
  {
    StateName = "EmergencyHWController";
    StateTransMask = 0;
  }

public:
};
class EmergencySWState : public State<EmergencySWState>
{
private:
  friend class State<EmergencySWState>;
  EmergencySWState(void)
  {
    StateName = "EmergencySW";
    StateTransMask = 0;
  }

public:
};
class EmergencySWAutowareState : public State<EmergencySWAutowareState>
{
private:
  friend class State<EmergencySWAutowareState>;
  EmergencySWAutowareState(void)
  {
    StateName = "EmergencySWAutoware";
    StateTransMask = 0;
  }

public:
};
class EmergencySWControllerState : public State<EmergencySWControllerState>
{
private:
  friend class State<EmergencySWControllerState>;
  EmergencySWControllerState(void)
  {
    StateName = "EmergencySWController";
    StateTransMask = 0;
  }

public:
};
}

#endif
