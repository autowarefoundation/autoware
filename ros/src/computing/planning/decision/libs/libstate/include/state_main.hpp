#ifndef __STATE_MAIN_HPP__
#define __STATE_MAIN_HPP__

#include <iostream>
#include <memory>
#include <vector>

#include <state.hpp>

namespace state_machine
{
// StartState
class StartState : public State<StartState>
{
private:
  friend class State<StartState>;
  StartState(void)
  {
    StateName = "Start";
    StateNum = START_STATE;
    StateTransMask = (uint64_t)STATE_END - 1;
    StateKind = MAIN_STATE;
  }

public:
  virtual void update(void) override
  {
  }
};

class InitialState : public State<InitialState>
{
private:
  friend class State<InitialState>;
  InitialState(void)
  {
    StateName = "Initial";
    StateNum = StateTransMask = INITIAL_STATE;
    StateTransMask |= START_STATE | EMERGENCY_STATE | MISSION_COMPLETE_STATE;
    StateKind = MAIN_STATE;
  }

public:
};

class LocateVehicleState : public State<LocateVehicleState>
{
private:
  friend class State<LocateVehicleState>;
  LocateVehicleState(void)
  {
    StateName = "Locate Vehicle";
    StateNum = StateTransMask = LOCATEVEHICLE_STATE;
    StateTransMask |= INITIAL_STATE;
    StateKind = MAIN_STATE;
  }

public:
};

class VehicleReadyState : public State<VehicleReadyState>
{
private:
  friend class State<VehicleReadyState>;
  VehicleReadyState(void)
  {
    StateName = "Vehicle Ready";
    StateNum = StateTransMask = VEHICLE_READY_STATE;
    StateTransMask |= LOCATEVEHICLE_STATE;
    StateKind = MAIN_STATE;
  }

public:
};



class DriveReadyState : public State<DriveReadyState>
{
private:
  friend class State<DriveReadyState>;
  DriveReadyState(void)
  {
    StateName = "Drive Ready";
    StateNum = StateTransMask = DRIVE_READY_STATE;
    StateTransMask |= VEHICLE_READY_STATE;
    StateKind = MAIN_STATE;
  }

public:
};

class DriveState : public State<DriveState>
{
private:
  friend class State<DriveState>;

  DriveState(void)
  {
    StateName = "Drive";
    StateNum = StateTransMask = DRIVE_STATE;
    StateTransMask |= DRIVE_READY_STATE;
    StateKind = MAIN_STATE;
  }

public:
};

class MissionCompleteState : public State<MissionCompleteState>
{
private:
  friend class State<MissionCompleteState>;
  MissionCompleteState(void)
  {
    StateName = "MissionComplete";
    StateNum = MISSION_COMPLETE_STATE;
    StateTransMask = DRIVE_STATE;
    StateKind = MAIN_STATE;
  }

public:
};

class EmergencyState : public State<EmergencyState>
{
private:
  friend class State<EmergencyState>;
  EmergencyState(void)
  {
    StateName = "Emergency";
    StateTransMask = 0;
  }

public:
};
}

#endif
