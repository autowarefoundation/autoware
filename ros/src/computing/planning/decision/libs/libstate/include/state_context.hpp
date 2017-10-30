#ifndef __STATE_CONTEXT_HPP__
#define __STATE_CONTEXT_HPP__

#include <atomic>
#include <iostream>
#include <queue>
#include <thread>
#include <unordered_map>
#include <utility>
#include "state.hpp"
#include "state_drive.hpp"
#include "state_emg.hpp"

namespace state_machine
{
class StateContext
{
private:
  class StateHolder
  {
  public:
    BaseState *MainState;
    BaseState *AccState;
    BaseState *StrState;
    BaseState *BehaviorState;
    BaseState *PerceptionState;
    BaseState *OtherState;
  } current_state_;

  std::vector<BaseState **> HolderList;

  std::unordered_map<uint64_t, BaseState *> StateStores;

  bool enableForceSetState;
  // unsigned long long ChangeStateFlags;
  std::queue<unsigned long long> ChangeStateFlags;
  std::atomic<bool> thread_loop;

  std::thread *thr_state_dec;

  void showStateMove(unsigned long long _state_num)
  {
    std::cout << "State will be [" << StateStores[_state_num]->getStateName() << "]" << std::endl;
  }

public:
  StateContext(void)
  {
    StateStores[START_STATE] = StartState::getInstance();
    StateStores[INITIAL_STATE] = InitialState::getInstance();
    StateStores[INITIAL_LOCATEVEHICLE_STATE] = LocateVehicleState::getInstance();
    StateStores[DRIVE_STATE] = DriveState::getInstance();
    StateStores[DRIVE_ACC_ACCELERATION_STATE] = DriveAccAccelerationState::getInstance();
    StateStores[DRIVE_ACC_DECELERATION_STATE] = DriveAccDecelerationState::getInstance();
    StateStores[DRIVE_ACC_KEEP_STATE] = DriveAccKeepState::getInstance();
    StateStores[DRIVE_ACC_STOP_STATE] = DriveAccStopState::getInstance();
    StateStores[DRIVE_STR_STRAIGHT_STATE] = DriveStrStraightState::getInstance();
    StateStores[DRIVE_STR_LEFT_STATE] = DriveStrLeftState::getInstance();
    StateStores[DRIVE_STR_RIGHT_STATE] = DriveStrRightState::getInstance();
    StateStores[DRIVE_BEHAVIOR_LANECHANGE_LEFT_STATE] = DriveBehaviorLaneChangeLeftState::getInstance();
    StateStores[DRIVE_BEHAVIOR_LANECHANGE_RIGHT_STATE] = DriveBehaviorLaneChangeRightState::getInstance();
    StateStores[DRIVE_BEHAVIOR_OBSTACLE_AVOIDANCE_STATE] = DriveBehaviorObstacleAvoidanceState::getInstance();
    StateStores[DRIVE_DETECT_OBSTACLE_STATE] = DriveDetectObstacleState::getInstance();
    StateStores[DRIVE_DETECT_STOPLINE_STATE] = DriveDetectStoplineState::getInstance();
    StateStores[DRIVE_DETECT_TRAFFICLIGHT_RED_STATE] = DriveDetectTrafficlightRedState::getInstance();
    StateStores[MISSION_COMPLETE_STATE] = MissionCompleteState::getInstance();
    StateStores[EMERGENCY_STATE] = EmergencyState::getInstance();

    HolderList.push_back(&current_state_.MainState);
    HolderList.push_back(&current_state_.AccState);
    HolderList.push_back(&current_state_.StrState);
    HolderList.push_back(&current_state_.BehaviorState);
    HolderList.push_back(&current_state_.PerceptionState);
    HolderList.push_back(&current_state_.OtherState);

    for (auto &&p : HolderList)
    {
      *p = nullptr;
    }
#if 0
    current_state_.MainState = nullptr;
    current_state_.AccState = nullptr;
    current_state_.StrState = nullptr;
    current_state_.BehaviorState = nullptr;
    current_state_.PerceptionState = nullptr;
    current_state_.OtherState = nullptr;
#endif
    thread_loop = true;

    this->InitContext();
  }
  ~StateContext()
  {
    thread_loop = false;
  }
  void stateDecider(void);

  bool isState(BaseState *base, unsigned long long _state_num);
  bool isCurrentState(unsigned long long _state_num);
  bool isCurrentState(unsigned char _state_kind, unsigned long long _state_num);
  bool inState(unsigned long long _state_num);

  bool setCurrentState(StateFlags flag);
  bool setCurrentState(BaseState *state);
  bool disableCurrentState(unsigned long long);

  BaseState *getCurrentMainState(void);
  BaseState *getCurrentState(void);
  std::string getCurrentStateName(void);
  std::string getStateName(void);
  BaseState **getCurrentStateHolderPtr(unsigned long long _state_num);
  void showCurrentStateName(void);
  std::string createStateMessageText(void);

  bool setEnableForceSetState(bool force_flag);
  BaseState *getStateObject(unsigned long long _state_num);
  void InitContext(void);

  bool TFInitialized(void);

  void handleTrafficLight(uint32_t _light_color);
  bool handleCurrentPose(double x, double y, double z, double roll, double pitch, double yaw);
  bool handlePointsRaw(bool _hasLidarData);

  bool handleIntersection(bool _hasIntersection, double _angle);
  bool handleTwistCmd(bool _hasTwistCmd);
};
}

#endif
