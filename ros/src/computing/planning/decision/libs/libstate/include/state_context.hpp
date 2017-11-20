#ifndef __STATE_CONTEXT_HPP__
#define __STATE_CONTEXT_HPP__

#include <atomic>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <utility>

#include "state.hpp"
#include "state_drive.hpp"
#include "state_emg.hpp"
#include "state_main.hpp"

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

  std::map<uint8_t, BaseState **> HolderMap;
  std::unordered_map<uint64_t, BaseState *> StateStores;

  bool enableForceSetState;
  std::queue<uint64_t> ChangeStateFlags;
  std::atomic<bool> thread_loop;

  std::thread *thr_state_dec;
  std::mutex change_state_mutex;

  void showStateMove(uint64_t _state_num)
  {
    std::cout << "State will be [" << StateStores[_state_num]->getStateName() << "]" << std::endl;
  }
  bool setCurrentState(BaseState *state);

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
    StateStores[DRIVE_ACC_STOPLINE_STATE] = DriveAccStopLineState::getInstance();
    StateStores[DRIVE_STR_STRAIGHT_STATE] = DriveStrStraightState::getInstance();
    StateStores[DRIVE_STR_LEFT_STATE] = DriveStrLeftState::getInstance();
    StateStores[DRIVE_STR_RIGHT_STATE] = DriveStrRightState::getInstance();
    StateStores[DRIVE_BEHAVIOR_LANECHANGE_LEFT_STATE] = DriveBehaviorLaneChangeLeftState::getInstance();
    StateStores[DRIVE_BEHAVIOR_LANECHANGE_RIGHT_STATE] = DriveBehaviorLaneChangeRightState::getInstance();
    StateStores[DRIVE_BEHAVIOR_OBSTACLE_AVOIDANCE_STATE] = DriveBehaviorObstacleAvoidanceState::getInstance();
    StateStores[MISSION_COMPLETE_STATE] = MissionCompleteState::getInstance();
    StateStores[EMERGENCY_STATE] = EmergencyState::getInstance();

    HolderMap[MAIN_STATE] = &current_state_.MainState;
    HolderMap[ACC_STATE] = &current_state_.AccState;
    HolderMap[STR_STATE] = &current_state_.StrState;
    HolderMap[BEHAVIOR_STATE] = &current_state_.BehaviorState;
    HolderMap[PERCEPTION_STATE] = &current_state_.PerceptionState;
    HolderMap[OTHER_STATE] = &current_state_.OtherState;

    for (auto &p : HolderMap)
    {
      *p.second = nullptr;
    }
    thread_loop = true;

    this->InitContext();
  }

  ~StateContext()
  {
    thread_loop = false;
  }

  void update(void);
  void changed(uint8_t _kind);
  void stateDecider(void);

  bool isState(BaseState *base, uint64_t _state_num);
  bool isCurrentState(uint64_t _state_num);
  bool isCurrentState(uint8_t _state_kind, uint64_t _state_num);
  bool inState(uint64_t _state_num);

  bool setCurrentState(uint64_t flag);
  bool disableCurrentState(uint64_t);

  BaseState *getCurrentMainState(void);
  BaseState *getCurrentState(void);
  std::string getCurrentStateName(void);
  std::string getStateName(void);
  
  BaseState **getCurrentStateHolderPtr(uint8_t _kind);
  BaseState **getCurrentStateHolderPtr(uint64_t _state_num);
  BaseState **getCurrentStateHolderPtr(BaseState *_state);
  void showCurrentStateName(void);
  std::string createStateMessageText(void);

  uint64_t getStateNum(BaseState *_state);
  uint64_t getStateTransMask(BaseState *_state);
  bool isEmptyMainState(void);
  bool isDifferentState(BaseState *_state_a, BaseState **_state_b);
  uint8_t getStateKind(BaseState *_state);
  bool isMainState(BaseState *_state);

  std::string getCurrentStateName(uint8_t kind);

  bool setEnableForceSetState(bool force_flag);
  BaseState *getStateObject(uint64_t _state_num);
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
