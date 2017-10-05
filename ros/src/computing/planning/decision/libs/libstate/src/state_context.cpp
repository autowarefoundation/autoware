#include <sched.h>
#include <sys/types.h>
#include <unistd.h>
#include <thread>
#include <vector>

#include <cassert>

#include <euclidean_space.hpp>
#include <state_context.hpp>

#include <state.hpp>
#include <state_common.hpp>

/**
 *
 * @file StateContext.cpp
 * @brief state context class
 * @author Yusuke Fujii
 * @date  2017/07/25
 *
 **/

namespace state_machine
{
/**
 * @fn
 *
 * @brief
 * @return
 */
void StateContext::showCurrentStateName(void)
{
  for (auto &&p : HolderList)
  {
    if (*p)
    {
      (*p)->showStateName();
    }
  }

#if 0
  if (sub_state)
    sub_state->showStateName();
  if (sub_sub_state)
    sub_sub_state->showStateName();
#endif
  std::cout << std::endl;
}

std::string StateContext::getCurrentStateName(void)
{
  return current_state_.MainState->getStateName();
}

/**
 * @fn
 * set to current state
 * @brief standard out a state name
 * @param (state) Setting class
 * @return void
 */
bool StateContext::setCurrentState(BaseState *_state)
{
  BaseState *prevState = current_state_.MainState;

  if (!prevState)
  {
    current_state_.MainState = _state;
    std::cout << "Successed to set state \""
              << "NULL"
              << "\" to \"" << _state->getStateName() << "\" : Mask is [" << _state->getStateTransMask() << "/"
              << "NULL"
              << "-" << _state->getStateNum() << "]" << std::endl;
  }
  else
  {
    if (_state && (enableForceSetState || (_state->getStateTransMask() & prevState->getStateNum())))
    {
      switch (_state->getStateKind())
      {
        case MAIN_STATE:
          current_state_.MainState = _state;
          current_state_.AccState = nullptr;
          current_state_.StrState = nullptr;
          current_state_.BehaviorState = nullptr;
          current_state_.PerceptionState = nullptr;
          current_state_.OtherState = nullptr;
          break;
        case ACC_STATE:
          current_state_.AccState = _state;
          break;
        case STR_STATE:
          current_state_.StrState = _state;
          break;
        case BEHAVIOR_STATE:
          current_state_.BehaviorState = _state;
          break;
        case PERCEPTION_STATE:
          current_state_.PerceptionState = _state;
          break;
        case OTHER_STATE:
          current_state_.OtherState = _state;
          break;
      }
      std::cout << "Successed to set state \"" << prevState->getStateName() << "\" to \"" << _state->getStateName()
                << "\" : Mask is [" << _state->getStateTransMask() << "/" << prevState->getStateNum() << "-"
                << _state->getStateNum() << "]" << std::endl;
    }
    else
    {
      std::cerr << "Failed to set state \"" << current_state_.MainState->getStateName() << "\" to \""
                << _state->getStateName() << "\" : Mask is [" << _state->getStateTransMask() << "/"
                << current_state_.MainState->getStateNum() << "-" << _state->getStateNum() << "]" << std::endl;
      prevState = nullptr;
      return false;
    }
  }
  return true;
}

bool StateContext::setCurrentState(StateFlags flag)
{
  this->setCurrentState(StateStores[flag]);
}

bool StateContext::setEnableForceSetState(bool force_flag)
{
  enableForceSetState = force_flag;
  return true;
}

BaseState *StateContext::getCurrentMainState(void)
{
  return current_state_.MainState;
}

BaseState *StateContext::getStateObject(unsigned long long _state_num)
{
  return StateStores[_state_num];
}

BaseState **StateContext::getCurrentStateHolderPtr(unsigned long long _state_num)
{
  BaseState **state_ptr;
  switch (getStateObject(_state_num)->getStateKind())
  {
    case MAIN_STATE:
      state_ptr = &current_state_.MainState;
      break;
    case ACC_STATE:
      state_ptr = &current_state_.AccState;
      break;
    case STR_STATE:
      state_ptr = &current_state_.StrState;
      break;
    case BEHAVIOR_STATE:
      state_ptr = &current_state_.BehaviorState;
      break;
    case PERCEPTION_STATE:
      state_ptr = &current_state_.PerceptionState;
      break;
    case OTHER_STATE:
      state_ptr = &current_state_.OtherState;
      break;
    default:
      state_ptr = nullptr;
      break;
  }
  return state_ptr;
}

bool StateContext::disableCurrentState(unsigned long long _state_num)
{
  BaseState **state_ptr = getCurrentStateHolderPtr(_state_num);
  if (state_ptr && this->isState((*state_ptr), _state_num))
  {
    *state_ptr = nullptr;
    return true;
  }
  else
  {
    return false;
  }
}

bool StateContext::isCurrentState(unsigned long long _state_num)
{
  BaseState **state_ptr = getCurrentStateHolderPtr(_state_num);
  return (*state_ptr) ? (*state_ptr)->getStateNum() == _state_num ? true : false : false;
  // return current_state_.MainState?current_state_.MainState->getStateNum() == _state_num?true:false:false;
}

bool StateContext::isState(BaseState *base, unsigned long long _state_num)
{
  return base ? base->getStateNum() == _state_num ? true : false : false;
}

bool StateContext::inState(unsigned long long _state_num)
{
  if (current_state_.MainState)
  {
    return ((current_state_.MainState->getStateNum() & _state_num) != 0) ? true : false;
  }
  else
  {
    return false;
  }
}

#define ANGLE_STRAIGHT 50.0
#define ANGLE_LEFT 360.0
#define ANGLE_RIGHT 180.0
bool StateContext::handleIntersection(bool _hasIntersection, double _angle)
{
  if (_hasIntersection)
  {
    // *Temporary implementation*
    // To straight/left/right recognition by using angle between
    // first-waypoint
    // and end-waypoint in intersection area.
    int temp = (int)std::floor(_angle + 360.0) % 360;
#if 0
    if (std::abs(temp) <= ANGLE_STRAIGHT)
      return this->setCurrentState(StateStores[DRIVE_MOVEFWD_STRAIGHT_STATE]);
    else if (temp <= ANGLE_RIGHT)
      return this->setCurrentState(StateStores[DRIVE_MOVEFWD_RIGHT_STATE]);
    else if (temp <= ANGLE_LEFT)
      return this->setCurrentState(StateStores[DRIVE_MOVEFWD_LEFT_STATE]);
    else
      return false;
#endif
  }
  else
  {
    return false;
  }
}

std::string StateContext::createStateMessageText(void)
{
  std::string ret;

  for (auto &&p : HolderList)
  {
    if (*p)
    {
      ret = ret + "\n" + (*p)->getStateName();
    }
  }
  return ret;
}

bool StateContext::handleTwistCmd(bool _hasTwistCmd)
{
  if (_hasTwistCmd)
    return this->setCurrentState(StateStores[DRIVE_STATE]);
  else
    return false;
}

void StateContext::stateDecider(void)
{
  // not running
  while (thread_loop)
  {
    if (!ChangeStateFlags.empty())
    {
      this->setCurrentState(StateStores[ChangeStateFlags.front()]);
      ChangeStateFlags.pop();
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }
  std::cerr << "StateDecider thread will be closed" << std::endl;
  return;
}

void StateContext::InitContext(void)
{
  thr_state_dec = new std::thread(&StateContext::stateDecider, this);
  thr_state_dec->detach();
  this->setCurrentState(StateStores[START_STATE]);
  return;
}
bool StateContext::TFInitialized(void)
{
  return this->setCurrentState(StateStores[INITIAL_STATE]);
}
}
