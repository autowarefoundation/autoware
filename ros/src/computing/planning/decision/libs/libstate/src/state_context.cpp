#include <sched.h>
#include <sys/types.h>
#include <unistd.h>
#include <thread>
#include <vector>

#include <cassert>
#include <mutex>

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
void StateContext::update(void)
{
  for (auto &p : HolderMap)
  {
    if (*p.second)
    {
      (*p.second)->update();
    }
  }
}

void StateContext::showCurrentStateName(void)
{
  for (auto &p : HolderMap)
  {
    if (*p.second)
    {
      (*p.second)->showStateName();
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

bool StateContext::isDifferentState(BaseState *_state_a, BaseState *_state_b)
{
  return _state_a == _state_b;
}

bool StateContext::isEmptyMainState(void)
{
  if (current_state_.MainState)
    return false;
  return true;
}

uint8_t StateContext::getStateKind(BaseState *_state)
{
  if (_state)
    return _state->getStateKind();
  else
    return NULL_STATE;
}

uint64_t StateContext::getStateTransMask(BaseState *_state)
{
  if (_state)
    return _state->getStateTransMask();
  else
    return 0;
}

uint64_t StateContext::getStateNum(BaseState *_state)
{
  if (_state)
    return _state->getStateNum();
  else
    return 0;
}

bool StateContext::isMainState(BaseState *_state)
{
  return getStateKind(_state) == MAIN_STATE;
}

bool StateContext::setCurrentState(BaseState *_state)
{
  change_state_mutex.lock();

  bool ret = true;

  if (isMainState(_state))
  {
    if (isEmptyMainState() || enableForceSetState ||
        (getStateTransMask(_state) & getStateNum(current_state_.MainState)))
    {
      current_state_.MainState = _state;
      current_state_.AccState = nullptr;
      current_state_.StrState = nullptr;
      current_state_.BehaviorState = nullptr;
      current_state_.PerceptionState = nullptr;
      current_state_.OtherState = nullptr;
    }
    else
    {
      ret = false;
    }
  }
  else
  {
    *HolderMap[getStateKind(_state)] = _state;
  }
  change_state_mutex.unlock();
  return ret;
}

bool StateContext::setCurrentState(uint64_t flag)
{
  bool ret = this->setCurrentState(StateStores[flag]);
  return ret;
}

bool StateContext::setEnableForceSetState(bool force_flag)
{
  enableForceSetState = force_flag;
  return true;
}

std::string StateContext::getCurrentStateName(uint8_t kind)
{
  if (*HolderMap[kind])
    return (*HolderMap[kind])->getStateName();
  return std::string("");
}
std::string StateContext::getCurrentStateName(void)
{
  return this->getCurrentStateName(MAIN_STATE);
}

BaseState *StateContext::getCurrentMainState(void)
{
  return current_state_.MainState;
}

BaseState *StateContext::getStateObject(uint64_t _state_num)
{
  return StateStores[_state_num];
}

BaseState **StateContext::getCurrentStateHolderPtr(uint64_t _state_num)
{
  BaseState **state_ptr;
  switch (getStateKind(getStateObject(_state_num)))
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

bool StateContext::disableCurrentState(uint64_t _state_num)
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

bool StateContext::isCurrentState(uint64_t _state_num)
{
  BaseState **state_ptr = getCurrentStateHolderPtr(_state_num);
  return (*state_ptr) ? (*state_ptr)->getStateNum() & _state_num ? true : false : false;
}

bool StateContext::isState(BaseState *base, uint64_t _state_num)
{
  return base ? base->getStateNum() & _state_num ? true : false : false;
}

bool StateContext::inState(uint64_t _state_num)
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

bool StateContext::handleIntersection(bool _hasIntersection, double _angle)
{
  /* deprecated */
  return false;
}

std::string StateContext::createStateMessageText(void)
{
  std::string ret;

  for (auto &p : HolderMap)
  {
    if (*p.second)
    {
      ret = ret + "\n" + (*p.second)->getStateName();
    }
  }
  return ret;
}

bool StateContext::handleTwistCmd(bool _hasTwistCmd)
{
  if (_hasTwistCmd)
    return this->setCurrentState(DRIVE_STATE);
  else
    return false;
}

void StateContext::stateDecider(void)
{
  // not running
}

void StateContext::InitContext(void)
{
  thr_state_dec = new std::thread(&StateContext::stateDecider, this);
  thr_state_dec->detach();
  this->setCurrentState(START_STATE);
  return;
}
bool StateContext::TFInitialized(void)
{
  return this->setCurrentState(INITIAL_STATE);
}
}
