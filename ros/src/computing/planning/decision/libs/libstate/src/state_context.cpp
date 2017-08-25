#include <sched.h>
#include <sys/types.h>
#include <unistd.h>
#include <thread>
#include <vector>

#include <cassert>

#include <euclidean_space.hpp>
#include <state_context.hpp>

#include <state_common.hpp>
#include <state.hpp>

using namespace std;

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
  state_->ShowStateName();
  if (sub_state)
    sub_state->ShowStateName();
  if (sub_sub_state)
    sub_sub_state->ShowStateName();
  std::cout << std::endl;
}

std::unique_ptr<std::string> StateContext::getCurrentStateName(void)
{
  return state_->GetStateName();
}

/**
 * @fn
 * set to current state, substate, subsubstate
 * @brief standard out a state name
 * @param (state) Setting class
 * @return void
 */
bool StateContext::setCurrentState(BaseState *_state)
{
  return _state ? this->setCurrentState(_state, nullptr, nullptr) : false;
}

bool StateContext::setCurrentState(BaseState *_state, BaseState *_substate)
{
  return _state ? this->setCurrentState(_state, _substate, nullptr) : false;
}

bool StateContext::setCurrentState(BaseState *_state, BaseState *_substate, BaseState *_subsubstate)
{
	BaseState *prevState = state_;

	if (!state_)
	{
		state_ = _state;

		std::cout << "Successed to set state \""
			<< "NULL"
			<< "\" to \"" << *_state->GetStateName().get() << "\" : Mask is [" << _state->GetStateTransMask() << "/"
			<< "NULL"
			<< "-" << _state->GetStateNum() << "]" << std::endl;
	}
	else
	{
		if (_state && (_state->GetStateTransMask() & state_->GetStateNum()))
		{

			if(!(state_ == _state)){  
				state_ = _state;
				sub_state = _substate;
				sub_sub_state = _subsubstate;

				std::cout << "Successed to set state \"" << *prevState->GetStateName().get() << "\" to \""
					<< *_state->GetStateName().get() << "\" : Mask is [" << _state->GetStateTransMask() << "/"
					<< prevState->GetStateNum() << "-" << _state->GetStateNum() << "]" << std::endl;
			}
		}
		else
		{
			std::cerr << "Failed to set state \"" << *state_->GetStateName().get() << "\" to \""
				<< *_state->GetStateName().get() << "\" : Mask is [" << _state->GetStateTransMask() << "/"
				<< state_->GetStateNum() << "-" << _state->GetStateNum() << "]" << std::endl;
			prevState = nullptr;
			return false;
		}
	}
	return true;
}

BaseState *StateContext::getCurrentState(void)
{
  return state_;
}

BaseState *StateContext::getStateObject(unsigned long long _state_num)
{
  return StateStores[_state_num];
}
bool StateContext::isState(unsigned long long _state_num)
{
  return state_->GetStateNum() == _state_num;
}

bool StateContext::inState(unsigned long long _state_num)
{
  return state_->GetStateNum() & _state_num;
}

void StateContext::handleTrafficLight(uint32_t _light_color)
{
  switch (_light_color)
  {
    case RED:
    case YELLOW:
      setCurrentState(StateStores[DRIVE_STOP_TRAFFICLIGHT_STATE]);
      break;
    case GREEN:
      break;

    default:
      break;
  }
}



#define ANGLE_STRAIGHT 50.0
#define ANGLE_LEFT  360.0
#define ANGLE_RIGHT 180.0
bool StateContext::handleIntersection(bool _hasIntersection, double _angle)
{
	if (_hasIntersection){
		// *Temporary implementation*
		// To straight/left/right recognition by using angle between
		// first-waypoint
		// and end-waypoint in intersection area.
		int temp = (int)std::floor(_angle+360.0) % 360;

		if (std::abs(temp) <= ANGLE_STRAIGHT)
			return this->setCurrentState(StateStores[DRIVE_MOVEFWD_STRAIGHT_STATE]);
		else if (temp <= ANGLE_RIGHT)
			return this->setCurrentState(StateStores[DRIVE_MOVEFWD_RIGHT_STATE]);
		else if (temp <= ANGLE_LEFT)
			return this->setCurrentState(StateStores[DRIVE_MOVEFWD_LEFT_STATE]);
		else return false;
	}else{
		return false;
	}
}

bool StateContext::handleTwistCmd(bool _hasTwistCmd)
{
  if (_hasTwistCmd)
    return this->setCurrentState(StateStores[DRIVE_MOVEFWD_STATE]);
  else
    return false;
}

bool StateContext::handlePointsRaw(bool _hasLidarData)
{
  return _hasLidarData ? this->setCurrentState(StateStores[INITIAL_LOCATEVEHICLE_STATE]) : false;
}

#define CONV_NUM 10
#define CONVERGENCE_THRESHOLD 0.01

bool StateContext::handleCurrentPose(double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
{
  static int _init_count = 0;
  static euclidean_space::point *a = new euclidean_space::point();
  static euclidean_space::point *b = new euclidean_space::point();

  static double distances[CONV_NUM] = { 0.0 };
  double avg_distances = 0.0;

  for (int i = 1; i < CONV_NUM; i++)
  {
    distances[i] = distances[i - 1];
    avg_distances += distances[i];
  }

  a->x = b->x;
  a->y = b->y;
  a->z = b->z;

  b->x = _x;
  b->y = _y;
  b->z = _z;

  distances[0] = euclidean_space::EuclideanSpace::find_distance(a, b);

  if (++_init_count <= CONV_NUM)
  {
    return false;
  }
  else
  {
    avg_distances = (avg_distances + distances[0]) / CONV_NUM;
    
    if (avg_distances <= CONVERGENCE_THRESHOLD)
      return this->setCurrentState(StateStores[DRIVE_STATE]);
    else
      return false;
  }
}

void StateContext::StateDecider(void)
{
  while (thread_loop)
  {
    if (ChangeStateFlags)
      for (unsigned long long l = 1; l < STATE_END; l = l * 2)
      {
        if (ChangeStateFlags & l && StateStores[l])
        {
          setCurrentState(StateStores[l]);
          ChangeStateFlags -= l;
        }
      }
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }
  std::cerr << "StateDecider thread will be closed" << std::endl;
  return;
}

void StateContext::InitContext(void)
{
  thr_state_dec = new std::thread(&StateContext::StateDecider, this);
  thr_state_dec->detach();
  this->setCurrentState(StateStores[START_STATE]);

  return;
}
bool StateContext::TFInitialized(void)
{
  return this->setCurrentState(StateStores[INITIAL_STATE]);
}
}
