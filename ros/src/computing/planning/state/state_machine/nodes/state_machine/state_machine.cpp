/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "state_machine.h"

namespace state_machine
{
void StateTrafficLightStop::update(StateContext *context)
{
  if (context->getLightColor() == TrafficLight::GREEN)
    context->setState(StateMoveForward::create());
}

void StateMoveForward::update(StateContext *context)
{
  if (context->getLightColor() == TrafficLight::RED)
    context->setState(StateTrafficLightStop::create());

  if(context->getChangeFlag() == ChangeFlag::right || context->getChangeFlag() == ChangeFlag::left)
    context->setState(StateLaneChange::create());
}

void StateLaneChange::update(StateContext *context)
{
  if(context->getChangeFlag() == ChangeFlag::straight)
    context->setState(StateMoveForward::create());
}

void StateStopSignStop::update(StateContext *context)
{
  // stop sign stop
}

void StateMissionComplete::update(StateContext *context)
{
  // Mission complete
}

void StateEmergency::update(StateContext *context)
{
  // Emergency
}

void StateObstacleAvoidance::update(StateContext *context)
{
  // Obstacle Avoidance
}



}  // state_machine