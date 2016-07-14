/*
 * PlannerXInterface.cpp
 *
 *  Created on: Jul 11, 2016
 *      Author: ai-driver
 */

#include "PlannerXInterface.h"

namespace PlannerXNS
{

PlannerX_Interface::PlannerX_Interface()
{
	bMap = false;
	bGoal = false;
	bPredefinedPath = false;
	bOriginPoint = false;
}

PlannerX_Interface::~PlannerX_Interface()
{
}

} /* namespace PlannerHNS */
