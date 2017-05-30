/*
 * CommonSimuDefinitions.h
 *
 *  Created on: Jun 20, 2016
 *      Author: hatem
 */

#ifndef COMMONSIMUDEFINITIONS_H_
#define COMMONSIMUDEFINITIONS_H_

#include <math.h>
#include <string>

namespace SimulationNS
{

enum CAR_TYPE
{
  Mv2Car, //!< Mv2Car
  PHVCar, //!< PHVCar
  HVCar,  //!< HVCar
  RoboCar,//!< RoboCar
  SimulationCar
};

class PID_CONST
{
public:
	double kP;
	double kI;
	double kD;

	PID_CONST()
	{
		kP = kI = kD = 0;
	}

	PID_CONST(const double& p, const double& i, const double& d)
	{
		kP = p;
		kI = i;
		kD = d;
	}
};

class ControllerParams
{
public:
	double SimulationSteeringDelay;
	double SteeringDelay;
	double minPursuiteDistance;
	PID_CONST Steering_Gain;
	PID_CONST Velocity_Gain;
	double Acceleration;
	double Deceleration;
	double FollowDistance;
	double LowpassSteerCutoff;
	double maxAccel;
	double maxDecel;


	ControllerParams()
	{
		SimulationSteeringDelay = 0.0;
		SteeringDelay 		= 0.8;
		Acceleration		= 0.05;
		Deceleration		= 1.6;
		FollowDistance		= 8.0;
		LowpassSteerCutoff	= 5.0;
		maxAccel			= 0.7;
		minPursuiteDistance = 2.0;
		maxDecel 			= 0.5;
	}
};

class CAR_BASIC_INFO
{
public:
  CAR_TYPE model;

  double turning_radius;
  double wheel_base;
  double max_speed_forward;
  double max_speed_backword;
  double max_steer_value;
  double min_steer_value;
  double max_brake_value;
  double min_brake_value;
  double max_steer_angle;
  double min_steer_angle;
  double length;
  double width;

  CAR_BASIC_INFO()
  {
	  model 				= SimulationCar;
	  turning_radius 		= 5.2;
	  wheel_base			= 2.7;
	  max_speed_forward		= 3.0;
	  max_speed_backword	= 1.0;
	  max_steer_value		= 660;
	  min_steer_value		= -660;
	  max_brake_value		= 0;
	  min_brake_value		= 0;
	  max_steer_angle		= 0.42;
	  min_steer_angle		= 0.42;
	  length				= 4.3;
	  width					= 1.82;
  }

  double CalcMaxSteeringAngle()
  {
    return  max_steer_angle;//asin(wheel_base/turning_radius);
  }

  double BoundSpeed(double s)
  {
	if(s > 0 && s > max_speed_forward)
		return max_speed_forward;
	if(s < 0 && s < max_speed_backword)
		return max_speed_backword;
	return s;
  }

  double BoundSteerAngle(double a)
  {
	if(a > max_steer_angle)
		return max_steer_angle;
	if(a < min_steer_angle)
		return min_steer_angle;

	return a;
  }

  double BoundSteerValue(double v)
  {
	  if(v >= max_steer_value)
		return max_steer_value;
	if(v <= min_steer_value)
		return min_steer_value;

	return v;
  }

};


} /* namespace SimulationNS */

#endif /* COMMONSIMUDEFINITIONS_H_ */
