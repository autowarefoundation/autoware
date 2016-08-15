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
};

class ControllerParams
{
public:
	double Wheelbase;
	double MaxSteerAngle;
	double Wheel2SteerAngle;
	double SteeringDelay;
	double LocalizationDelay;
	double PursuiteDistance;
	PID_CONST Gain;
	PID_CONST GainLimit;
	double Acceleration;
	double Deceleration;
	double FollowDistance;

	double FOLLOW_PARAM_DIST;
	double FOLLOW_PARAM_VEL;
	double FOLLOW_PARAM_TTC;
	double LowpassSteerCutoff;
	double LowpassSteerQ;
	double LowpassAccelCutoff;
	double LowpassAccelQ;
	double steerPGain;
	double steerIGain;
	double steerDGain;
	double upperVelForSteerAdjust;
	double lowerVelForSteerAdjust;
	double gainForSteerAdjust;
	double watchAheadDist;
	double maxAccel;
	double minAccel;
	double accStopDist;
	double accRestartDist;
	double accTargetDistCoeff;
	double accAccelWhenStop;

	ControllerParams()
	{
		Wheelbase 			= 2.95;
		MaxSteerAngle 		= 0.52;
		Wheel2SteerAngle 	= 20.0;
		SteeringDelay 		= 0.8;
		LocalizationDelay	= 0.0;
		PursuiteDistance	= Wheelbase*1.0;
		Gain.kP				= 0.6;
		Gain.kD				= 0.001;
		Gain.kI				= 0.01;
		GainLimit			= Gain;
		Acceleration		= 0.05;
		Deceleration		= 1.6;
		FollowDistance		= 8.0;

		FOLLOW_PARAM_DIST 	= 0.044721;
		FOLLOW_PARAM_VEL	= 0.302395;
		FOLLOW_PARAM_TTC	= 5.0;
		LowpassSteerCutoff	= 5.0;
		LowpassSteerQ		= 0.5;
		LowpassAccelCutoff	= 2.0;
		LowpassAccelQ		= 0.5;
		steerPGain			= 0.6;
		steerIGain			= 0.01;
		steerDGain			= 0.01;
		upperVelForSteerAdjust = 5.0;
		lowerVelForSteerAdjust = 15.0;
		gainForSteerAdjust	= 0.8;
		watchAheadDist		= 50.0;
		maxAccel			= 0.7;
		minAccel			= -2.5;
		accStopDist			= 6.5;
		accRestartDist		= 7.5;
		accTargetDistCoeff	= 2.5;
		accAccelWhenStop	= -2.5;
	}
};

class CAR_BASIC_INFO
{
public:
  CAR_TYPE model;
  std::string texture_object;
  PID_CONST acceleration_pid;
  PID_CONST velocity_pid;
  PID_CONST steer_pid;
  PID_CONST brake_pid;

  double turning_radius;
  double wheel_base;
  double initialSpeed;
  double max_speed_forward;
  double max_speed_backword;
  double max_steer_value;
  double min_steer_value;
  double max_brake_value;
  double min_brake_value;
  double distance_factor;
  double time_factor;
  double max_steer_angle;
  double min_steer_angle;
  double length;
  double width;

  CAR_BASIC_INFO()
  {
	  model 				= SimulationCar;
	  turning_radius 		= 5.2;
	  wheel_base			= 2.7;
	  initialSpeed			= 0;
	  max_speed_forward		= 25;
	  max_speed_backword	= 4;
	  max_steer_value		= 700;
	  min_steer_value		= -700;
	  max_brake_value		= 0;
	  min_brake_value		= 0;
	  distance_factor		= 1;
	  time_factor			= 1;
	  max_steer_angle		= 0.32;
	  min_steer_angle		= 0.32;
	  length				= 4.3;
	  width					= 1.82;
  }

  double CalcMaxSteeringAngle()
  {
    return  asin(wheel_base/turning_radius);
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
