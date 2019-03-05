
/// \file PlannerCommonDef.h
/// \brief Definition file for control related data types
/// \author Hatem Darweesh
/// \date Dec 14, 2016


#ifndef PLANNERCOMMONDEF_H_
#define PLANNERCOMMONDEF_H_

#include <math.h>
#include <string>

namespace PlannerHNS
{

enum MAP_SOURCE_TYPE
{
	MAP_AUTOWARE,
	MAP_FOLDER,
	MAP_KML_FILE,
	MAP_ONE_CSV_FILE,
	MAP_LANES_CSV_FILES
};

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
		Acceleration		= 0.5;
		Deceleration		= -0.8;
		FollowDistance		= 8.0;
		LowpassSteerCutoff	= 5.0;
		maxAccel			= 0.9;
		minPursuiteDistance = 2.0;
		maxDecel 			= -1.5;
	}
};

class CAR_BASIC_INFO
{
public:
  CAR_TYPE model;

  double turning_radius;
  double wheel_base;
  double max_speed_forward;
  double min_speed_forward;
  double max_speed_backword;
  double max_steer_value;
  double min_steer_value;
  double max_brake_value;
  double min_brake_value;
  double max_steer_angle;
  double min_steer_angle;
  double length;
  double width;
  double max_acceleration;
  double max_deceleration;

  CAR_BASIC_INFO()
  {
	  model 				= SimulationCar;
	  turning_radius 		= 5.2;
	  wheel_base			= 2.7;
	  max_speed_forward		= 3.0;
	  min_speed_forward		= 0.0;
	  max_speed_backword	= 1.0;
	  max_steer_value		= 660;
	  min_steer_value		= -660;
	  max_brake_value		= 0;
	  min_brake_value		= 0;
	  max_steer_angle		= 0.42;
	  min_steer_angle		= 0.42;
	  length				= 4.3;
	  width					= 1.82;
	  max_acceleration		= 1.5; // m/s2
	  max_deceleration		= -1.5; // 1/3 G
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


} /* namespace PlannerHNS */

#endif /* PLANNERCOMMONDEF_H_ */
