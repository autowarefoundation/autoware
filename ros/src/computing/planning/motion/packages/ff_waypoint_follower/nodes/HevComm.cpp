/*
 * HevComm.cpp
 *
 *  Created on: August 7, 2016
 *      Author: hatem
 */

#include "HevComm.h"

HevComm::HevComm()
{
	hev = new HevCnt();
	if(!hev->Init())
	{
		delete hev;
		hev = 0;
		return;
	}

	hev->Start();

	for(int i=0; i<21; i++)
		_config.data[i] = 0;

	hev->SetDrvMode(MODE_MANUAL);
	hev->SetStrMode(MODE_MANUAL);
	hev->SetConfigCallback(this);
}

HevComm::~HevComm()
{
	if(!hev) return;
	 hev->Stop();
	 hev->Close();
	 //Delete creates an Error !!!
	 if(hev)
		 	 {
		 delete hev;
		 hev = 0;
	 }
}

void HevComm::ConnectToCar()
{
	if(!hev) return;
	hev->SetDrvMode(MODE_PROGRAM);
	hev->SetDrvCMode(CONT_MODE_VELOCITY);
	hev->SetDrvOMode(OVERRIDE_MODE_ON);
	hev->SetDrvServo(0x00);

	hev->SetStrMode(MODE_PROGRAM);
	hev->SetStrCMode(CONT_MODE_ANGLE);
	hev->SetStrOMOde(OVERRIDE_MODE_ON);
	hev->SetStrServo(0x00);
}

bool HevComm::IsAutoDrive()
{
	if(!hev) return false;

	DrvInf driv_inf;
	hev->GetDrvInf(&driv_inf);

	if(driv_inf.mode == MODE_PROGRAM)
		return true;
	else
		return false;
}

bool HevComm::IsAutoSteer()
{
	if(!hev) return false;
	StrInf str_inf;
	hev->GetStrInf(&str_inf);

	if(str_inf.mode == MODE_PROGRAM)
		return true;
	else
		return false;
}

bool HevComm::IsAuto()
{
	if(!hev) return false;

	return IsAutoSteer() && IsAutoDrive();
}

void HevComm::DisconnectFromCar()
{
	if(!hev) return;
	 hev->SetDrvMode(MODE_MANUAL);
	 hev->SetDrvCMode(CONT_MODE_STROKE);
	 hev->SetDrvServo(0x10);

	 hev->SetStrMode(MODE_MANUAL);
	 hev->SetStrCMode(CONT_MODE_TORQUE);
	 hev->SetStrServo(0x10);
}

int HevComm::InitializeComm(const PlannerHNS::CAR_BASIC_INFO& carInfo)
{
	m_CarInfo = carInfo;
	return 1;
}

int HevComm::StartComm()
{
	return 1;
}

void HevComm::SetDriveMotorStatus(bool bOn)
{
	if (bOn) {
		ConnectToCar();
	} else {
		DisconnectFromCar();
	}
}

void HevComm::SetSteerMotorStatus(bool bOn)
{
}

void HevComm::SetNormalizedSteeringAngle(double angle)
{
	if(!hev) return;
	double sv = angle * m_CarInfo.max_steer_value / m_CarInfo.max_steer_angle;

	sv = m_CarInfo.BoundSteerValue(sv);

	hev->SetStrAngle(sv);
}

void HevComm::SetNormalizedSpeed(double speed)
{
	if(!hev) return;
	//1- convert from standard meter /second to the configured units

	//speed = speed * m_pCarInfo->distance_factor/m_pCarInfo->time_factor;
	speed = speed * 3.6;

	//speed = speed * 100;

	//speed = speed * 100;
	//speed = speed * 600/60;

	//2- check for max, min

	//if (speed > m_pCarInfo->max_speed_forward)
	//	speed = m_pCarInfo->max_speed_forward;
	//else if (speed < m_pCarInfo->max_speed_backword)
	//	speed = m_pCarInfo->max_speed_backword;

	hev->SetDrvVeloc(speed);

}

int HevComm::GetHevCorrectShift(PlannerHNS::SHIFT_POS s)
{
	int sp = 0x00;
	switch (s)
	{
	case PlannerHNS::SHIFT_POS_DD:
		sp = 0x10;
		break;
	case PlannerHNS::SHIFT_POS_NN:
		sp = 0x20;
		break;
	case PlannerHNS::SHIFT_POS_RR:
		sp = 0x40;
		break;
	default:
		sp = 0x00;
	}
	return sp;
}

PlannerHNS::SHIFT_POS HevComm::GetGeneralCorrectShift(int s)
{
	PlannerHNS::SHIFT_POS sp = PlannerHNS::SHIFT_POS_BB;
	switch (s)
	{
	case 0x10:
		sp = PlannerHNS::SHIFT_POS_DD;
		break;
	case 0x20:
		sp = PlannerHNS::SHIFT_POS_NN;
		break;
	case 0x40:
		sp = PlannerHNS::SHIFT_POS_RR;
		break;
	default:
		sp = PlannerHNS::SHIFT_POS_BB;
	}
	return sp;
}

void HevComm::SetShift(PlannerHNS::SHIFT_POS shift)
{
	if(!hev) return;
	int sp = GetHevCorrectShift(shift);
	hev->SetDrvShiftMode(sp);

}

void HevComm::SetBrakeValue(double brake)
{

}

bool HevComm::GetDriveMotorStatus()
{
	return true;
}

bool HevComm::GetSteerMotorStatus()
{
	return true;
}

double HevComm::GetCurrentSteerAngle()
{
	if(!hev) return 0;
	StrInf inf;
	hev->GetStrInf(&inf);

	return inf.angle * m_CarInfo.max_steer_angle/m_CarInfo.max_steer_value;

//	return a * m_pCarInfo->max_steer_angle
//			/ (m_pCarInfo->max_steer_value / 10.0);

}

double HevComm::GetCurrentSpeed()
{
	if(!hev) return 0;
	DrvInf inf;
	hev->GetDrvInf(&inf);

	return (double) inf.veloc/ (double)3.6;

	//return s * m_pCarInfo->time_factor/m_pCarInfo->distance_factor;
}

PlannerHNS::SHIFT_POS HevComm::GetCurrentShift()
{
	if(!hev) return PlannerHNS::SHIFT_POS_BB;
	DrvInf inf;
	hev->GetDrvInf(&inf);
	return GetGeneralCorrectShift(inf.actualShift);
}

double HevComm::GetCurrentBrakeValue()
{
	return 0;
}

int HevComm::GoLive(bool bLive)
{

	SetDriveMotorStatus(bLive);
	SetSteerMotorStatus(bLive);

	return 1;
}

