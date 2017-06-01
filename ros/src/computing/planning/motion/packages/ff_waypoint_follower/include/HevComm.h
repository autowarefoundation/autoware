/*
 * HevComm.h
 *
 *  Created on: August 7, 2016
 *      Author: hatem
 */

#ifndef HEVCOMM_H_
#define HEVCOMM_H_

#include "PlannerCommonDef.h"
#include "RoadNetwork.h"
#include "HevCnt.h"

class HevComm : public ChangeConfig
{
public:
	HevComm();
	virtual ~HevComm();

	virtual int InitializeComm(const PlannerHNS::CAR_BASIC_INFO& carInfo);
	virtual int StartComm();
	virtual void SetDriveMotorStatus(bool bOn);
	virtual void SetSteerMotorStatus(bool bOn);
	virtual void SetNormalizedSteeringAngle(double angle); // radians
	virtual void SetNormalizedSpeed(double speed); // meter/second;
	virtual void SetShift(PlannerHNS::SHIFT_POS shift); //(0x00=B, 0x10=D, 0x20=N, 0x40=R)
	virtual void SetBrakeValue(double brake); // normalized from 0~1

	//get and read
	virtual bool GetDriveMotorStatus();
	virtual bool GetSteerMotorStatus();
	virtual double GetCurrentSteerAngle(); // by radians
	virtual double GetCurrentSpeed(); // by meter/second
	virtual PlannerHNS::SHIFT_POS GetCurrentShift();
	virtual double GetCurrentBrakeValue(); // normalized from 0~1
	int GetHevCorrectShift(PlannerHNS::SHIFT_POS s);
	PlannerHNS::SHIFT_POS GetGeneralCorrectShift(int shift);

	virtual int GoLive(bool bLive);
	virtual bool IsAutoDrive();
	virtual bool IsAutoSteer();
	virtual bool IsAuto();

	//Dumy implementations
	bool CloseComm(){return true;}

	PlannerHNS::CAR_BASIC_INFO m_CarInfo;


private:
	HevCnt* hev;
	ConfigInf _config;

	void UpdateConfig(int num, int index, int data[])
	{
		for(int i=0; i<num; i++){
			_config.data[index-100] = data[i];
		}
	};

	void ConnectToCar();
	void DisconnectFromCar();
};

#endif /* HEVCOMM_H_ */
