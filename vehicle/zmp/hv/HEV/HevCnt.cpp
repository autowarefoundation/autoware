#include "HevCnt.h"
#include <time.h>
#define USE_DEMO 1

HevCnt::HevCnt()
{}

HevCnt::~HevCnt()
{}

bool HevCnt::Init()
{
    _canCom = new CanCommunication();
    _hevCnt = new HevControl();

    _canCom->InitCommunication(CAN_CHANNEL_0, CAN_BITRATE_500K);
    _canCom->InitCommunication(CAN_CHANNEL_1, CAN_BITRATE_1M);

    _hevCnt->InitHevControl(_canCom);
    _hevCnt->SetStatusCallback(this);
    _targetAngle=0;
    _beforAngle=0;
    _targetCnt=0;
    _callback = NULL;

//    _selectLog.battInf = false;
//    _selectLog.brkInf = false;
//    _selectLog.cameraInf = false;
//    _selectLog.drvInf = false;
//    _selectLog.otherInf = false;
//    _selectLog.start = false;
//    _selectLog.strInf = false;
    return true;
}

bool HevCnt::Start()
{
//    _canCom->StartCommunication();

//    _hevCnt->ClearDiag(ENGINE_ECU);
//    _hevCnt->ClearDiag(HV_ECU);
//    _hevCnt->ClearDiag(BRAKE_ECU);
//    _hevCnt->ClearDiag(BATTERY_ECU);
    _hevCnt->SetStrMode(0x10);
    _hevCnt->SetStrControlMode(0x10);
    _hevCnt->SetStrOverrideMode(0x10);
    _hevCnt->SetStrAngle(0.0f);
    _hevCnt->SetStrServo(0x10);
    _hevCnt->SetStrAngle(0.0f);

    _canCom->StartCommunication();
    return true;
}

bool HevCnt::SetConfigCallback(ChangeConfig* callback)
{
    _callback = callback;
    return true;
}

bool HevCnt::Process()
{
    return true;
}

bool HevCnt::Stop()
{

    return true;
}

bool HevCnt::Close()
{

    return true;
}

void HevCnt::GetBattInf(BattInf* batt)
{
    batt->soc = _battInf.soc;
    batt->voltage = _battInf.voltage;
    batt->current = _battInf.current;
    batt->max_temp = _battInf.max_temp;
    batt->min_temp = _battInf.min_temp;
    batt->max_chg_current = _battInf.max_chg_current;
    batt->max_dischg_current = _battInf.max_dischg_current;
}

void HevCnt::GetBrakeInf(BrakeInf* brake)
{
    brake->pressed = _brakeInf.pressed;
    brake->actualPedalStr = _brakeInf.actualPedalStr;
    brake->targetPedalStr = _brakeInf.targetPedalStr;
    brake->inputPedalStr = _brakeInf.inputPedalStr;
    brake->prl = _brakeInf.prl;
    brake->pfl = _brakeInf.pfl;
    brake->prr = _brakeInf.prr;
    brake->pfr = _brakeInf.pfr;
    brake->sks1 = _brakeInf.sks1;
    brake->sks2 = _brakeInf.sks2;
    brake->pmc1 = _brakeInf.pmc1;
    brake->pmc2 = _brakeInf.pmc2;
}

void HevCnt::GetOtherInf(OtherInf* other)
{
    other->door     = _otherInf.door;
    other->temp     = _otherInf.temp;
    other->rpm      = _otherInf.rpm;
    other->light    = _otherInf.light;
    other->level    = _otherInf.level;
    other->drv_mode = _otherInf.drv_mode;
    other->ev_mode  = _otherInf.ev_mode;
    other->cluise   = _otherInf.cluise;
    other->acc      = _otherInf.acc;
    other->sideAcc  = _otherInf.sideAcc;
    other->dtcData1 = _otherInf.dtcData1;
    other->dtcData2 = _otherInf.dtcData2;
    other->dtcData3 = _otherInf.dtcData3;
    other->dtcData4 = _otherInf.dtcData4;
    other->dtcData5 = _otherInf.dtcData5;
    other->dtcData6 = _otherInf.dtcData6;
    other->dtcData7 = _otherInf.dtcData7;
    other->dtcData8 = _otherInf.dtcData8;

    other->angleFromP = _otherInf.angleFromP;
    other->shiftFromPrius = _otherInf.shiftFromPrius;
    other->drvPedalStrFromP = _otherInf.drvPedalStrFromP;
    other->brkPedalStrFromP = _otherInf.brkPedalStrFromP;
    other->velocFlFromP = _otherInf.velocFlFromP;
    other->velocFrFromP = _otherInf.velocFrFromP;
    other->velocRlFromP = _otherInf.velocRlFromP;
    other->velocRrFromP = _otherInf.velocRrFromP;
    other->velocFromP = _otherInf.velocFromP;
    other->velocFromP2 = _otherInf.velocFromP2;
}

void HevCnt::GetDrvInf(DrvInf* drv)
{
    drv->mode = _drvInf.mode;
    drv->contMode = _drvInf.contMode;
    drv->overrideMode = _drvInf.overrideMode;
    drv->servo = _drvInf.servo;
    drv->actualPedalStr = _drvInf.actualPedalStr;
    drv->targetPedalStr = _drvInf.targetPedalStr;
    drv->inputPedalStr = _drvInf.inputPedalStr;
    drv->vpa1 = _drvInf.vpa1;
    drv->vpa2 = _drvInf.vpa2;
    drv->targetVeloc = _drvInf.targetVeloc;
    drv->veloc = _drvInf.veloc;
    drv->actualShift = _drvInf.actualShift;
    drv->targetShift = _drvInf.targetShift;
    drv->inputShift = _drvInf.inputShift;
    drv->shiftRawVsx1 = _drvInf.shiftRawVsx1;
    drv->shiftRawVsx2 = _drvInf.shiftRawVsx2;
    drv->shiftRawVsx3 = _drvInf.shiftRawVsx3;
    drv->shiftRawVsx4 = _drvInf.shiftRawVsx4;
}

void HevCnt::GetStrInf(StrInf* str)
{
    str->angle = _strInf.angle;
    str->cont_mode = _strInf.cont_mode;
    str->mode = _strInf.mode;
    str->overrideMode = _strInf.overrideMode;
    str->servo = _strInf.servo;
    str->targetAngle = _strInf.targetAngle;
    str->targetTorque = _strInf.targetTorque;
    str->torque = _strInf.torque;
    str->trq1 = _strInf.trq1;
    str->trq2 = _strInf.trq2;
}


void HevCnt::UpdateSteerState(REP_STEER_INFO_INDEX index)
{
    switch(index){
    case REP_STR_MODE:
        _hevCnt->GetStrMode((int&)_strInf.mode);
        _hevCnt->GetStrControlMode((int&)_strInf.cont_mode);
        _hevCnt->GetStrOverrideMode((int&)_strInf.overrideMode);
        _hevCnt->GetStrServo((int&)_strInf.servo);
        break;
    case REP_TORQUE:
        _hevCnt->GetStrTorque((int&)_strInf.torque);
        _hevCnt->GetStrTargetTorque((int&)_strInf.targetTorque);
        _hevCnt->SetStrTorque(_strInf.targetTorque + _asistTrq);
        break;
    case REP_ANGLE: _hevCnt->GetStrAngle((float&)_strInf.angle, (float&)_strInf.targetAngle); break;
    case REP_ANGLE_FROMOBD: _hevCnt->GetStrAngleFromOBD((float&)_otherInf.angleFromP); break;
	default: printf("\n"); break;
    }

    return;
}

void HevCnt::UpdateDriveState(REP_DRIVE_INFO_INDEX index)
{
    switch(index){
    case REP_DRV_MODE:
        _hevCnt->GetDrvMode((int&)_drvInf.mode);
        _hevCnt->GetDrvControlMode((int&)_drvInf.contMode);
        _hevCnt->GetDrvOverrideMode((int&)_drvInf.overrideMode);
        _hevCnt->GetDrvServo((int&)_drvInf.servo);
    case REP_GAS_PEDAL: _hevCnt->GetGasStroke((int&)_drvInf.actualPedalStr, (int&)_drvInf.targetPedalStr, (int&)_drvInf.inputPedalStr); break;
    case REP_GAS_PEDAL_FROMOBD: _hevCnt->GetGasStrokeFromOBD((int&)_otherInf.drvPedalStrFromP); break;
    case REP_VELOCITY: _hevCnt->GetVeloc((float&)_drvInf.veloc, (float&)_drvInf.targetVeloc); break;
    case REP_VELOCITY_FROMOBD: _hevCnt->GetVelocFromOBD((float&)_otherInf.velocFromP); break;
    case REP_VELOCITY_FROMOBD2: _hevCnt->GetVelocFromOBD2((float&)_otherInf.velocFromP2); break;
    case REP_WHEEL_VELOCITY_F: _hevCnt->GetWheelVelocF((float&)_otherInf.velocFrFromP, (float&)_otherInf.velocFlFromP); break;
    case REP_WHEEL_VELOCITY_R: _hevCnt->GetWheelVelocR((float&)_otherInf.velocRrFromP, (float&)_otherInf.velocRlFromP); break;
    case REP_BRAKE_PEDAL: _hevCnt->GetBrakeStroke((int&)_brakeInf.actualPedalStr, (int&)_brakeInf.targetPedalStr, (int&)_brakeInf.inputPedalStr); break;
    case REP_BRAKE_PEDAL_FROMOBD: _hevCnt->GetBrakeStrokeFromOBD((float&)_otherInf.brkPedalStrFromP,(bool&)_brakeInf.pressed); break;
    case REP_SHIFT_POS: _hevCnt->GetShiftMode((int&)_drvInf.actualShift, (int&)_drvInf.targetShift, (int&)_drvInf.inputShift); break;
    case REP_SHIFT_POS_FROMOBD: _hevCnt->GetShiftModeFromOBD((int&)_otherInf.shiftFromPrius); break;
    case REP_HEV_MODE: _hevCnt->GetEvMode((int&)_otherInf.ev_mode); break;
    case REP_ICE_RPM: _hevCnt->GetIceRpm((int&)_otherInf.rpm); break;
    case REP_ICE_COOLANT_TEMP: _hevCnt->GetIceCoolantTemp((int&)_otherInf.temp); break;
    case REP_ACCELERLATION: _hevCnt->GetAcc((float&)_otherInf.acc); break;
    case REP_SIDE_ACCELERLATION: _hevCnt->GetSideAcc((float&)_otherInf.sideAcc); break;
    case REP_DRIVE_MODE: _hevCnt->GetDriveMode((int&)_otherInf.drv_mode); break;
    case REP_CRUISE_STATE: _hevCnt->GetCruiseControl((bool&)_otherInf.cluise); break;
    case REP_DTC_STATUS: _hevCnt->GetDtcStatus((char&)_otherInf.dtcData1, (char&)_otherInf.dtcData2, (char&)_otherInf.dtcData3,
                         (char&)_otherInf.dtcData4, (char&)_otherInf.dtcData5, (char&)_otherInf.dtcData6, (char&)_otherInf.dtcData7, (char&)_otherInf.dtcData8);
	default: printf("\n"); break;
    }
    return;
}

void HevCnt::UpdateBattState(REP_BATT_INFO_INDEX index)
{
    switch(index){
	case REP_BATT_INFO: 
	    _hevCnt->GetBattInfo((float&)_battInf.soc,
				 (int&)_battInf.max_temp,
				 (int&)_battInf.min_temp,
				 (float&)_battInf.max_chg_current,
				 (float&)_battInf.max_dischg_current);
    case REP_BATT_INFO_CURRENT:
        _hevCnt->GetBattCurrent((float&)_battInf.current);
        break;
    case REP_BATT_INFO_VOLT:
        _hevCnt->GetBattVoltage((int&)_battInf.voltage);
	    break;
	default: printf("\n"); break;
    }

    return;
}

void HevCnt::UpdateOtherState(REP_OTHER_INFO_INDEX index)
{
    switch(index){
	case REP_LIGHT_STATE:
	    _hevCnt->GetLightState((LIGHT_STATE&)_otherInf.light);
	    break;
	case REP_GAS_LEVEL:
        _hevCnt->GetGasLevel((int&)_otherInf.level);
	    break;
	case REP_DOOR_STATE:
	    _hevCnt->GetDoorState((DOOR_STATE&)_otherInf.door);
	    break;
	default: printf("\n"); break;
    }

    return;
}

void HevCnt::getDate()
{
    time(&_day_time);
    _s_time = gmtime(&_day_time);
    gettimeofday(&_getTime, NULL);
}

void HevCnt::UpdateDemoSensorState(REP_DEMO_SENSOR_INFO_INDEX index)
{
    float v[12];
    switch (index) {
    case REP_OFZ0:
        _hevCnt->GetOfzValue0(v);
                // 中央2点の平均
                // -100から100にノーマライズ
        _sensInf.ofz[0] = (v[5] + v[6]) / 2 * 100;
 //               _sensInf.cntS[0]++;
//		printf("##########################################OFZ0 v5=%2.2f v6=%2.2f _ofz=%d\n",
//			v[5], v[6], _sensInf.ofz[0]);
       break;
    case REP_OFZ1:
        _hevCnt->GetOfzValue1(v);
        _sensInf.ofz[1] = -(v[5] + v[6]) / 2 * 100;
//                _sensInf.cntS[1]++;
//		printf("##########################################OFZ1 v5=%2.2f v6=%2.2f _ofz=%d\n",
//			v[5], v[6], _sensInf.ofz[1]);
        break;
    case REP_OFZ2:
        _hevCnt->GetOfzValue2(v);
        _sensInf.ofz[2] = (v[5] + v[6]) / 2 * 100;
//                _sensInf.cntS[2]++;
//		printf("##########################################OFZ2 v5=%2.2f v6=%2.2f _ofz=%d\n",
//			v[5], v[6], _sensInf.ofz[2]);
        break;
    case REP_OFZ3:
        _hevCnt->GetOfzValue3(v);
        _sensInf.ofz[3] = -(v[5] + v[6]) / 2 * 100;
//                _sensInf.cntS[3]++;
//		printf("##########################################OFZ3 v5=%2.2f v6=%2.2f _ofz=%d\n",
//			v[5], v[6], _sensInf.ofz[3]);
        break;
    case REP_SEAT_SENSOR:
        _hevCnt->GetSeatSensor((float&)_sensInf.seat);
                // 人が30cm以内にいたら1
//		printf("Sens s=%2.2f\n",_sensInf.seat);
        break;
	default: break;
	}	
}

void HevCnt::ReceiveConfig(int num, int index, int value[])
{
    printf("ReceiveConfig() num=%d index=%d value=%d\n", 
	   num, index, value[index]);
    int data[3];
    for(int i=0; i<num; i++){
        _config.data[index - 100] = value[i];
        data[i] = value[i];
    }
    if(NULL != _callback){
        _callback->UpdateConfig(num, index, data);
    }
}

void HevCnt::ReceiveErrorStatus(int level, int errCode)
{
    printf("ReceiveErrorStatus() level=%d errCode=%d\n", level, errCode);
    _errCode = errCode;
    _errLevel = level;
}

void HevCnt::ReceiveEcho(int kind, int no)
{
    printf("ReceiveEcho() kind=%d no=%d\n", kind, no);
}

void HevCnt::ReceiveImuMsg(REP_IMU_INFO_INDEX index)
{
    printf("ReceiveImuMsg() index=%d\n", index);
}

void HevCnt::rcvTime(char* date)
{
    getDate();
    sprintf(date, "%d/%d/%d/%d:%d:%d.%ld", _s_time->tm_year+1900, _s_time->tm_mon+1, _s_time->tm_mday,
            _s_time->tm_hour, _s_time->tm_min, _s_time->tm_sec, _getTime.tv_usec);
}

void HevCnt::SetSeat(int seat)
{
    _sensInf.seat = seat;
}

void HevCnt::SetOfz(int index, int val)
{
    _sensInf.ofz[index] = val;
}

void HevCnt::SetCnts(int index)
{
    _sensInf.cntS[index] += 1;
}

int HevCnt::GetSeat()
{
    return _sensInf.seat;
}

int HevCnt::GetOfz(int index)
{
    return _sensInf.ofz[index];
}

int HevCnt::GetCnts(int index)
{
    return _sensInf.cntS[index];
}

float HevCnt::GetDist()
{
    return _sensInf.distance;
}

void HevCnt::SetStrMode(int mode)
{
    _hevCnt->SetStrMode(mode);
}

void HevCnt::SetStrCMode(int cmode)
{
    _hevCnt->SetStrControlMode(cmode);
}

void HevCnt::SetStrOMOde(int omode)
{
    _hevCnt->SetStrOverrideMode(omode);
}

void HevCnt::SetStrTorque(int torque)
{
//    _hevCnt->SetStrTorque(torque);
    _asistTrq = torque;
}

void HevCnt::SetStrAngle(int angle)
{
    float sndVal = angle /10.0f;
    _hevCnt->SetStrAngle(sndVal);
}

void HevCnt::SetStrServo(int servo)
{
    _hevCnt->SetStrServo(servo);
}

void HevCnt::SetDrvMode(int mode)
{
    _hevCnt->SetDrvMode(mode);
}

void HevCnt::SetDrvCMode(int cmode)
{
    _hevCnt->SetDrvControlMode(cmode);
}

void HevCnt::SetDrvOMode(int omode)
{
    _hevCnt->SetDrvOverrideMode(omode);
}

void HevCnt::SetDrvStroke(int stroke)
{
    _hevCnt->SetGasStroke(stroke);
}

void HevCnt::SetDrvVeloc(int veloc)
{
    float sndVal = veloc /100.0f;
    _hevCnt->SetVeloc(sndVal);
}

void HevCnt::SetDrvShiftMode(int shift)
{
    _hevCnt->SetShiftMode(shift);
}

void HevCnt::SetDrvServo(int servo)
{
    _hevCnt->SetDrvServo(servo);
}

void HevCnt::SetBrakeStroke(int stroke)
{
    _hevCnt->SetBrakeStroke(stroke);
}

void HevCnt::SetControlGain(int index, int gain)
{
    _hevCnt->SetControlGain(index, gain);
}

void HevCnt::SndDiagReq(HEV_ECU kind)
{
    _hevCnt->GetDiag(kind);
}

void HevCnt::SndDiagClear(HEV_ECU kind)
{
    _hevCnt->ClearDiag(kind);
}

void HevCnt::SndErrReq()
{
    _hevCnt->ReadErrorStatusReq();
}

void HevCnt::GetErrCode(int* errLevel, int* errCode)
{
    *errCode = _errCode;
    *errLevel = _errLevel;
}

void HevCnt::GetConfig(HEV_CONFIG kind)
{
    _hevCnt->ReqControlGain((int)kind, 1);
}

void HevCnt::SetConfig(HEV_CONFIG kind, int val)
{
    _hevCnt->SetControlGain((int)kind, val);
}

void HevCnt::SetConfig(HEV_CONFIG kind, float val)
{
    _hevCnt->SetControlGain((int)kind, val);
}

void HevCnt::SaveConfig()
{
    _hevCnt->SaveControlGain();
}
