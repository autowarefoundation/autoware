#include "PhvCnt.h"
#include <time.h>
#define USE_DEMO 1

PhvCnt::PhvCnt()
{}

PhvCnt::~PhvCnt()
{}

// Autoware Extension
void PhvCnt::ClearCntDiag()
{
    CANMsg msg;
    msg.LEN = 1;
    msg.ID = MSG_GROUP_SYSCOM << 6 | MSG_SYSCOM_REQ_ERR_STATE;
    msg.DATA[0] = 1;
    _canCom->SendCanMessage(CAN_CHANNEL_1, &msg);
}

bool PhvCnt::Init()
{
    _canCom = new CanCommunication();
    _phvCnt = new PhvControl();
    _carTomo = new CarTomo();

    _canCom->InitCommunication(CAN_CHANNEL_0, CAN_BITRATE_500K);
    _canCom->InitCommunication(CAN_CHANNEL_1, CAN_BITRATE_1M);

    _phvCnt->InitPhvControl(_canCom);

    _carTomo->Init();
    _phvCnt->SetCarTomoCom(_carTomo);
    _phvCnt->SetStatusCallback(this);
    _targetAngle=0;
    _beforAngle=0;
    _targetCnt=0;
    _callback = NULL;

    ClearCntDiag(); // Autoware Extension

    return true;
}

bool PhvCnt::Start()
{
//    _phvCnt->ClearDiag(ENGINE_ECU);
//    _phvCnt->ClearDiag(HV_ECU);
//    _phvCnt->ClearDiag(BRAKE_ECU);
//    _phvCnt->ClearDiag(BATTERY_ECU);
    _phvCnt->SetDrvMode(MODE_MANUAL);
    _phvCnt->SetDrvOverrideMode(OVERRIDE_MODE_ENABLE);
    _phvCnt->SetDrvControlMode(CONT_MODE_STROKE);
    _phvCnt->SetDrvServo(SERVO_MODE_OFF);
    _phvCnt->SetDrvStroke(0.0f);

    _phvCnt->SetStrMode(MODE_MANUAL);
    _phvCnt->SetStrControlMode(CONT_MODE_ANGLE);
    _phvCnt->SetStrOverrideMode(OVERRIDE_MODE_ENABLE);
    _phvCnt->SetStrAngle(0.0f);
    _phvCnt->SetStrServo(SERVO_MODE_OFF);
    _phvCnt->SetStrAngle(0.0f);

    _canCom->StartCommunication();
    _carTomo->Start();
    return true;
}

bool PhvCnt::SetConfigCallback(ChangeConfig* callback)
{
    _callback = callback;
    return true;
}

bool PhvCnt::Process()
{
    return true;
}

bool PhvCnt::Stop()
{

    return true;
}

bool PhvCnt::Close()
{

    return true;
}

void PhvCnt::GetBattInf(BattInf* batt)
{
    batt->soc = _battInf.soc;
    batt->voltage = _battInf.voltage;
    batt->current = _battInf.current;
    batt->subVoltage = _battInf.subVoltage;
}

void PhvCnt::GetBrakeInf(BrakeInf* brake)
{
    brake->actualPedalStr = _brakeInf.actualPedalStr;
    brake->targetPedalStr = _brakeInf.targetPedalStr;
    brake->inputPedalStr = _brakeInf.inputPedalStr;
    brake->sks = _brakeInf.sks;
    brake->sla = _brakeInf.sla;
    brake->regPress = _brakeInf.regPress;
    brake->wheelPress = _brakeInf.wheelPress;
    brake->slr = _brakeInf.slr;
    brake->src = _brakeInf.src;
    brake->sksT = _brakeInf.sksT;
    brake->regT = _brakeInf.regT;
}

void PhvCnt::GetOtherInf(OtherInf* other)
{
    other->odometry = _otherInf.odometry;
    other->velocFrFromP = _otherInf.velocFrFromP;
    other->velocFlFromP = _otherInf.velocFlFromP;
    other->velocRrFromP = _otherInf.velocRrFromP;
    other->velocRlFromP = _otherInf.velocRlFromP;
    other->velocFromP = _otherInf.velocFromP;
    other->engineRpm      = _otherInf.engineRpm;
    other->motorRpm      = _otherInf.motorRpm;
    other->coolantTemp     = _otherInf.coolantTemp;
    other->shiftFromPrius = _otherInf.shiftFromPrius;
    other->drvPedalStrFromP = _otherInf.drvPedalStrFromP;
    other->brkPedalStrFromP = _otherInf.brkPedalStrFromP;
    other->brkState = _otherInf.brkState;
    other->angleFromP = _otherInf.angleFromP;
    other->drv_mode = _otherInf.drv_mode;
    other->ecoMode = _otherInf.ecoMode;
    other->ev_mode  = _otherInf.ev_mode;
    other->light    = _otherInf.light;
    for(int i=0; i<8;i++){
        other->diagData[i] = _otherInf.diagData[i];
    }
    other->doorOpenDr = _otherInf.doorOpenDr;
    other->doorOpenPass = _otherInf.doorOpenPass;
    other->doorOpenRr = _otherInf.doorOpenRr;
    other->doorOpenRl = _otherInf.doorOpenRl;
    other->doorOpenTrunk = _otherInf.doorOpenTrunk;
    other->doorLockDr = _otherInf.doorLockDr;
    other->doorLockPass = _otherInf.doorLockPass;
    other->doorLockRr = _otherInf.doorLockRr;
    other->doorLockRl = _otherInf.doorLockRl;
    other->doorLockTrunk = _otherInf.doorLockTrunk;
    other->windowDr = _otherInf.windowDr;
    other->windowPass = _otherInf.windowPass;
    other->windowRr = _otherInf.windowRr;
    other->windowRl = _otherInf.windowRl;
}

void PhvCnt::GetDrvInf(DrvInf* drv)
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
    drv->actualVeloc = _drvInf.actualVeloc;
    drv->actualShift = _drvInf.actualShift;
    drv->targetShift = _drvInf.targetShift;
    drv->inputShift = _drvInf.inputShift;
    drv->shiftRawVsx1 = _drvInf.shiftRawVsx1;
    drv->shiftRawVsx2 = _drvInf.shiftRawVsx2;
    drv->shiftRawVsx3 = _drvInf.shiftRawVsx3;
    drv->shiftRawVsx4 = _drvInf.shiftRawVsx4;
}

void PhvCnt::GetStrInf(StrInf* str)
{
     str->mode = _strInf.mode;
     str->cont_mode = _strInf.cont_mode;
     str->overrideMode = _strInf.overrideMode;
     str->servo = _strInf.servo;
     str->targetTorque = _strInf.targetTorque;
     str->actualTorque = _strInf.actualTorque;
     str->trq1 = _strInf.trq1;
     str->trq2 = _strInf.trq2;
     str->actualAngle = _strInf.actualAngle;
     str->targetAngle = _strInf.targetAngle;
}

void PhvCnt::GetPosInf(PosInf *pos)
{
    pos->gga = _posInf.gga;
    pos->hTemp = _posInf.hTemp;
    pos->humid = _posInf.humid;
    pos->press = _posInf.press;
    pos->pTemp = _posInf.pTemp;
}

void PhvCnt::GetIncInf(IncInf *inc)
{
    inc->angleX = _incInf.angleX;
    inc->angleY = _incInf.angleY;
}

void PhvCnt::GetImuInf(ImuInf* imuInf)
{
    imuInf->accX = _imuInf.accX;
    imuInf->accY = _imuInf.accY;
    imuInf->accZ = _imuInf.accZ;

    imuInf->gyroX = _imuInf.gyroX;
    imuInf->gyroY = _imuInf.gyroY;
    imuInf->gyroZ = _imuInf.gyroZ;

    imuInf->compX = _imuInf.compX;
    imuInf->compY = _imuInf.compY;
    imuInf->compZ = _imuInf.compZ;

    imuInf->battLevel = _imuInf.battLevel;
    imuInf->firmVersion = _imuInf.firmVersion;
    imuInf->format = _imuInf.format;
    imuInf->hardVersion = _imuInf.hardVersion;
    imuInf->period = _imuInf.period;
    imuInf->rangeAcc = _imuInf.rangeAcc;
    imuInf->rangeComp = _imuInf.rangeComp;
    imuInf->rangeGyro = _imuInf.rangeGyro;
    imuInf->role = _imuInf.role;
}


void PhvCnt::UpdateSteerState(REP_STEER_INFO_INDEX index)
{
    switch(index){
    case REP_STR_MODE:
        _phvCnt->GetStrMode((PHV_MODE&)_strInf.mode);
        _phvCnt->GetStrControlMode((STEER_CONTROL_MODE&)_strInf.cont_mode);
        _phvCnt->GetStrOverrideMode((OVERRIDE_MODE&)_strInf.overrideMode);
        _phvCnt->GetStrServo((SERVO_MODE&)_strInf.servo);
        break;
    case REP_STR_TORQUE:
        _phvCnt->GetStrActualTorque((float&)_strInf.actualTorque);
        _phvCnt->GetStrTargetTorque((float&)_strInf.targetTorque);
//        if((_strInf.actualTorque + _asistTrq) > 1.0f)
//            _phvCnt->SetStrTorque(1.0f);
//        else if((_strInf.actualTorque + _asistTrq) < -1.0f)
//            _phvCnt->SetStrTorque(-1.0f);
//        else
//            if(_strInf.mode == MODE_PROGRAM && _strInf.cont_mode == CONT_MODE_TORQUE)
//                _phvCnt->SetStrTorque(_strInf.actualTorque + _asistTrq);
        break;
    case REP_STR_ANGLE: _phvCnt->GetStrAngle((float&)_strInf.targetAngle, (float&)_strInf.actualAngle); break;
    case REP_STR_ANGLE_FROMV: _phvCnt->GetVehicleSteerAngle((float&)_otherInf.angleFromP); break;
    case REP_STR_TORQUE_SENSOR_RAW: _phvCnt->GetStrTorqueRaw((short&)_strInf.trq1, (short&)_strInf.trq2); break;
	default: printf("\n"); break;
    }
    return;
}

void PhvCnt::UpdateDriveState(REP_DRIVE_INFO_INDEX index)
{
    switch(index){
    case REP_DRV_MODE:
        _phvCnt->GetDrvMode((PHV_MODE&)_drvInf.mode);
        _phvCnt->GetDrvControlMode((DRIVE_CONTROL_MODE&)_drvInf.contMode);
        _phvCnt->GetDrvOverrideMode((OVERRIDE_MODE&)_drvInf.overrideMode);
        _phvCnt->GetDrvServo((SERVO_MODE&)_drvInf.servo);
        break;
    case REP_DRV_PEDAL_STROKE: _phvCnt->GetDrvStroke((float&)_drvInf.inputPedalStr, (float&)_drvInf.targetPedalStr, (float&)_drvInf.actualPedalStr); break;
    case REP_DRV_SHIFT_POS: _phvCnt->GetShift((SHIFT_POSITION&)_drvInf.inputShift, (SHIFT_POSITION&)_drvInf.targetShift, (SHIFT_POSITION&)_drvInf.actualShift); break;
    case REP_DRV_VELOCITY: _phvCnt->GetDrvVeloc((float&)_drvInf.targetVeloc, (float&)_drvInf.actualVeloc); break;
    case REP_DRV_PEDAL_SENSOR_RAW:
        _phvCnt->GetDrvStrokeRaw((short&)_drvInf.vpa1, (short&)_drvInf.vpa2);
        break;
    case REP_DRV_SHIFT_SENSOR_RAW:
        _phvCnt->GetDrvShiftRaw((short&)_drvInf.shiftRawVsx1, (short&)_drvInf.shiftRawVsx2, (short&)_drvInf.shiftRawVsx3, (short&)_drvInf.shiftRawVsx4);
        break;
    case REP_DRV_WHEEL_VELOCITY_FROMV:
        _phvCnt->GetVehicleWheelVeloc((float&)_otherInf.velocFrFromP, (float&)_otherInf.velocFlFromP, (float&)_otherInf.velocRrFromP, (float&)_otherInf.velocRlFromP);
        break;
    case REP_DRV_MOTOR_RPM_FROMV:
        _phvCnt->GetVehicleMotorRpm((float&)_otherInf.motorRpm);
        break;
    case REP_DRV_VELOCITY_FROMV:
        _phvCnt->GetVehicleVeloc((float&)_otherInf.velocFromP);
        break;
    case REP_DRV_ENGINE_RPM_FROMV:
        _phvCnt->GetVehicleEngineRpm((float&)_otherInf.engineRpm);
        break;
    case REP_DRV_PEDAL_STROKE2_FROMV:
        _phvCnt->GetVehicleDrvStroke2((float&)_otherInf.drvPedalStrFromP);
        break;
    case REP_DRV_EV_MODE_FROMV:
        _phvCnt->GetVehicleEvMode((unsigned char&)_otherInf.ev_mode);
        break;
    case REP_DRV_COOLANT_TEMP_FROMV:
        _phvCnt->GetVehicleCoolantTemp((float&)_otherInf.coolantTemp);
        break;
    case REP_DRV_SHIFT3_FROMV:
        _phvCnt->GetVehicleShiftLever3((unsigned char&)_otherInf.shiftFromPrius);
        break;
/*    case REP_DRV_ICE_FROMV:
        _phvCnt->GetVehicleIce((unsigned char&)_otherInf.ice);
        break;*/
/*    case REP_DRV_ENGINE_RPM2_FROMV:
        _phvCnt->GetVehicleEngineRpm2((float&)_otherInf.engineRpm);
        break;*/
    case REP_DRV_ECO_SWITCH_FROMV:
        _phvCnt->GetVehicleSwitchEco((bool&)_otherInf.ecoMode);
        break;
	default: printf("\n"); break;
    }
    return;
}

void PhvCnt::UpdateBrakeState(REP_BRAKE_INFO_INDEX index)
{
    switch(index){
    case REP_BRAKE_PEDAL:
        _phvCnt->GetBrakeInputStroke((float&)_brakeInf.inputPedalStr);
        _phvCnt->GetBrakeTargetStroke((float&)_brakeInf.targetPedalStr);
        _phvCnt->GetBrakeActualStroke((float&)_brakeInf.actualPedalStr);
        break;
    case REP_BRAKE_SENSOR1:
        _phvCnt->GetBrakeSensor1((short&)_brakeInf.sks, (short&)_brakeInf.sla, (short&)_brakeInf.regPress,(short&)_brakeInf.wheelPress);
        break;
    case REP_BRAKE_SENSOR2:
        _phvCnt->GetBrakeSensor2((short&)_brakeInf.slr,(short&)_brakeInf.src,(short&)_brakeInf.sksT,(short&)_brakeInf.regT);
        break;
/*    case REP_BRAKE_SWITCH_FROMV:
        _phvCnt->GetVehicleBrakePedalState((bool&)_otherInf.brkState);
        break;*/
    case REP_BRAKE_PEDAL_STROKE_FROMV:
        _phvCnt->GetVehicleBrakeStroke((float&)_otherInf.brkPedalStrFromP);
        break;
    case REP_BRAKE_PEDAL_STATE_FROMV:
        _phvCnt->GetVehicleBrakePedalState((bool&)_otherInf.brkState);
        break;
    default: break;
    }
    return;
}

void PhvCnt::UpdateBattState(REP_BATT_INFO_INDEX index)
{
    switch(index){
    case REP_BATT_VOLTAGE_FROMV:
        _phvCnt->GetVehicleVoltage((float&)_battInf.voltage);
        break;
    case REP_BATT_CURRENT2_FROMV:
        _phvCnt->GetVehicleCurrent((float&)_battInf.current);
        break;
    case REP_BATT_SUB_VOLTAGE_FROMV:
        _phvCnt->GetVehicleSubVoltage((float&)_battInf.subVoltage);
        break;
    case REP_BATT_SOC_FROMV:
        _phvCnt->GetVehicleSoc((float&)_battInf.soc);
        break;
	default: printf("\n"); break;
    }

    return;
}

void PhvCnt::UpdateOtherState(REP_OTHER_INFO_INDEX index)
{
    switch(index){
/*    case REP_OTHER_ACCELERLATION_FROMV:
        _phvCnt->GetVehicleAcceleration((short&)_otherInf.accX, (short&)_otherInf.accY, (short&)_otherInf.accZ);
        break;*/
/*    case REP_OTHER_INCLINO_FROMV:
        _phvCnt->GetVehicleInclino((unsigned char&)_otherInf.inclino);
        break;*/
    case REP_OTHER_ODOMETRY_FROMV:
        _phvCnt->GetVehicleOdometry((unsigned int&)_otherInf.odometry);
        break;
    case REP_OTHER_DOOR_OPEN_STATE_FROMV:
        _phvCnt->GetVehicleDoorOpenState((DOOR_OPEN_STATE&)_otherInf.doorOpenDr,
                                           (DOOR_OPEN_STATE&)_otherInf.doorOpenPass,
                                           (DOOR_OPEN_STATE&)_otherInf.doorOpenRr,
                                           (DOOR_OPEN_STATE&)_otherInf.doorOpenRl,
                                           (DOOR_OPEN_STATE&)_otherInf.doorOpenTrunk);
        break;
    case REP_OTHER_LIGHT_STATE_FROMV:
        _phvCnt->GetVehicleLightState((LIGHT_STATE&)_otherInf.light);
        break;
    case REP_OTHER_DOOR_LOCK_STATE_FROMV:
        _phvCnt->GetVehicleDoorLockState((DOOR_LOCK_STATE&)_otherInf.doorLockDr,
                                          (DOOR_LOCK_STATE&)_otherInf.doorLockPass,
                                          (DOOR_LOCK_STATE&)_otherInf.doorLockRr,
                                          (DOOR_LOCK_STATE&)_otherInf.doorLockRl,
                                          (DOOR_LOCK_STATE&)_otherInf.doorLockTrunk);
        break;
    case REP_OTHER_WINDOW_STATE_FROMV:
        _phvCnt->GetVehicleWindowState((DOOR_WINDOW_STATE&)_otherInf.windowDr,
                                        (DOOR_WINDOW_STATE&)_otherInf.windowPass,
                                        (DOOR_WINDOW_STATE&)_otherInf.windowRr,
                                        (DOOR_WINDOW_STATE&)_otherInf.windowRl);
        break;

    case RES_OTHER_DIAG_DATA:
        for(int i=0; i<8; i++){
            _phvCnt->GetVehicleDiagData(i, _otherInf.diagData[i]);
        }
        break;
	default: printf("\n"); break;
    }

    return;
}

void PhvCnt::ReceiveError(int error_code)
{
//    _otherInf.errCode = error_code;
}

void PhvCnt::getDate()
{
    time(&_day_time);
    _s_time = gmtime(&_day_time);
    gettimeofday(&_getTime, NULL);
}

void PhvCnt::ReceiveConfig(int num, int index, int value[])
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

void PhvCnt::ReceiveErrorStatus(int level, int errCode)
{
    printf("ReceiveErrorStatus() level=%d errCode=%d\n", level, errCode);
    _errCode = errCode;
    _errLevel = level;
}

void PhvCnt::ReceiveEcho(int kind, int no)
{
    printf("ReceiveEcho() kind=%d no=%d\n", kind, no);
}

void PhvCnt::ReceiveImuMsg(REP_IMUZ2_INFO_INDEX index)
{
    switch(index){
    case REP_IMUZ2_ACC_DATA:    /*!< 加速度データ */
        _phvCnt->GetImuAccX((double&)_imuInf.accX);
        _phvCnt->GetImuAccY((double&)_imuInf.accY);
        _phvCnt->GetImuAccZ((double&)_imuInf.accZ);
        break;
    case REP_IMUZ2_GYRO_DATA:/*!< 角速度データ */
        _phvCnt->GetImuGyroX((double&)_imuInf.gyroX);
        _phvCnt->GetImuGyroY((double&)_imuInf.gyroY);
        _phvCnt->GetImuGyroZ((double&)_imuInf.gyroZ);
        break;
    case REP_IMUZ2_COMP_DATA: /*!< 地磁気データ */
        _phvCnt->GetImuCompX((double&)_imuInf.compX);
        _phvCnt->GetImuCompY((double&)_imuInf.compY);
        _phvCnt->GetImuCompZ((double&)_imuInf.compZ);
        break;
    case RES_IMUZ2_STATE:       /*!< IMU-Z2状態 */
        break;
    case RES_IMUZ2_PROFILE:/*!< デバイスプロファイル */
        break;
    default: break;
    }
}

void PhvCnt::UpdatePoszState(REP_POSZ_INFO_INDEX index)
{
    switch(index){
    case REP_POSZ_GPGGA:
        _phvCnt->GetPoszGPGGA((GPGGA_DATA&)_posInf); break;
    case REP_POSZ_GPZDA: break;
    case REP_POSZ_GPGLL: break;
    case REP_POSZ_GPGSA: break;
    case REP_POSZ_GPGSV: break;
    case REP_POSZ_GPVTG: break;
    case REP_POSZ_GPRMC: break;
    case REP_POSZ_PRESSURE:
        _phvCnt->GetPoszPressData((float&)_posInf.press, (float&)_posInf.pTemp); break;
    case REP_POSZ_HUMIDTY:
        _phvCnt->GetPoszHumidData((float&)_posInf.humid, (float&)_posInf.hTemp); break;
    default: break;
    }
}

void PhvCnt::UpdateInczState(REP_INCZ_INFO_INDEX index)
{
    switch(index){
    case REP_INCZ_STATE:
    case REP_INCZ_ACC:
        break;
    case REP_INCZ_ANGLE:
        _phvCnt->GetInczInf((double&)_incInf.angleX, (double&)_incInf.angleY);
        break;
        default: break;
    }
}



void PhvCnt::rcvTime(char* date)
{
    getDate();
    sprintf(date, "%d/%d/%d/%d:%d:%d.%ld", _s_time->tm_year+1900, _s_time->tm_mon+1, _s_time->tm_mday,
            _s_time->tm_hour, _s_time->tm_min, _s_time->tm_sec, _getTime.tv_usec);
}

void PhvCnt::SetStrMode(PHV_MODE mode)
{
    if(mode != MODE_MANUAL && mode != MODE_PROGRAM)
        return;

    _phvCnt->SetStrMode(mode);
}

void PhvCnt::SetStrCMode(STEER_CONTROL_MODE cmode)
{
    if(cmode != CONT_MODE_ANGLE && cmode != CONT_MODE_TORQUE)
        return;

    _phvCnt->SetStrControlMode(cmode);
}

void PhvCnt::SetStrOMOde(OVERRIDE_MODE omode)
{
    if(omode != OVERRIDE_MODE_ENABLE && omode != OVERRIDE_MODE_DISABLE)
        return;

    _phvCnt->SetStrOverrideMode(omode);
}

void PhvCnt::SetStrTorque(float torque)
{
    if(torque > 1.0f)
        torque = 1.0f;
    else if(torque < -1.0)
        torque = -1.0f;
    _phvCnt->SetStrTorque(torque);
//    _asistTrq = torque;
}

void PhvCnt::SetStrAngle(short angle)
{
    if(angle >6600)
        angle = 6600;
    else if(angle < -6600)
        angle = -6600;

    float sndVal = angle /10.0f;
    _phvCnt->SetStrAngle(sndVal);
}

void PhvCnt::SetStrServo(SERVO_MODE servo)
{
    if(servo != SERVO_MODE_OFF && servo != SERVO_MODE_ON)
        return;

    _phvCnt->SetStrServo(servo);
}

void PhvCnt::SetDrvMode(PHV_MODE mode)
{
    if(mode != MODE_MANUAL && mode != MODE_PROGRAM)
        return;

    _phvCnt->SetDrvMode(mode);
    if(mode == MODE_PROGRAM){
        _phvCnt->SetDrvControlMode(CONT_MODE_STROKE);
        _phvCnt->SetDrvStroke(0.0f);
        _phvCnt->SetBrakeStroke(0.0f);
    }

}

void PhvCnt::SetDrvCMode(DRIVE_CONTROL_MODE cmode)
{
    if(cmode != CONT_MODE_VELOCITY && cmode != CONT_MODE_STROKE)
        return;

    _phvCnt->SetDrvControlMode(cmode);
}

void PhvCnt::SetDrvOMode(OVERRIDE_MODE omode)
{
    if(omode != OVERRIDE_MODE_ENABLE && omode != OVERRIDE_MODE_DISABLE)
        return;

    _phvCnt->SetDrvOverrideMode(omode);
}

void PhvCnt::SetDrvStroke(float stroke)
{
    if(stroke > 1.0f)
        stroke = 1.0f;
    else if(stroke < -1.0f)
        stroke = -1.0f;

    _phvCnt->SetDrvStroke(stroke);
}

void PhvCnt::SetDrvVeloc(float veloc)
{
    if(veloc > 60.0f)
        veloc = 60.0f;
    else if(veloc < 0.0f)
        veloc = 0.0f;
    _phvCnt->SetVeloc(veloc);
}

void PhvCnt::SetDrvShiftMode(SHIFT_POSITION shift)
{
    if(shift != SHIFT_POS_P && shift != SHIFT_POS_R &&
       shift != SHIFT_POS_N && shift != SHIFT_POS_D &&
       shift != SHIFT_POS_B && shift != SHIFT_POS_S &&
       shift != SHIFT_POS_U)
        return;

    _phvCnt->SetShiftMode(shift);
}

void PhvCnt::SetDrvServo(SERVO_MODE servo)
{
    if(servo != SERVO_MODE_OFF && servo != SERVO_MODE_ON)
        return;

    _phvCnt->SetDrvServo(servo);
}

void PhvCnt::SetBrakeStroke(float stroke)
{
    if(stroke > 1.0f)
        stroke = 1.0f;
    else if(stroke < -1.0f)
        stroke = -1.0f;

    _phvCnt->SetBrakeStroke(stroke);
}

void PhvCnt::SndBrakeLamp(unsigned char lamp)
{
    _phvCnt->SetBrakeLamp(lamp);
}

void PhvCnt::SndLeftBlinker(unsigned char left)
{
    _phvCnt->SetBlinkerLeft(left);
}

void PhvCnt::SndRightBlinker(unsigned char right)
{
    _phvCnt->SetBlinkerRight(right);
}

void PhvCnt::SndBrakeAutoLamp(unsigned char autoMode)
{
    _phvCnt->SetBrakeAutoLamp(autoMode);
}

void PhvCnt::SetControlGain(int index, int gain)
{
    _phvCnt->SetControlGain(index, gain);
}

void PhvCnt::SndDiagReq(PHV_ECU kind)
{
    _phvCnt->GetDiag(kind);
}

/*void PhvCnt::SndDiagClear(PHV_ECU kind)
{
    CANMsg msg;
    msg.ID = 0x72F;
    msg.LEN = 1;
    msg.DATA[0] = 0x7E;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x72F;
    msg.LEN = 1;
    msg.DATA[0] = 0x54;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7A1;
    msg.LEN = 1;
    msg.DATA[0] = 0x3E;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7A1;
    msg.LEN = 1;
    msg.DATA[0] = 0x14;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7A9;
    msg.LEN = 1;
    msg.DATA[0] = 0x7E;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7A9;
    msg.LEN = 3;
    msg.DATA[0] = 0x7F;
    msg.DATA[1] = 0x14;
    msg.DATA[2] = 0x78;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7A9;
    msg.LEN = 1;
    msg.DATA[0] = 0x54;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B0;
    msg.LEN = 1;
    msg.DATA[0] = 0x3E;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B0;
    msg.LEN = 2;
    msg.DATA[0] = 0x13;
    msg.DATA[1] = 0x82;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B0;
    msg.LEN = 2;
    msg.DATA[0] = 0x13;
    msg.DATA[1] = 0x82;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B0;
    msg.LEN = 2;
    msg.DATA[0] = 0x13;
    msg.DATA[1] = 0x82;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B0;
    msg.LEN = 1;
    msg.DATA[0] = 0x14;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B8;
    msg.LEN = 1;
    msg.DATA[0] = 0x7E;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B8;
    msg.LEN = 3;
    msg.DATA[0] = 0x7F;
    msg.DATA[1] = 0x13;
    msg.DATA[2] = 0x12;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B8;
    msg.LEN = 3;
    msg.DATA[0] = 0x7F;
    msg.DATA[1] = 0x13;
    msg.DATA[2] = 0x12;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B8;
    msg.LEN = 3;
    msg.DATA[0] = 0x7F;
    msg.DATA[1] = 0x13;
    msg.DATA[2] = 0x12;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B8;
    msg.LEN = 3;
    msg.DATA[0] = 0x7F;
    msg.DATA[1] = 0x13;
    msg.DATA[2] = 0x12;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B8;
    msg.LEN = 3;
    msg.DATA[0] = 0x7F;
    msg.DATA[1] = 0x13;
    msg.DATA[2] = 0x12;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B8;
    msg.LEN = 3;
    msg.DATA[0] = 0x7F;
    msg.DATA[1] = 0x13;
    msg.DATA[2] = 0x12;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B8;
    msg.LEN = 3;
    msg.DATA[0] = 0x7F;
    msg.DATA[1] = 0x13;
    msg.DATA[2] = 0x12;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7B8;
    msg.LEN = 1;
    msg.DATA[0] = 0x54;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7C0;
    msg.LEN = 1;
    msg.DATA[0] = 0x3E;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7C0;
    msg.LEN = 1;
    msg.DATA[0] = 0x14;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7C4;
    msg.LEN = 1;
    msg.DATA[0] = 0x3E;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7C4;
    msg.LEN = 1;
    msg.DATA[0] = 0x14;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7C8;
    msg.LEN = 1;
    msg.DATA[0] = 0x7E;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7C8;
    msg.LEN = 3;
    msg.DATA[0] = 0x7F;
    msg.DATA[1] = 0x14;
    msg.DATA[2] = 0x78;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7CC;
    msg.LEN = 1;
    msg.DATA[0] = 0x7E;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7CC;
    msg.LEN = 1;
    msg.DATA[0] = 0x54;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7EA;
    msg.LEN = 1;
    msg.DATA[0] = 0x7E;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7EA;
    msg.LEN = 1;
    msg.DATA[0] = 0x44;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7EA;
    msg.LEN = 1;
    msg.DATA[0] = 0x7E;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7EA;
    msg.LEN = 1;
    msg.DATA[0] = 0x44;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7EF;
    msg.LEN = 1;
    msg.DATA[0] = 0x7E;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);

    msg.ID = 0x7EF;
    msg.LEN = 1;
    msg.DATA[0] = 0x44;
    _canCom->SendCanMessage(CAN_CHANNEL_PHV, &msg);
}*/

void PhvCnt::GetErrCode(int* errLevel, int* errCode)
{
    *errCode = _errCode;
    *errLevel = _errLevel;
}

void PhvCnt::GetConfig(PHV_CONFIG kind)
{
    _phvCnt->ReqControlGain((int)kind, 1);
}

void PhvCnt::SetConfig(PHV_CONFIG kind, int val)
{
    _phvCnt->SetControlGain((int)kind, val);
}

void PhvCnt::SaveConfig()
{
    _phvCnt->WriteControlGain();
}
