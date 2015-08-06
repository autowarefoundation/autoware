#ifndef __AUTOWARE_CONFIG_H__
#define __AUTOWARE_CONFIG_H__

// zmp parameters
#define ZMP_CLASS Mv2Cnt

#define ZMP_CLEAR_CNT_DIAG() mv2->ClearCntDiag()
#define ZMP_UPDATE_STATE() {					\
		mv2->GetBrakeInf(&_brakeInf);			\
		mv2->GetDrvInf(&_drvInf);				\
		mv2->GetStrInf(&_strInf);				\
	}
#define ZMP_ACCEL_STROKE() _drvInf.actualPedalStr
#define ZMP_BRAKE_STROKE() _brakeInf.actualPedalStr
#define ZMP_STEERING_TORQUE() _strInf.torque
#define ZMP_STEERING_ANGLE() _strInf.angle
#define ZMP_VELOCITY() _drvInf.veloc

#define ZMP_SET_DRV_VELOC(x) mv2->SetDrvVeloc((x))
#define ZMP_SET_DRV_STROKE(x) mv2->SetDrvStroke((x))
#define ZMP_SET_BRAKE_STROKE(x) mv2->SetBrakeStroke((x))
#define ZMP_SET_STR_TORQUE(x) mv2->SetStrTorque((x))
#define ZMP_SET_STR_ANGLE(x) mv2->SetStrAngle((x))

#define ZMP_SET_SHIFT_POS_D() mv2->SetDrvShiftMode(SHIFT_POS_D)
#define ZMP_SET_SHIFT_POS_R() mv2->SetDrvShiftMode(SHIFT_POS_R)
#define ZMP_SET_SHIFT_POS_B() mv2->SetDrvShiftMode(SHIFT_POS_B)
#define ZMP_SET_SHIFT_POS_N() mv2->SetDrvShiftMode(SHIFT_POS_N)

#define ZMP_STOP() {													\
	mv2->SetDrvStroke(0);												\
	usleep(200000);														\
	mv2->SetBrakeStroke(4095);											\
	usleep(200000);														\
	}

// Note: SERVO ON=0x00, OFF=0x10
#define ZMP_DRV_CONTROLLED()											\
	((_drvInf.mode == 0x00 && _drvInf.servo == 0x00) ? 1 : 0)

#define ZMP_SET_DRV_MANUAL() {											\
		if (_drvInf.mode == 0x00) {										\
			hev->SetDrvMode(0x10);										\
			usleep(200000);												\
		}																\
		if (_drvInf.servo == 0x00)	{									\
			hev->SetDrvServo(0x10);										\
			usleep(200000);												\
		}																\
	}

#define ZMP_SET_DRV_PROGRAM() {											\
		if (_drvInf.mode == 0x10) {										\
			hev->SetDrvMode(0x00);										\
			usleep(200000);												\
			hev->SetDrvCMode(CONT_MODE_STROKE);							\
			usleep(200000);												\
		}																\
		if (_drvInf.servo == 0x10)	{									\
			hev->SetDrvServo(0x00);										\
			usleep(200000);												\
		}																\
	}

#define ZMP_STR_CONTROLLED()											\
	((_strInf.mode == 0x00 && _strInf.servo == 0x00) ? 1 : 0)

#define ZMP_SET_STR_MANUAL() {											\
		if (_strInf.mode == 0x00) {										\
			hev->SetStrMode(0x10);										\
			usleep(200000);												\
		}																\
		if (_strInf.servo == 0x00)	{									\
			hev->SetStrServo(0x10);										\
			usleep(200000);												\
		}																\
	}

#define ZMP_SET_STR_PROGRAM() {											\
		if (_strInf.mode == 0x10) {										\
			hev->SetStrMode(0x00);										\
			usleep(200000);												\
			hev->SetStrCMode(CONT_MODE_TORQUE);							\
			usleep(200000);												\
		}																\
		if (_strInf.servo == 0x10)	{									\
			hev->SetStrServo(0x00);										\
			usleep(200000);												\
		}																\
	}

// prius parameters
#define WHEEL_BASE 1.53 // tire-to-tire size of COMS.
#define WHEEL_ANGLE_MAX 31.28067 // max angle of front tires.
#define WHEEL_TO_STEERING (STEERING_ANGLE_MAX/WHEEL_ANGLE_MAX)
#define STEERING_ANGLE_MAX 666 // max angle of steering
#define STEERING_ANGLE_LIMIT 550 // could be STEERING_ANGLE_MAX but...
#define STEERING_INTERNAL_PERIOD 20 // ms (10ms is too fast for MV2)

// accel/brake parameters
#define _K_ACCEL_P 230.0
#define _K_ACCEL_I 12.0
#define _K_ACCEL_D 50.0
#define _K_ACCEL_I_CYCLES 20
#define _ACCEL_MAX_I 1000
#define _ACCEL_STROKE_DELTA_MAX 1000
#define _ACCEL_RELEASE_STEP 400
#define _ACCEL_PEDAL_MAX 3000

#define _K_BRAKE_P 1000.0
#define _K_BRAKE_I 20.0
#define _K_BRAKE_D 50.0
#define _K_BRAKE_I_CYCLES 20
#define _BRAKE_MAX_I 200
#define _BRAKE_STROKE_DELTA_MAX 2000
#define _BRAKE_RELEASE_STEP 500
#define _BRAKE_PEDAL_MAX 4095
#define _BRAKE_PEDAL_MED 3000

// steering parameters
#define _STEERING_MAX_ANGVELSUM 1000
#define _K_STEERING_TORQUE 10
#define _K_STEERING_TORQUE_I 0.5
#define _STEERING_MAX_TORQUE 2000
#define _STEERING_MAX_SUM 100 //deg*0.1s for I control

#define _K_STEERING_P 60
#define _K_STEERING_I 13
#define _K_STEERING_D 10

#define _STEERING_ANGLE_ERROR 0 // deg

#endif
