#ifndef __AUTOWARE_CONFIG_H__
#define __AUTOWARE_CONFIG_H__

// zmp parameters
#define ZMP_CLASS HevCnt

#define ZMP_CLEAR_CNT_DIAG() hev->ClearCntDiag()
#define ZMP_UPDATE_STATE() {					\
		hev->GetBrakeInf(&_brakeInf);			\
		hev->GetDrvInf(&_drvInf);				\
		hev->GetStrInf(&_strInf);				\
	}
#define ZMP_ACCEL_STROKE() _drvInf.actualPedalStr
#define ZMP_BRAKE_STROKE() _brakeInf.actualPedalStr
#define ZMP_STEERING_TORQUE() _strInf.torque
#define ZMP_STEERING_ANGLE() _strInf.angle
#define ZMP_VELOCITY() _drvInf.veloc

#define ZMP_SET_DRV_VELOC(x) hev->SetDrvVeloc((x))
#define ZMP_SET_DRV_STROKE(x) hev->SetDrvStroke((x))
#define USE_BRAKE_LAMP
#ifdef USE_BRAKE_LAMP
#define ZMP_SET_BRAKE_STROKE(x) do { \
	hev->SetBrakeStroke((x)); \
	if ((x) > 0) { \
		sndBrkLampOn(); \
	} else { \
		sndBrkLampOff(); \
	} \
    } while(0)
#else
#define ZMP_SET_BRAKE_STROKE(x) hev->SetBrakeStroke((x))
#endif /* USE_BRAKE_LAMP */
#define ZMP_SET_STR_TORQUE(x) hev->SetStrTorque((x))
#define ZMP_SET_STR_ANGLE(x) hev->SetStrAngle((x))

#define ZMP_SET_SHIFT_POS_D() hev->SetDrvShiftMode(SHIFT_POS_D)
#define ZMP_SET_SHIFT_POS_R() hev->SetDrvShiftMode(SHIFT_POS_R)
#define ZMP_SET_SHIFT_POS_B() hev->SetDrvShiftMode(SHIFT_POS_B)
#define ZMP_SET_SHIFT_POS_N() hev->SetDrvShiftMode(SHIFT_POS_N)

#define ZMP_STOP() {													\
	hev->SetDrvStroke(0);												\
	usleep(200000);														\
	hev->SetBrakeStroke(4095);											\
	usleep(200000);														\
	}

#define ZMP_DRV_CONTROLLED()											\
	((_drvInf.mode == 0x10 && _drvInf.servo == 0x10) ? 1 : 0)

#define ZMP_SET_DRV_MANUAL() {											\
		if (_drvInf.mode == 0x10) {										\
			hev->SetDrvMode(0x00);										\
			usleep(200000);												\
		}																\
		if (_drvInf.servo == 0x10)	{									\
			hev->SetDrvServo(0x00);										\
			usleep(200000);												\
		}																\
	}

#define ZMP_SET_DRV_PROGRAM() {											\
		if (_drvInf.mode == 0x00) {										\
			hev->SetDrvMode(0x10);										\
			usleep(200000);												\
			hev->SetDrvCMode(CONT_MODE_STROKE);							\
			usleep(200000);												\
		}																\
		if (_drvInf.servo == 0x00)	{									\
			hev->SetDrvServo(0x10);										\
			usleep(200000);												\
		}																\
	}

#define ZMP_STR_CONTROLLED()											\
	((_strInf.mode == 0x10 && _strInf.servo == 0x10) ? 1 : 0)

#define ZMP_SET_STR_MANUAL() {											\
		if (_strInf.mode == 0x10) {										\
			hev->SetStrMode(0x00);										\
			usleep(200000);												\
		}																\
		if (_strInf.servo == 0x10)	{									\
			hev->SetStrServo(0x00);										\
			usleep(200000);												\
		}																\
	}

#define ZMP_SET_STR_PROGRAM() {											\
		if (_strInf.mode == 0x00) {										\
			hev->SetStrMode(0x10);										\
			usleep(200000);												\
			hev->SetStrCMode(CONT_MODE_TORQUE);							\
			usleep(200000);												\
		}																\
		if (_strInf.servo == 0x00)	{									\
			hev->SetStrServo(0x10);										\
			usleep(200000);												\
		}																\
	}

// estima parameters
#define WHEEL_BASE 2.95 // tire-to-tire size of Estima.
#define WHEEL_ANGLE_MAX 31.16764 // max angle of front tires.
#define WHEEL_TO_STEERING (STEERING_ANGLE_MAX/WHEEL_ANGLE_MAX)
#define STEERING_ANGLE_MAX 600 // max angle of steering
#define STEERING_ANGLE_LIMIT 550 // could be STEERING_ANGLE_MAX but...
#define STEERING_INTERNAL_PERIOD 20 // ms (10ms is too fast for HEV)

// accel/brake parameters
#define _K_ACCEL_P_UNTIL20 120.0
#define _K_ACCEL_I_UNTIL20 0.3
#define _K_ACCEL_D_UNTIL20 2.0
#define _K_ACCEL_P_UNTIL10 90.0 // 40.0 (for 0-10km/h) // 30.0
#define _K_ACCEL_I_UNTIL10 0.2 // 0.1 (for 0-10km/h) // 5.0
#define _K_ACCEL_D_UNTIL10 2.0
#define _K_ACCEL_I_CYCLES 1000
#define _ACCEL_MAX_I 3000
#define _ACCEL_STROKE_DELTA_MAX 1000
#define _ACCEL_RELEASE_STEP 400
#define _ACCEL_PEDAL_MAX 2000 // 1700
#define _ACCEL_PEDAL_OFFSET 200

#define _K_BRAKE_P 170.0 // 190.0 // 100.0
#define _K_BRAKE_I 0.5 // 0.7
#define _K_BRAKE_D 10.0 // 10.0
#define _K_BRAKE_I_CYCLES 100
#define _BRAKE_MAX_I 4095 // 200
#define _BRAKE_STROKE_DELTA_MAX 1000
#define _BRAKE_RELEASE_STEP 500
#define _BRAKE_PEDAL_MAX 1900 // 2200 // 2016/09/12 1900 // 2016/09/05 1750 // 4095 // 1800-kitui
//#define _BRAKE_PEDAL_MED 2000
#define _BRAKE_PEDAL_STOPPING_MAX 2500 // 3000 // 2016/09/12 2500
#define _BRAKE_PEDAL_STOPPING_MED 2000 // 2500 // 2016/09/12 2000

#define _BRAKE_PEDAL_OFFSET 1000

// steering parameters
#define _STEERING_MAX_ANGVELSUM 1000
#define _STEERING_MAX_TORQUE 4000
#define _STEERING_MAX_SUM 100
#define _STEERING_ANGVEL_BOUNDARY 500.0 // deg/s
#define _STEERING_IGNORE_ERROR 1.0 // deg
#define _STEERING_DIFF_SMALL 10 // deg

// fast PID factors
#define _K_STEERING_P 60
#define _K_STEERING_I 5.0
#define _K_STEERING_D 5.0

// fast PID factors
#define _K_STEERING_P_STRICT 7.0 // 8.0 //60
#define _K_STEERING_I_STRICT 4.0 // 0.8 //5.0
#define _K_STEERING_D_STRICT 4.0 // 1.5 //5.0

// slow PID factors
#define _K_STEERING_P_SLOW 7.0 // 6.0 // 6.0
#define _K_STEERING_I_SLOW 4.0 // 7.0 // 5.0
#define _K_STEERING_D_SLOW 4.0 // 6.0 // 5.0

#define _STEERING_ANGLE_ERROR 0 // deg

//added by zmp
#define _TORQUE_ARRAY_NUM 15

#endif
