#ifndef __AUTOWARE_CONFIG_H__
#define __AUTOWARE_CONFIG_H__

// zmp parameters
#define ZMP_CLASS PhvCnt

#define ZMP_CLEAR_CNT_DIAG() phv->ClearCntDiag()
#define ZMP_UPDATE_STATE() {					\
		phv->GetBrakeInf(&_brakeInf);			\
		phv->GetDrvInf(&_drvInf);				\
		phv->GetStrInf(&_strInf);				\
	}
#define ZMP_ACCEL_STROKE() _drvInf.actualPedalStr
#define ZMP_BRAKE_STROKE() _brakeInf.actualPedalStr
#define ZMP_STEERING_TORQUE() _strInf.actualTorque
#define ZMP_STEERING_ANGLE() _strInf.actualAngle
#define ZMP_VELOCITY() _drvInf.actualVeloc

#define ZMP_SET_DRV_VELOC(x) phv->SetDrvVeloc((x))
#define ZMP_SET_DRV_STROKE(x) phv->SetDrvStroke((x))
#define ZMP_SET_BRAKE_STROKE(x) phv->SetBrakeStroke((x))
#define ZMP_SET_STR_TORQUE(x) phv->SetStrTorque((x))
#define ZMP_SET_STR_ANGLE(x) phv->SetStrAngle((x))

#define ZMP_SET_SHIFT_POS_D() phv->SetDrvShiftMode(SHIFT_POS_D)
#define ZMP_SET_SHIFT_POS_R() phv->SetDrvShiftMode(SHIFT_POS_R)
#define ZMP_SET_SHIFT_POS_B() phv->SetDrvShiftMode(SHIFT_POS_B)
#define ZMP_SET_SHIFT_POS_N() phv->SetDrvShiftMode(SHIFT_POS_N)

#define ZMP_STOP() {													\
	phv->SetDrvStroke(0);												\
	usleep(200000);														\
	phv->SetBrakeStroke(4095);											\
	usleep(200000);														\
	}

#define ZMP_DRV_CONTROLLED()											\
	((_drvInf.mode == MODE_PROGRAM && _drvInf.servo == SERVO_MODE_ON) ? 1 : 0)

#define ZMP_SET_DRV_MANUAL() {											\
		if (_drvInf.mode == MODE_PROGRAM) {								\
			phv->SetDrvMode(MODE_MANUAL);								\
			usleep(200000);												\
		}																\
		if (_drvInf.servo == SERVO_MODE_ON)	{							\
			phv->SetDrvServo(SERVO_MODE_OFF);							\
			usleep(200000);												\
		}																\
	}

#define ZMP_SET_DRV_PROGRAM() {											\
		if (_drvInf.mode == MODE_MANUAL) {								\
			phv->SetDrvMode(MODE_PROGRAM);								\
			usleep(200000);												\
			phv->SetDrvCMode(CONT_MODE_STROKE);							\
			usleep(200000);												\
		}																\
		if (_drvInf.servo == SERVO_MODE_OFF)	{						\
			phv->SetDrvServo(SERVO_MODE_ON);							\
			usleep(200000);												\
		}																\
	}

#define ZMP_STR_CONTROLLED()											\
	((_strInf.mode == MODE_PROGRAM && _strInf.servo == SERVO_MODE_ON) ? 1 : 0)

#define ZMP_SET_STR_MANUAL() {											\
		if (_strInf.mode == MODE_PROGRAM) {								\
			phv->SetStrMode(MODE_MANUAL);								\
			usleep(200000);												\
		}																\
		if (_strInf.servo == SERVO_MODE_ON)	{							\
			phv->SetStrServo(SERVO_MODE_OFF);							\
			usleep(200000);												\
		}																\
	}

#define ZMP_SET_STR_PROGRAM() {											\
		if (_strInf.mode == MODE_MANUAL) {								\
			phv->SetStrMode(MODE_PROGRAM);								\
			usleep(200000);												\
			phv->SetStrCMode(CONT_MODE_TORQUE);							\
			usleep(200000);												\
		}																\
		if (_strInf.servo == SERVO_MODE_OFF)	{						\
			phv->SetStrServo(SERVO_MODE_ON);							\
			usleep(200000);												\
		}																\
	}

// prius parameters
#define WHEEL_BASE 2.7 // tire-to-tire size of Prius.
#define WHEEL_ANGLE_MAX 31.28067 // max angle of front tires.
#define WHEEL_TO_STEERING (STEERING_ANGLE_MAX/WHEEL_ANGLE_MAX)
#define STEERING_ANGLE_MAX 666 // max angle of steering
#define STEERING_ANGLE_LIMIT 550 // could be STEERING_ANGLE_MAX but...
#define STEERING_INTERNAL_PERIOD 20 // ms (10ms is too fast for HEV)

// accel/brake parameters
#define _K_ACCEL_P 30.0 //3.0
#define _K_ACCEL_I 2.0
#define _K_ACCEL_D 2.0
#define _K_ACCEL_I_CYCLES 100
#define _ACCEL_MAX_I 600
#define _ACCEL_STROKE_DELTA_MAX 1000
#define _ACCEL_RELEASE_STEP 400
#define _ACCEL_PEDAL_MAX 1700

#define _K_BRAKE_P 40.0
#define _K_BRAKE_I 10.0 //5.0
#define _K_BRAKE_D 10.0
#define _K_BRAKE_I_CYCLES 100
#define _BRAKE_MAX_I 200
#define _BRAKE_STROKE_DELTA_MAX 1000
#define _BRAKE_RELEASE_STEP 500
#define _BRAKE_PEDAL_MAX 4095
#define _BRAKE_PEDAL_MED 3000

// steering parameters
#define _STEERING_MAX_ANGVELSUM 1000
#define _K_STEERING_TORQUE 10
#define _K_STEERING_TORQUE_I 0.5
#define _STEERING_MAX_TORQUE 2000
#define _STEERING_MAX_SUM 100 //deg*0.1s for I control

#define _K_STEERING_P 8
#define _K_STEERING_I 4
#define _K_STEERING_D 2

#define _STEERING_ANGLE_ERROR 0 // deg

#endif
