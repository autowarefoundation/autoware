#ifndef __AUTOWARE_CONFIG_H__
#define __AUTOWARE_CONFIG_H__

// prius parameters
#define WHEEL_BASE 2.7 // tire-to-tire size of Prius.
#define WHEEL_ANGLE_MAX 31.28067 // max angle of front tires.
#define WHEEL_TO_STEERING (STEERING_ANGLE_MAX/WHEEL_ANGLE_MAX)
#define STEERING_ANGLE_MAX 666 // max angle of steering
#define STEERING_ANGLE_LIMIT 550 // could be STEERING_ANGLE_MAX but...
#define STEERING_INTERNAL_PERIOD 20 // ms (10ms is too fast for HEV)

// accel/brake parameters
#define _K_ACCEL_P 30.0
#define _K_ACCEL_I 2.0
#define _K_ACCEL_D 2.0
#define _K_ACCEL_I_CYCLES 100
#define _ACCEL_MAX_I 600
#define _ACCEL_STROKE_DELTA_MAX 1000

#define _K_BRAKE_P 40.0
#define _K_BRAKE_I 10.0
#define _K_BRAKE_D 10.0
#define _K_BRAKE_I_CYCLES 100
#define _BRAKE_MAX_I 200
#define _BRAKE_STROKE_DELTA_MAX 1000

// steering parameters
#define _STEERING_MAX_ANGVELSUM 1000
#define _K_STEERING_TORQUE 10
#define _K_STEERING_TORQUE_I 0.5
#define _STEERING_MAX_TORQUE 2000
#define _STEERING_MAX_SUM 100 //deg*0.1s for I control

#define _K_STEERING_P 60
#define _K_STEERING_I 12
#define _K_STEERING_D 10

#define _STEERING_ANGLE_ERROR 0 // deg

#endif
