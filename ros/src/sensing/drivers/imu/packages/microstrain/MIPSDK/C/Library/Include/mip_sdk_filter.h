/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_filter.h
//! @author  Nathan Miller
//! @version 1.1
//
//! @description FILTER MIP Descriptor Set Definitions.
//
// External dependencies:
//
//
//
//!@copyright 2014 Lord Microstrain Sensing Systems. 
//
//!@section CHANGES
//! 
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING 
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER 
//! FOR THEM TO SAVE TIME. AS A RESULT, LORD MICROSTRAIN SENSING SYSTEMS
//! SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES 
//! WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR 
//! THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION 
//! WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////


#ifndef _MIP_SDK_FILTER_H
#define _MIP_SDK_FILTER_H


////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include "mip.h"
#include "mip_sdk_interface.h"

////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
////////////////////////////////////////////////////////////////////////////////
//! @def


////////////////////////////////////////////////////////////////////////////////
//
// Descriptor Set designators - used in the Desc Set field of the MIP header
//
////////////////////////////////////////////////////////////////////////////////

#define MIP_FILTER_COMMAND_SET                                0x0D
#define MIP_FILTER_DATA_SET                                   0x82


////////////////////////////////////////////////////////////////////////////////
// FILTER COMMAND DESCRIPTORS (command desc are < 0x80)
////////////////////////////////////////////////////////////////////////////////

#define MIP_FILTER_CMD_RESET_FILTER                           0x01
#define MIP_FILTER_CMD_SET_INITIAL_ATTITUDE                   0x02
#define MIP_FILTER_CMD_SET_INITIAL_HEADING                    0x03
#define MIP_FILTER_CMD_SET_INITIAL_HEADING_FROM_AHRS          0x04
#define MIP_FILTER_CMD_VEHICLE_DYNAMICS_MODE                  0x10
#define MIP_FILTER_CMD_SENSOR2VEHICLE_TRANSFORMATION          0x11
#define MIP_FILTER_CMD_SENSOR2VEHICLE_OFFSET                  0x12
#define MIP_FILTER_CMD_ANTENNA_OFFSET                         0x13
#define MIP_FILTER_CMD_ESTIMATION_CONTROL					   0x14
#define MIP_FILTER_CMD_GPS_SOURCE_CONTROL                     0x15
#define MIP_FILTER_CMD_EXTERNAL_GPS_UPDATE                    0x16
#define MIP_FILTER_CMD_EXTERNAL_HEADING_UPDATE                0x17
#define MIP_FILTER_CMD_HEADING_UPDATE_CONTROL                 0x18
#define MIP_FILTER_CMD_AUTOINIT_CONTROL                       0x19
#define MIP_FILTER_CMD_ACCEL_NOISE                            0x1A
#define MIP_FILTER_CMD_GYRO_NOISE                             0x1B
#define MIP_FILTER_CMD_ACCEL_BIAS_MODEL                       0x1C
#define MIP_FILTER_CMD_GYRO_BIAS_MODEL                        0x1D
#define MIP_FILTER_CMD_ZERO_VELOCITY_UPDATE                   0x1E
#define MIP_FILTER_CMD_EXTERNAL_HEADING_TIMESTAMP             0x1F
#define MIP_FILTER_CMD_ANGULAR_RATE_ZERO_UPDATE               0x20
#define MIP_FILTER_CMD_TARE_ORIENTATION                       0x21
#define MIP_FILTER_CMD_COMMANDED_ZUPT                         0x22
#define MIP_FILTER_CMD_COMMANDED_ZERO_ANGULAR_RATE_UPDATE     0x23

#define MIP_FILTER_CMD_REFERENCE_POSITION                                  0x26
#define MIP_FILTER_CMD_ENABLE_MEASUREMENT                                  0x41
#define MIP_FILTER_CMD_MAG_NOISE                                           0x42
#define MIP_FILTER_CMD_DECLINATION_SOURCE        		                    0x43
#define MIP_FILTER_CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL  0x44
#define MIP_FILTER_CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL    0x45
#define MIP_FILTER_CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL    0x46


////////////////////////////////////////////////////////////////////////////////
// FILTER REPLY DESCRIPTORS (reply desc are >= 0x80)
////////////////////////////////////////////////////////////////////////////////

#define MIP_FILTER_REPLY_VEHICLE_DYNAMICS_MODE                0x80
#define MIP_FILTER_REPLY_SENSOR2VEHICLE_TRANSFORMATION        0x81
#define MIP_FILTER_REPLY_SENSOR2VEHICLE_OFFSET                0x82
#define MIP_FILTER_REPLY_ANTENNA_OFFSET                       0x83
#define MIP_FILTER_REPLY_ESTIMATION_CONTROL				   0x84
#define MIP_FILTER_REPLY_GPS_SOURCE_CONTROL                   0x86
#define MIP_FILTER_REPLY_HEADING_UPDATE_CONTROL               0x87
#define MIP_FILTER_REPLY_AUTOINIT_CONTROL                     0x88
#define MIP_FILTER_REPLY_ACCEL_NOISE                          0x89
#define MIP_FILTER_REPLY_GYRO_NOISE                           0x8A
#define MIP_FILTER_REPLY_ACCEL_BIAS_MODEL                     0x8B
#define MIP_FILTER_REPLY_GYRO_BIAS_MODEL                      0x8C
#define MIP_FILTER_REPLY_ZERO_VELOCITY_UPDATE                 0x8D
#define MIP_FILTER_REPLY_ZERO_ANGULAR_RATE_UPDATE             0x8E

#define MIP_FILTER_REPLY_REFERENCE_POSITION                                  0x90
#define MIP_FILTER_REPLY_ENABLE_MEASUREMENT                                  0xB0
#define MIP_FILTER_REPLY_MAG_NOISE                                           0xB1
#define MIP_FILTER_REPLY_DECLINATION_SOURCE     		                      0xB2
#define MIP_FILTER_REPLY_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL  0xB3
#define MIP_FILTER_REPLY_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL    0xB4
#define MIP_FILTER_REPLY_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL    0xB5
#define MIP_FILTER_REPLY_MAG_ANGULAR_RATE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL 0xB6




////////////////////////////////////////////////////////////////////////////////
//
// FILTER DATA DESCRIPTORS
//
////////////////////////////////////////////////////////////////////////////////

#define MIP_FILTER_DATA_LLH_POS                               0x01
#define MIP_FILTER_DATA_NED_VEL                               0x02
#define MIP_FILTER_DATA_ATT_QUATERNION                        0x03
#define MIP_FILTER_DATA_ATT_MATRIX                            0x04
#define MIP_FILTER_DATA_ATT_EULER_ANGLES                      0x05
#define MIP_FILTER_DATA_GYRO_BIAS                             0x06
#define MIP_FILTER_DATA_ACCEL_BIAS                            0x07
#define MIP_FILTER_DATA_POS_UNCERTAINTY                       0x08
#define MIP_FILTER_DATA_VEL_UNCERTAINTY                       0x09
#define MIP_FILTER_DATA_ATT_UNCERTAINTY_EULER                 0x0A
#define MIP_FILTER_DATA_GYRO_BIAS_UNCERTAINTY                 0x0B
#define MIP_FILTER_DATA_ACCEL_BIAS_UNCERTAINTY                0x0C
#define MIP_FILTER_DATA_LINEAR_ACCELERATION                   0x0D
#define MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE              0x0E
#define MIP_FILTER_DATA_WGS84_GRAVITY                         0x0F
#define MIP_FILTER_DATA_FILTER_STATUS                         0x10
#define MIP_FILTER_DATA_FILTER_TIMESTAMP                      0x11
#define MIP_FILTER_DATA_ATT_UNCERTAINTY_QUATERNION            0x12
#define MIP_FILTER_DATA_GRAVITY_VECTOR                        0x13
#define MIP_FILTER_DATA_HEADING_UPDATE_STATE                  0x14
#define MIP_FILTER_DATA_MAGNETIC_MODEL                        0x15

#define MIP_FILTER_DATA_GYRO_SCALE_FACTOR                     0x16	  
#define MIP_FILTER_DATA_ACCEL_SCALE_FACTOR                    0x17
#define MIP_FILTER_DATA_GYRO_SCALE_FACTOR_UNCERTAINTY         0x18	  
#define MIP_FILTER_DATA_ACCEL_SCALE_FACTOR_UNCERTAINTY        0x19
#define MIP_FILTER_DATA_MAG_BIAS    		  	               0x1A	
#define MIP_FILTER_DATA_MAG_BIAS_UNCERTAINTY                  0x1B
#define MIP_FILTER_DATA_COMPENSATED_ACCELERATION              0x1C	
#define MIP_FILTER_DATA_STANDARD_ATMOSPHERE_DATA              0x20	
#define MIP_FILTER_DATA_PRESSURE_ALTITUDE                     0x21	
#define MIP_FILTER_DATA_DENSITY_ALTITUDE                      0x22	
#define MIP_FILTER_DATA_ANTENNA_OFFSET_CORRECTION             0x30
#define MIP_FILTER_DATA_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY 0x31	  



////////////////////////////////////////////////////////////////////////////////
// FILTER PARAMETERS
////////////////////////////////////////////////////////////////////////////////

//Bias Estimation Control
#define MIP_FILTER_BIAS_ESTIMATION_OFF 0x00
#define MIP_FILTER_BIAS_ESTIMATION_ON  0x01


//Dynamics Modes
#define MIP_FILTER_DYNAMICS_MODE_PORTABLE    0x01
#define MIP_FILTER_DYNAMICS_MODE_AUTOMOTIVE  0x02
#define MIP_FILTER_DYNAMICS_MODE_AIRBORNE    0x03


//Heading update sources
#define MIP_FILTER_HEADING_SOURCE_NONE          0x00
#define MIP_FILTER_HEADING_SOURCE_MAGNETOMETER  0x01
#define MIP_FILTER_HEADING_SOURCE_GPS_VELOCITY  0x02
#define MIP_FILTER_HEADING_SOURCE_EXTERNAL      0x03

#define IS_FILTER_HEADING_SOURCE(SOURCE) (((SOURCE) == MIP_FILTER_HEADING_SOURCE_NONE)         || \
                                       ((SOURCE) == MIP_FILTER_HEADING_SOURCE_MAGNETOMETER) || \
                                       ((SOURCE) == MIP_FILTER_HEADING_SOURCE_GPS_VELOCITY) || \
                                       ((SOURCE) == MIP_FILTER_HEADING_SOURCE_EXTERNAL))

//Heading update types
#define MIP_FILTER_HEADING_UPDATE_TYPE_TRUE_NORTH      0x01
#define MIP_FILTER_HEADING_UPDATE_TYPE_MAGNETIC_NORTH  0x02

#define IS_FILTER_HEADING_UPDATE_TYPE(TYPE) (((TYPE) == MIP_FILTER_HEADING_UPDATE_TYPE_TRUE_NORTH) || \
                                          ((TYPE) == MIP_FILTER_HEADING_UPDATE_TYPE_MAGNETIC_NORTH))

//Declination Source
#define MIP_FILTER_DECLINATION_SOURCE_NONE     0x01
#define MIP_FILTER_DECLINATION_SOURCE_MODEL    0x02
#define MIP_FILTER_DECLINATION_SOURCE_MANUAL   0x03

#define IS_FILTER_DECLINATION_SOURCE(SOURCE) (((SOURCE) == MIP_FILTER_DECLINATION_SOURCE_NONE)  || \
                                           ((SOURCE) == MIP_FILTER_DECLINATION_SOURCE_MODEL) || \
                                           ((SOURCE) == MIP_FILTER_DECLINATION_SOURCE_MANUAL))



////////////////////////////////////////////////////////////////////////////////
//
// Flag Definitions
//
////////////////////////////////////////////////////////////////////////////////

///
//EKF Modes
///

#define MIP_FILTER_EKF_STATE_STARTUP              0x00
#define MIP_FILTER_EKF_STATE_INIT                 0x01
#define MIP_FILTER_EKF_STATE_RUN_SOLUTION_VALID   0x02
#define MIP_FILTER_EKF_STATE_RUN_SOLUTION_ERROR   0x03

///
//Dynamics Modes
///

#define MIP_FILTER_EKF_DYNAMICS_MODE_PORTABLE    0x01
#define MIP_FILTER_EKF_DYNAMICS_MODE_AUTOMOTIVE  0x02
#define MIP_FILTER_EKF_DYNAMICS_MODE_AIRBORNE    0x03

#define IS_MIP_FILTER_EKF_DYNAMICS_MODE(MODE) (((MODE) == MIP_FILTER_EKF_DYNAMICS_MODE_PORTABLE)   || \
                                            ((MODE) == MIP_FILTER_EKF_DYNAMICS_MODE_AUTOMOTIVE) || \
                                            ((MODE) == MIP_FILTER_EKF_DYNAMICS_MODE_AIRBORNE))

///
//EKF Status Flags
///

#define MIP_FILTER_EKF_STATUS_FLAG_INIT_NO_ATTITUDE            0x1000
#define MIP_FILTER_EKF_STATUS_FLAG_INIT_NO_POSITION_VELOCITY   0x2000

#define MIP_FILTER_EKF_STATUS_FLAG_IMU_UFILTERAILABLE             0x0001
#define MIP_FILTER_EKF_STATUS_FLAG_GPS_UFILTERAILABLE             0x0002
#define MIP_FILTER_EKF_STATUS_FLAG_MATRIX_SINGULARITY          0x0008
#define MIP_FILTER_EKF_STATUS_FLAG_POSITION_COVARIANCE_WARNING 0x0010
#define MIP_FILTER_EKF_STATUS_FLAG_VELOCITY_COVARIANCE_WARNING 0x0020
#define MIP_FILTER_EKF_STATUS_FLAG_ATTITUDE_COVARIANCE_WARNING 0x0040
#define MIP_FILTER_EKF_STATUS_FLAG_NAN_IN_SOLUTION_WARNING     0x0080

#define MIP_FILTER_EKF_STATUS_FLAG_GYRO_BIAS_EST_HIGH_WARNING             0x0100
#define MIP_FILTER_EKF_STATUS_FLAG_ACCEL_BIAS_EST_HIGH_WARNING            0x0200
#define MIP_FILTER_EKF_STATUS_FLAG_GYRO_SCALE_FACTOR_EST_HIGH_WARNING     0x0400
#define MIP_FILTER_EKF_STATUS_FLAG_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING    0x0800
#define MIP_FILTER_EKF_STATUS_FLAG_MAG_BIAS_EST_HIGH_WARNING              0x1000
#define MIP_FILTER_EKF_STATUS_FLAG_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING 0x2000


///
//Bias Estimation Control
///

#define MIP_FILTER_EKF_BIAS_ESTIMATION_CONTROL_OFF 0x00
#define MIP_FILTER_EKF_BIAS_ESTIMATION_CONTROL_ON  0x01

#define IS_MIP_FILTER_EKF_BIAS_ESTIMATION_CONTROL(CONTROL) (((CONTROL)  == MIP_FILTER_EKF_BIAS_ESTIMATION_CONTROL_OFF) || \
                                                         ((CONTROL)  == MIP_FILTER_EKF_BIAS_ESTIMATION_CONTROL_ON))


///
// Heading Update State
///

#define FILTER_HEADING_UPDATE_STATE_SOURCE_VALID         0x0001
#define FILTER_HEADING_UPDATE_STATE_HEADING_VALID        0x0002
#define FILTER_HEADING_UPDATE_STATE_HEADING_1SIGMA_VALID 0x0004

#define FILTER_HEADING_UPDATE_STATE_PACKET_VALID (FILTER_HEADING_UPDATE_STATE_SOURCE_VALID | FILTER_HEADING_UPDATE_STATE_HEADING_VALID | FILTER_HEADING_UPDATE_STATE_HEADING_1SIGMA_VALID)

///
// Estimation Control Flags
///

#define FILTER_ESTIMATION_CONTROL_GYRO_BIAS              0x0001
#define FILTER_ESTIMATION_CONTROL_ACCEL_BIAS             0x0002
#define FILTER_ESTIMATION_CONTROL_GYRO_SCALE_FACTOR      0x0004
#define FILTER_ESTIMATION_CONTROL_ACCEL_SCALE_FACTOR     0x0008
#define FILTER_ESTIMATION_CONTROL_GPS_ANTENNA_OFFSET     0x0010
#define FILTER_ESTIMATION_CONTROL_MASK                   0x001F

///
// Tare Command Axis definitions
///

#define FILTER_TARE_ROLL_AXIS                            0x01
#define FILTER_TARE_PITCH_AXIS                           0x02
#define FILTER_TARE_YAW_AXIS                             0x04


////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////

#pragma pack(1)

////////////////////////////////////////////////////////////////////////////////
//
// Commands
//
////////////////////////////////////////////////////////////////////////////////

///
// External GPS Update Information: GPS Time, Position, Velocity
///

typedef struct _mip_filter_external_gps_update_command
{
 double tow;                //(sec)
 u16    week_number;

 double pos[3];             //Lat, Lon, Height (LLH) Position (deg, deg, m)
 float  vel[3];             //NED Velocity, (m/s)

 float  pos_1sigma[3];      //NED position 1-sigma (m)
 float  vel_1sigma[3];      //NED velocity 1-sigma (m/s)

}mip_filter_external_gps_update_command;


///
// External Heading Update Information: Heading angle
///

typedef struct _mip_filter_external_heading_update_command
{
 float heading_angle;        //(deg, +-180)
 float heading_angle_1sigma; //(deg)
 u8    type;                 // 1 - true north, 2 - magnetic north
}mip_filter_external_heading_update_command;


///
//Zero Velocity/Angular-Rate Update Control
///

typedef struct _mip_filter_zero_update_command
{
 u8 enable;
 float threshold;
}mip_filter_zero_update_command;


///
//External Heading Update with Timestamp
///

typedef struct _mip_filter_external_heading_with_time_command
{
 double	gps_tow;
 u16 gps_week_number;
 float heading_angle_rads;
 float heading_angle_sigma_rads;
 u8 heading_type;
}mip_filter_external_heading_with_time_command;


///
//Accel Magnitude Error Adaptive Measurement Command
///

typedef struct _mip_filter_accel_magnitude_error_adaptive_measurement_command
{
 u8 enable;
 float low_pass_cutoff;
 float low_limit, high_limit;
 float low_limit_1sigma, high_limit_1sigma;
 float min_1sigma;
}mip_filter_accel_magnitude_error_adaptive_measurement_command;


///
//Magnetometer Magnitude Error Adaptive Measurement Command
///

typedef struct _mip_filter_magnetometer_magnitude_error_adaptive_measurement_command
{
 u8 enable;
 float low_pass_cutoff;
 float low_limit, high_limit;
 float low_limit_1sigma, high_limit_1sigma;
 float min_1sigma;
}mip_filter_magnetometer_magnitude_error_adaptive_measurement_command;


///
//Magnetometer Dip Angle Error Adaptive Measurement Command
///

typedef struct _mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command
{
 u8 enable;
 float low_pass_cutoff;
 float high_limit;
 float high_limit_1sigma;
 float min_1sigma;
}mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command;




////////////////////////////////////////////////////////////////////////////////
//
// Data
//
////////////////////////////////////////////////////////////////////////////////


//LLH Position
typedef struct _mip_filter_llh_pos
{
 double latitude, longitude; //Degrees
 double ellipsoid_height;    //meters
 u16    valid_flags;
}mip_filter_llh_pos;



//NED Velocity
typedef struct _mip_filter_ned_velocity
{
 float north, east, down;   //meters/sec
 u16   valid_flags;
}mip_filter_ned_velocity;


//Attitude (quaternion)
typedef struct _mip_filter_attitude_quaternion
{
 float q[4];
 u16   valid_flags;
}mip_filter_attitude_quaternion;


//Attitude (DCM)
typedef struct _mip_filter_attitude_dcm
{
 float dcm[3][3];         //Rows, Columns
 u16   valid_flags;
}mip_filter_attitude_dcm;


//Attitude (Euler Angles)
typedef struct _mip_filter_attitude_euler_angles
{
 float roll, pitch, yaw; //radians
 u16   valid_flags;
}mip_filter_attitude_euler_angles;


//Gyro Bias Estimates
typedef struct _mip_filter_gyro_bias
{
 float x, y, z;    //sensor body frame (radians/sec)
 u16   valid_flags;
}mip_filter_gyro_bias;


//Accel Bias Estimates
typedef struct _mip_filter_accel_bias
{
 float x, y, z;    //sensor body frame (m/s^2)
 u16   valid_flags;
}mip_filter_accel_bias;


//LLH Position Uncertainty
typedef struct _mip_filter_llh_pos_uncertainty
{
 float north, east, down;           //1-sigma (meters)
 u16   valid_flags;
}mip_filter_llh_pos_uncertainty;


//NED Velocity Uncertainty
typedef struct _mip_filter_ned_vel_uncertainty
{
 float north, east, down;           //1-sigma (meters/sec)
 u16   valid_flags;
}mip_filter_ned_vel_uncertainty;


//Attitude Uncertainty (Euler Angles)
typedef struct _mip_filter_euler_attitude_uncertainty
{
 float roll, pitch, yaw;  //1-sigma (radians)
 u16   valid_flags;
}mip_filter_euler_attitude_uncertainty;


//Gyro Bias Uncertainty
typedef struct _mip_filter_gyro_bias_uncertainty
{
 float x, y, z;    //sensor body frame (radians/sec)
 u16   valid_flags;
}mip_filter_gyro_bias_uncertainty;


//Accel Bias Uncertainty
typedef struct _mip_filter_accel_bias_uncertainty
{
 float x, y, z;    //sensor body frame (m/s^2)
 u16   valid_flags;
}mip_filter_accel_bias_uncertainty;


//Navigation Solution Timetamp
typedef struct _mip_filter_timestamp
{
 double tow;  //Time of Week (seconds)
 u16    week_number;
 u16    valid_flags;
}mip_filter_timestamp;


//Navigation Status
typedef struct _mip_filter_status
{
 u16 filter_state;
 u16 dynamics_mode;
 u16 status_flags;
}mip_filter_status;


//Nav Linear Acceleration Estimate
typedef struct _mip_filter_linear_acceleration
{
 float x, y, z;   //sensor or vehicle frame (m/s^2)
 u16   valid_flags;
}mip_filter_linear_acceleration;


//Nav Compensated Acceleration Estimate
typedef struct _mip_filter_compensated_acceleration
{
 float x, y, z;   //sensor or vehicle frame (m/s^2)
 u16   valid_flags;
}mip_filter_compensated_acceleration;


//Nav Gravity Estimate
typedef struct _mip_filter_gravity_vector
{
 float x, y, z;   //sensor or vehicle frame (m/s^2)
 u16   valid_flags;
}mip_filter_gravity_vector;


//Nav Compensated Angular Rate Estimate
typedef struct _mip_filter_compensated_angular_rate
{
 float x, y, z;   //sensor or vehicle frame (rad/s)
 u16   valid_flags;
}mip_filter_compensated_angular_rate;


//Attitude Uncertainty (Quaternion Elements)
typedef struct _mip_filter_quaternion_attitude_uncertainty
{
 float q0, q1, q2, q3;  //1-sigma (radians)
 u16   valid_flags;
}mip_filter_quaternion_attitude_uncertainty;


//WGS84 Gravity magnitude
typedef struct _mip_filter_wgs84_gravity_mag
{
 float magnitude;  //m/s^2
 u16   valid_flags;
}mip_filter_wgs84_gravity_mag;


//Heading Update Source
typedef struct _mip_filter_heading_update_state
{
 float heading;
 float heading_1sigma;
 u16   source;
 u16   valid_flags;
}mip_filter_heading_update_state;


//Magnetic Model
typedef struct _mip_filter_magnetic_model
{
 float intensity_north, intensity_east, intensity_down; //nT
 float inclination, declination;                        //deg (AKA Dip, Magnetic Variation)
 u16   valid_flags;
}mip_filter_magnetic_model;


//Accel Scale Factor Estimates
typedef struct _mip_filter_accel_scale_factor_mip_field
{
 float x, y, z;    //sensor body frame
 u16   valid_flags; 
}mip_filter_accel_scale_factor_mip_field;


//Accel Scale Factor Uncertainty
typedef struct _mip_filter_accel_scale_factor_uncertainty_mip_field
{
 float x, y, z;    //sensor body frame
 u16   valid_flags; 
}mip_filter_accel_scale_factor_uncertainty_mip_field;


//Gyro Scale Factor Estimates
typedef struct _mip_filter_gyro_scale_factor_mip_field
{
 float x, y, z;    //sensor body frame
 u16   valid_flags; 
}mip_filter_gyro_scale_factor_mip_field;


//Gyro Scale Factor Uncertainty
typedef struct _mip_filter_gyro_scale_factor_uncertainty_mip_field
{
 float x, y, z;    //sensor body frame
 u16   valid_flags; 
}mip_filter_gyro_scale_factor_uncertainty_mip_field;


//Mag Bias Estimates
typedef struct _mip_filter_mag_bias_mip_field
{
 float x, y, z;    //sensor body frame (Gauss)
 u16   valid_flags; 
}mip_filter_mag_bias_mip_field;


//Mag Bias Uncertainty
typedef struct _mip_filter_mag_bias_uncertainty_mip_field
{
 float x, y, z;    //sensor body frame (Gauss)
 u16   valid_flags; 
}mip_filter_mag_bias_uncertainty_mip_field;


//Compensated Mag Vector Estimate
typedef struct _mip_filter_compensated_mag_vector_mip_field 
{
 float x, y, z;   //sensor body frame (Gauss)
 u16   valid_flags;
}mip_filter_compensated_mag_vector_mip_field;


//Standard Atmosphere
typedef struct _mip_filter_standard_atmosphere_mip_field
{
 float geometric_altitude;    //m, input into calculation
 float geopotential_altitude; //m
 float standard_temperature;  //degC
 float standard_pressure;     //mBar
 float standard_density;      //kg/m^3
 u16   valid_flags; 
}mip_filter_standard_atmosphere_mip_field;


//Pressure Altitude
typedef struct _mip_filter_pressure_altitude_mip_field
{
 float pressure_altitude;    //m
 u16   valid_flags; 
}mip_filter_pressure_altitude_mip_field;


//Density Altitude
typedef struct _mip_filter_density_altitude_mip_field
{
 float density_altitude;    //m
 u16   valid_flags; 
}mip_filter_density_altitude_mip_field;


//GPS Antenna Offset Correction Estimates
typedef struct _mip_filter_antenna_offset_correction_mip_field
{
 float x, y, z;    //sensor body frame
 u16   valid_flags; 
}mip_filter_antenna_offset_correction_mip_field;


//GPS Antenna Offset Correction Uncertainty
typedef struct _mip_filter_antenna_offset_correction_uncertainty_mip_field
{
 float x, y, z;    //sensor body frame
 u16   valid_flags; 
}mip_filter_antenna_offset_correction_uncertainty_mip_field;



#pragma pack()





////////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
////////////////////////////////////////////////////////////////////////////////


///
//FILTER Data
///

void mip_filter_llh_pos_byteswap(mip_filter_llh_pos *llh_pos);
void mip_filter_ned_velocity_byteswap(mip_filter_ned_velocity *ned_velocity);
void mip_filter_attitude_quaternion_byteswap(mip_filter_attitude_quaternion *attitude_quaternion);
void mip_filter_attitude_dcm_byteswap(mip_filter_attitude_dcm *attitude_dcm);
void mip_filter_attitude_euler_angles_byteswap(mip_filter_attitude_euler_angles *attitude_euler_angles);
void mip_filter_gyro_bias_byteswap(mip_filter_gyro_bias *gyro_bias);
void mip_filter_accel_bias_byteswap(mip_filter_accel_bias *accel_bias);
void mip_filter_llh_pos_uncertainty_byteswap(mip_filter_llh_pos_uncertainty *llh_pos_uncertainty);
void mip_filter_ned_vel_uncertainty_byteswap(mip_filter_ned_vel_uncertainty *ned_vel_uncertainty);
void mip_filter_euler_attitude_uncertainty_byteswap(mip_filter_euler_attitude_uncertainty *euler_attitude_uncertainty);
void mip_filter_gyro_bias_uncertainty_byteswap(mip_filter_gyro_bias_uncertainty *gyro_bias_uncertainty);
void mip_filter_accel_bias_uncertainty_byteswap(mip_filter_accel_bias_uncertainty *accel_bias_uncertainty);
void mip_filter_timestamp_byteswap(mip_filter_timestamp *timestamp);
void mip_filter_status_byteswap(mip_filter_status *status);
void mip_filter_linear_acceleration_byteswap(mip_filter_linear_acceleration *acceleration);
void mip_filter_compensated_acceleration_byteswap(mip_filter_compensated_acceleration *acceleration);
void mip_filter_gravity_vector_byteswap(mip_filter_gravity_vector *gravity_vector);
void mip_filter_compensated_angular_rate_byteswap(mip_filter_compensated_angular_rate *angular_rate);
void mip_filter_quaternion_attitude_uncertainty_byteswap(mip_filter_quaternion_attitude_uncertainty *quaternion_attitude_uncertainty);
void mip_filter_wgs84_gravity_mag_byteswap(mip_filter_wgs84_gravity_mag *wgs84_gravity_mag);
void mip_filter_heading_update_state_byteswap(mip_filter_heading_update_state *heading_update_state);
void mip_filter_magnetic_model_byteswap(mip_filter_magnetic_model *magnetic_model);
void mip_filter_external_heading_update_with_time_byteswap(mip_filter_external_heading_with_time_command *external_heading_command);


void mip_filter_accel_scale_factor_byteswap(mip_filter_accel_scale_factor_mip_field *accel_scale_factor);
void mip_filter_accel_scale_factor_uncertainty_byteswap(mip_filter_accel_scale_factor_uncertainty_mip_field *accel_scale_factor_uncertainty);
void mip_filter_gyro_scale_factor_byteswap(mip_filter_gyro_scale_factor_mip_field *gyro_scale_factor);
void mip_filter_gyro_scale_factor_uncertainty_byteswap(mip_filter_gyro_scale_factor_uncertainty_mip_field *gyro_scale_factor_uncertainty);
void mip_filter_mag_bias_byteswap(mip_filter_mag_bias_mip_field *mag_bias);
void mip_filter_mag_bias_uncertainty_byteswap(mip_filter_mag_bias_uncertainty_mip_field *mag_bias_uncertainty);
void mip_filter_compensated_mag_vector_byteswap(mip_filter_compensated_mag_vector_mip_field *compensated_mag_vector);
void mip_filter_standard_atmosphere_byteswap(mip_filter_standard_atmosphere_mip_field *standard_atmosphere);
void mip_filter_pressure_altitude_byteswap(mip_filter_pressure_altitude_mip_field *pressure_altitude);
void mip_filter_density_altitude_byteswap(mip_filter_density_altitude_mip_field *density_altitude);
void mip_filter_antenna_offset_correction_byteswap(mip_filter_antenna_offset_correction_mip_field *antenna_offset_correction);
void mip_filter_antenna_offset_correction_uncertainty_byteswap(mip_filter_antenna_offset_correction_uncertainty_mip_field *antenna_offset_correction_uncertainty);


///
//FILTER Commands
///

u16 mip_filter_reset_filter(mip_interface *device_interface);
u16 mip_filter_set_init_attitude(mip_interface *device_interface, float euler_angles[3]);
u16 mip_filter_set_init_heading(mip_interface *device_interface, float heading);
u16 mip_filter_set_init_attitude_from_ahrs(mip_interface *device_interface, float declination);
u16 mip_filter_vehicle_dynamics_mode(mip_interface *device_interface, u8 function_selector, u8 *dynamics_mode);
u16 mip_filter_sensor2vehicle_tranformation(mip_interface *device_interface, u8 function_selector, float euler_angles[3]);
u16 mip_filter_sensor2vehicle_offset(mip_interface *device_interface, u8 function_selector, float offset[3]);
u16 mip_filter_antenna_offset(mip_interface *device_interface, u8 function_selector, float offset[3]);
u16 mip_filter_gps_source(mip_interface *device_interface, u8 function_selector, u8 *gps_source);
u16 mip_filter_external_gps_update(mip_interface *device_interface, mip_filter_external_gps_update_command *command);
u16 mip_filter_external_heading_update(mip_interface *device_interface, mip_filter_external_heading_update_command *command);
u16 mip_filter_heading_source(mip_interface *device_interface, u8 function_selector, u8 *heading_source);
u16 mip_filter_auto_initialization(mip_interface *device_interface, u8 function_selector, u8 *enable);
u16 mip_filter_accel_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3]);
u16 mip_filter_accel_bias_model(mip_interface *device_interface, u8 function_selector, float bias_beta[3], float bias_noise_1sigma[3]);
u16 mip_filter_gyro_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3]);
u16 mip_filter_gyro_bias_model(mip_interface *device_interface, u8 function_selector, float bias_beta[3], float bias_noise_1sigma[3]);
u16 mip_filter_zero_velocity_update_control(mip_interface *device_interface, u8 function_selector,  mip_filter_zero_update_command *zero_velocity_control);
u16 mip_filter_zero_angular_rate_update_control(mip_interface *device_interface, u8 function_selector, mip_filter_zero_update_command *zero_angular_rate_control);
u16 mip_filter_tare_orientation(mip_interface *device_interface, u8 function_selector, u8 tare_bitfield);
u16 mip_filter_commanded_zero_velocity_update(mip_interface *device_interface);
u16 mip_filter_commanded_zero_angular_rate_update(mip_interface *device_interface);
u16 mip_filter_external_heading_update_with_time(mip_interface *device_interface, mip_filter_external_heading_with_time_command *heading_with_time_command);

u16 mip_filter_mag_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3]);
u16 mip_filter_reference_position(mip_interface *device_interface,  u8 function_selector, u8 *reference_enable, double reference_position[3]);
u16 mip_filter_estimation_control(mip_interface *device_interface, u8 function_selector, u16 *estimation_control);
u16 mip_filter_enable_measurement(mip_interface *device_interface, u8 function_selector, u16 *measurement_enable); 
u16 mip_filter_declination_source(mip_interface *device_interface, u8 function_selector, u8 *declination_source);
u16 mip_filter_accel_magnitude_error_adaptive_measurement(mip_interface *device_interface, u8 function_selector, mip_filter_accel_magnitude_error_adaptive_measurement_command *params);
u16 mip_filter_mag_magnitude_error_adaptive_measurement(mip_interface *device_interface, u8 function_selector, mip_filter_magnetometer_magnitude_error_adaptive_measurement_command *params); 
u16 mip_filter_mag_dip_angle_error_adaptive_measurement(mip_interface *device_interface, u8 function_selector, mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command *params); 

#endif
