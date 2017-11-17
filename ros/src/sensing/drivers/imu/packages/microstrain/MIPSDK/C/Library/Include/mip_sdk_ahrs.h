/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_ahrs.h 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP AHRS Descriptor Set Definitions
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

#ifndef _MIP_SDK_AHRS_H
#define _MIP_SDK_AHRS_H

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
// Descriptor Set designator - used in the Desc Set field of the MIP header
//
////////////////////////////////////////////////////////////////////////////////

#define MIP_AHRS_DATA_SET					0x80


////////////////////////////////////////////////////////////////////////////////
//
// Descriptors 
//
////////////////////////////////////////////////////////////////////////////////

#define MIP_AHRS_DATA_ACCEL_RAW					0x01	// 12: 3 single (vector) (no units)
#define MIP_AHRS_DATA_GYRO_RAW		      		0x02	// 12: 3 single (vector) (no units)
#define MIP_AHRS_DATA_MAG_RAW		      		0x03	// 12: 3 single (vector) (no units)
#define MIP_AHRS_DATA_ACCEL_SCALED		  		0x04	// 12: 3 single (vector) (g)
#define MIP_AHRS_DATA_GYRO_SCALED		  		0x05	// 12: 3 single (vector) (radians/second)
#define MIP_AHRS_DATA_MAG_SCALED		 		0x06 	// 12: 3 single (vector) (gauss)
#define MIP_AHRS_DATA_DELTA_THETA		  		0x07	// 12: 3 single (vector) (radians/second)
#define MIP_AHRS_DATA_DELTA_VELOCITY	   		0x08	// 12: 3 single (vector) (meters/second)
#define MIP_AHRS_DATA_ORIENTATION_MATRIX 		0x09	// 36: 9 single (Matrix) (no units)
#define MIP_AHRS_DATA_QUATERNION				0x0A	// 16: 4 single (Quaternion) (no units)
#define MIP_AHRS_DATA_ORIENTATION_UPDATE_MATRIX 0x0B	// 36: 9 single (Matrix) (no units)
#define MIP_AHRS_DATA_EULER_ANGLES				0x0C 
#define MIP_AHRS_DATA_TEMPERATURE_RAW	 		0x0D 	// 8:  4 u16  		(no units)
#define MIP_AHRS_DATA_TIME_STAMP_INTERNAL		0x0E	// 4:  1 u32 (Integer) (device specific)
#define MIP_AHRS_DATA_TIME_STAMP_PPS			0x0F	// 9:  u8 flags u32 seconds u32 nanoseconds
#define MIP_AHRS_DATA_STAB_MAG	      			0x10	// 12: 3 single (vector) (Gauss)
#define MIP_AHRS_DATA_STAB_ACCEL				0x11	// 12: 3 single (vector) (g)
#define MIP_AHRS_DATA_TIME_STAMP_GPS			0x12	// 14: double GPS TOW u16 GPS week number u16 flags 
#define MIP_AHRS_DATA_PRESSURE_SCALED		    0x17 



#define MIP_AHRS_DATA_ASPP			      		0x81	// MicroStrain ASPP packet
#define MIP_AHRS_DATA_GXSB		         		0x82	// MicroStrain GX series single byte command



////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////

///
// Raw sensor 
///

#pragma pack(1)

typedef struct _mip_ahrs_raw_accel 
{
 float raw_accel[3];       //Counts
}mip_ahrs_raw_accel;

typedef struct _mip_ahrs_raw_gyro 
{
 float raw_gyro[3];       //Counts
}mip_ahrs_raw_gyro;

typedef struct _mip_ahrs_raw_mag 
{
 float raw_mag[3];       //Counts
}mip_ahrs_raw_mag;


///
// Scaled/Temp-Comped sensor 
///

typedef struct _mip_ahrs_scaled_accel 
{
 float scaled_accel[3];       //"G"s
}mip_ahrs_scaled_accel;

typedef struct _mip_ahrs_scaled_gyro
{
 float scaled_gyro[3];       //radians/sec
}mip_ahrs_scaled_gyro;

typedef struct _mip_ahrs_scaled_mag 
{
 float scaled_mag[3];       //Gauss
}mip_ahrs_scaled_mag;

typedef struct _ahrs_scaled_pressure_mip_field 
{
 float scaled_pressure;       //mBar
}ahrs_scaled_pressure_mip_field;


///
// Delta Theta/Velocity
///

typedef struct _mip_ahrs_delta_theta 
{
 float delta_theta[3];       //radians/sec
}mip_ahrs_delta_theta;

typedef struct _mip_ahrs_delta_velocity 
{
 float delta_velocity[3];       //meters/sec
}mip_ahrs_delta_velocity;


///
// Orientations
///

typedef struct _mip_ahrs_orientation_matrix 
{
 float m[3][3];    
}mip_ahrs_orientation_matrix;

typedef struct _mip_ahrs_quaternion 
{
 float q[4];    
}mip_ahrs_quaternion;

typedef struct _mip_ahrs_euler_angles
{
 float roll, pitch, yaw;    
}mip_ahrs_euler_angles;


///
// Orientation Update 
///

typedef struct _mip_ahrs_orientation_update_matrix
{
 float m[3][3];    
}mip_ahrs_orientation_update_matrix;

///
// Raw Temperature 
///

typedef struct _mip_ahrs_raw_temp 
{
 u16 raw_temp[4];       //0 = Mag, 1 = Gyro_3/Accel_3, 2 = Gyro_2/Accel_1/Accel_2, 3 = Gyro_1
}mip_ahrs_raw_temp;

///
// Timestamps
///

typedef struct _mip_ahrs_internal_timestamp 
{
 u32 counts; // 16 uS increments, 1/62,500 of a second
}mip_ahrs_internal_timestamp;


typedef struct _mip_ahrs_1pps_timestamp 
{
 u8  flags;
 u32 seconds, nanoseconds; // seconds, nanoseconds
}mip_ahrs_1pps_timestamp;


typedef struct _mip_ahrs_gps_timestamp 
{
 double tow;  //Time of Week (seconds)
 u16 week_number; 
 u16 valid_flags;
}mip_ahrs_gps_timestamp;


///
// Unit Vector Fields
///

typedef struct _mip_ahrs_up_vector 
{
 float up[3];       //Unit Vector
}mip_ahrs_up_vector;


typedef struct _mip_ahrs_north_vector 
{
 float north[3];       //Unit Vector
}mip_ahrs_north_vector;


///
// AHRS Signal Conditioning Settings
///

typedef struct _mip_ahrs_signal_settings
{
 u16 orientation_decimation;
 u16 data_conditioning_flags;
 u8  inertial_filter_width;
 u8  mag_filter_width;
 u16 up_compensation;
 u16 north_compensation;
 u8  mag_bandwidth;
 u16 reserved;
}mip_ahrs_signal_settings;


///
// Complementary Filter Settings
///

typedef struct _mip_complementary_filter_settings
{
 u8    up_compensation_enable;
 u8    north_compensation_enable;
 float up_compensation_time_constant;
 float north_compensation_time_constant;
}mip_complementary_filter_settings;


///
// Scaled data low pass filter settings
///

typedef struct _mip_low_pass_filter_settings
{
 u8		data_type;
 u8		filter_type_selector;
 u8		manual_cutoff;
 u16	cutoff_frequency;
 u8		reserved;	
}mip_low_pass_filter_settings;

#pragma pack()

////////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
////////////////////////////////////////////////////////////////////////////////

void mip_ahrs_raw_accel_byteswap(mip_ahrs_raw_accel *raw_accel);
void mip_ahrs_raw_gyro_byteswap(mip_ahrs_raw_gyro *raw_gyro);
void mip_ahrs_raw_mag_byteswap(mip_ahrs_raw_mag *raw_mag);
void mip_ahrs_scaled_accel_byteswap(mip_ahrs_scaled_accel *scaled_accel);
void mip_ahrs_scaled_gyro_byteswap(mip_ahrs_scaled_gyro *scaled_gyro);
void mip_ahrs_scaled_mag_byteswap(mip_ahrs_scaled_mag *scaled_mag);
void mip_ahrs_delta_theta_byteswap(mip_ahrs_delta_theta *delta_theta);
void mip_ahrs_delta_velocity_byteswap(mip_ahrs_delta_velocity *delta_velocity);
void mip_ahrs_orientation_matrix_byteswap(mip_ahrs_orientation_matrix *orientation_matrix);
void mip_ahrs_quaternion_byteswap(mip_ahrs_quaternion *quaternion);
void mip_ahrs_euler_angles_byteswap(mip_ahrs_euler_angles *euler_angles);
void mip_ahrs_orientation_update_matrix_byteswap(mip_ahrs_orientation_update_matrix *orientation_update_matrix);
void mip_ahrs_raw_temp_byteswap(mip_ahrs_raw_temp *raw_temp);
void mip_ahrs_internal_timestamp_byteswap(mip_ahrs_internal_timestamp *internal_timestamp);
void mip_ahrs_1pps_timestamp_byteswap(mip_ahrs_1pps_timestamp *pps_timestamp);
void mip_ahrs_gps_timestamp_byteswap(mip_ahrs_gps_timestamp *gps_timestamp);
void mip_ahrs_up_vector_byteswap(mip_ahrs_up_vector *up_vector);
void mip_ahrs_north_vector_byteswap(mip_ahrs_north_vector *north_vector);
void mip_ahrs_signal_settings_byteswap(mip_ahrs_signal_settings *signal_settings);


#endif
