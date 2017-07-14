/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_gx4_imu.h
//! @authors Gregg Carpenter and Nathan Miller
//! @version 1.1
//
//! @description MIP GX4 IMU Specific Structures and Definitions
//
// External dependencies:
//
//  None
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


#ifndef _MIP_GX4_IMU_H
#define _MIP_GX4_IMU_H

////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include "mip.h"

////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
////////////////////////////////////////////////////////////////////////////////

//! @def

////////////////////////////////////////////////////////////////////////////////
// GX4 PARAMETERS
////////////////////////////////////////////////////////////////////////////////

#define GX4_IMU_MODEL_NUMBER            6237

#define GX4_IMU_BASIC_STATUS_SEL	1
#define GX4_IMU_DIAGNOSTICS_STATUS_SEL	2

////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////

#pragma pack(1)

///
// Device Basic Status (returned in MIP_REPLY_DESC_3DM_DEVICE_STATUS field
// when status_selector = GX4_BASIC_STATUS_SEL)
///

typedef struct _gx4_imu_basic_status_field
{
 u16 device_model;		// always GX4_MODEL_NUMBER
 u8  status_selector;	// always GX4_BASIC_STATUS_SEL
 
 u32 status_flags;
 
 //System variables
 u32 system_timer_ms;
}gx4_imu_basic_status_field;


///
// Device Diagnostic Status (returned in MIP_REPLY_DESC_3DM_DEVICE_STATUS field
// when status_selector = GX4_DIAGNOSTICS_STATUS_SEL)
///

typedef struct _gx4_imu_diagnostic_status_field
{
 u16 device_model;		// always GX4_IMU_MODEL_NUMBER
 u8  status_selector;	// always GX4_DIAGNOSTICS_STATUS_SEL
 
 u32 status_flags;
 
 //System variables
 u32 system_timer_ms;
 
 //factory settings
 u8 has_mag;
 u8 has_pressure;

 //Sensor parameters
 u16 gyro_range;
 u8  accel_range;
 float mag_range;
 float pressure_range;

 //Temperature
 float temp_degc;
 u32 last_temp_read_ms;
 u8 temp_sensor_error;

 //GPS PPS Stats
 u32 num_gps_pps_triggers;
 u32 last_gps_pps_trigger_ms;

 //Enabled streams
 u8 stream_enabled;
 
 //Dropped packets (number of packets)
 u32 dropped_packets;
 
 //Com port stats
 u32 com_port_bytes_written,  com_port_bytes_read;
 u32 com_port_write_overruns, com_port_read_overruns;
 
}gx4_imu_diagnostic_device_status_field;


#pragma pack()

#endif
