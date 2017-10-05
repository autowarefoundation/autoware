/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_rq1_imu.h 
//! @authors  Nathan Miller and Gregg Carpenter
//! @version 1.1
//
//! @description MIP RQ1_IMU Specific Structures and Definitions
//
// External dependencies:
//
//  mip_types.h
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



#ifndef _MIP_RQ1_IMU_H
#define _MIP_RQ1_IMU_H

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
// RQ1_IMU PARAMETERS
////////////////////////////////////////////////////////////////////////////////

#define RQ1_IMU_MODEL_NUMBER    6238

#define RQ1_IMU_BASIC_STATUS_SEL		1
#define RQ1_IMU_DIAGNOSTICS_STATUS_SEL		2

////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////

#pragma pack(1)

///
// Device Basic Status (returned in MIP_REPLY_DESC_3DM_DEVICE_STATUS field
// when status_selector = RQ1_BASIC_STATUS_SEL)
///

typedef struct _rq1_imu_basic_status_field
{
 u16 device_model;		// always RQ1_IMU_MODEL_NUMBER
 u8  status_selector;	// always RQ1_IMU_BASIC_STATUS_SEL
 
 u32 status_flags;
 
 //System variables
 u64 system_timer_us;
}rq1_imu_basic_status_field;


///
// Device Diagnostic Status (returned in MIP_REPLY_DESC_3DM_DEVICE_STATUS field
// when status_selector = RQ1_IMU_DIAGNOSTICS_STATUS_SEL)
///

typedef struct _rq1_imu_diagnostic_status_field
{
 u16 device_model;		// always RQ1_MODEL_NUMBER
 u8  status_selector;	// always RQ1_IMU_DIAGNOSTICS_STATUS_SEL
 
 u32 status_flags;
 
 //System variables
 u64 system_timer_us;
 
 //On-board temp sensors
 float temp_degc;
 u32 last_temp_read_ms;
 u8 bad_temp_sensor_detected;
 u8 cold_temp_detected;

 //Power supply errors
 u8 bad_voltage_detected;
  
 //GPS PPS Stats
 u32 num_gps_pps_triggers;
 u32 last_gps_pps_trigger_ms;
 
 //Enabled stream
 u8 stream_enabled;

 //Dropped packets (number of packets)
 u32 dropped_packets;
 
 //Com port stats
 u16 com_port_state;
 u32 com_port_baudrate;
 u32 com_port_bytes_written,  com_port_bytes_read;
 u32 com_port_write_overruns, com_port_read_overruns;
  
}rq1_imu_diagnostic_device_status_field;


#pragma pack()

#endif
