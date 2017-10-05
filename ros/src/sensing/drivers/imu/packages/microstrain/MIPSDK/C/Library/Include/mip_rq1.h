/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_rq1.h 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP RQ1 Specific Structures and Definitions
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



#ifndef _MIP_RQ1_H
#define _MIP_RQ1_H

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




////////////////////////////////////////////////////////////////////////////////
// RQ-1 PARAMETERS
////////////////////////////////////////////////////////////////////////////////

#define RQ1_MODEL_NUMBER    6232

#define RQ1_BASIC_STATUS_SEL		1
#define RQ1_DIAGNOSTICS_STATUS_SEL	2

#define RQ1_SYSTEM_STATE_INITIALIZATION 0x0001
#define RQ1_SYSTEM_STATE_SENSOR_STARTUP	0x0002
#define RQ1_SYSTEM_STATE_RUNNING	0x0003

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

typedef struct _rq1_basic_status_field
{
 u16 device_model;		// always RQ1_MODEL_NUMBER
 u8  status_selector;	// always RQ1_BASIC_STATUS_SEL
 
 u32 status_flags;
 
 //System variables
 u16 system_state;
 u32 system_timer_ms;
}rq1_basic_status_field;


///
// Device Diagnostic Status (returned in MIP_REPLY_DESC_3DM_DEVICE_STATUS field
// when status_selector = RQ1_DIAGNOSTICS_STATUS_SEL)
///

typedef struct _rq1_diagnostic_status_field
{
 u16 device_model;		// always RQ1_MODEL_NUMBER
 u8  status_selector;	// always RQ1_DIAGNOSTICS_STATUS_SEL
 
 u32 status_flags;
 
 //System variables
 u16 system_state;
 u32 system_timer_ms;
 
 //Power control
 u8 gps_power_on;
 
 //On-board temp 
 u8    cold_condition;
 float temp_degc;
 u32   last_temp_read_ms;
 
 //GPS PPS Stats
 u32 num_gps_pps_triggers;
 u32 last_gps_pps_trigger_ms;
 
 //Enabled streams
 u8 imu_stream_enabled;
 u8 gps_stream_enabled;
 u8 filter_stream_enabled;

 //Dropped packets (number of packets)
 u32 imu_dropped_packets;
 u32 gps_dropped_packets;
 u32 filter_dropped_packets;
 
 //Com port stats
 u32 com1_port_bytes_written,  com1_port_bytes_read;
 u32 com1_port_write_overruns, com1_port_read_overruns;
 
 //IMU Interface stats
 u32 imu_parser_errors;
 u32 imu_message_count;
 u32 imu_last_message_ms;

 //GPS Interface stats
 u32 gps_parser_errors;
 u32 gps_message_count;
 u32 gps_last_message_ms;
  
}rq1_diagnostic_device_status_field;


#pragma pack()

#endif
