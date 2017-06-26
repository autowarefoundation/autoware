/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_gx4_45.h 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP GX4-45 Specific Structures and Definitions
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

#ifndef _MIP_GX4_45_H
#define _MIP_GX4_45_H

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
// GX4 PARAMETERS
////////////////////////////////////////////////////////////////////////////////


#define GX4_45_MODEL_NUMBER    6236

#define GX4_45_BASIC_STATUS_SEL		  1
#define GX4_45_DIAGNOSTICS_STATUS_SEL 2

#define GX4_45_SYSTEM_STATE_INIT           0x01
#define GX4_45_SYSTEM_STATE_SENSOR_STARTUP 0x02
#define GX4_45_SYSTEM_STATE_RUNNING        0x03


////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////

#pragma pack(1)

///
// Device Basic Status (returned in MIP_REPLY_DESC_3DM_DEVICE_STATUS field
// when status_selector = GX4_45_BASIC_STATUS_SEL)
///

typedef struct _gx4_45_basic_status_field
{
 u16 device_model;		// always GX4_45_MODEL_NUMBER
 u8  status_selector;	// always GX4_45_BASIC_STATUS_SEL
 
 u32 status_flags;
 
 //System variables
 u16 system_state;
 u32 system_timer_ms;
}gx4_45_basic_status_field;


///
// Device Diagnostic Status (returned in MIP_REPLY_DESC_3DM_DEVICE_STATUS field
// when status_selector = GX4_45_DIAGNOSTICS_STATUS_SEL)
///

typedef struct _gx4_45_diagnostic_status_field
{
 u16 device_model;		// always GX4_45_MODEL_NUMBER
 u8  status_selector;	// always GX4_45_DIAGNOSTICS_STATUS_SEL
 
 u32 status_flags;
 
 //System variables
 u16 system_state;
 u32 system_timer_ms;
 
 //Power control
 u8 gps_power_on;
  
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
  
}gx4_45_diagnostic_device_status_field;


#pragma pack()

#endif
