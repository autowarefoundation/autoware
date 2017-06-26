/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_gx4_15.h - 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description  MIP GX4 Specific Structures and Definitions
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


#ifndef _MIP_GX4_15_H
#define _MIP_GX4_15_H

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


#define GX4_15_MODEL_NUMBER             6233

#define GX4_15_BASIC_STATUS_SEL		    1
#define GX4_15_DIAGNOSTICS_STATUS_SEL	2




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

typedef struct _gx4_15_basic_status_field
{
 u16 device_model;		// always GX4_MODEL_NUMBER
 u8  status_selector;	// always GX4_BASIC_STATUS_SEL
 
 u32 status_flags;
 
 //System variables
 u32 system_timer_ms;
}gx4_15_basic_status_field;


///
// Device Diagnostic Status (returned in MIP_REPLY_DESC_3DM_DEVICE_STATUS field
// when status_selector = GX4_DIAGNOSTICS_STATUS_SEL)
///

typedef struct _gx4_15_diagnostic_status_field
{
 u16 device_model;		// always GX4_MODEL_NUMBER
 u8  status_selector;	// always GX4_DIAGNOSTICS_STATUS_SEL
 
 u32 status_flags;
 
 //System variables
 u32 system_timer_ms;
 
 //PPS Stats
 u32 num_pps_triggers;
 u32 last_pps_trigger_ms;
 
 //Enabled streams
 u8 imu_stream_enabled;
 u8 filter_stream_enabled;

 //Dropped packets (number of packets)
 u32 imu_dropped_packets;
 u32 filter_dropped_packets;
 
 //Com port stats
 u32 com1_port_bytes_written,  com1_port_bytes_read;
 u32 com1_port_write_overruns, com1_port_read_overruns;
 
 //USB stats
 u32 usb_bytes_written,  usb_bytes_read;
 u32 usb_write_overruns, usb_read_overruns;
 
 //IMU Interface stats
 u32 imu_parser_errors;
 u32 imu_message_count;
 u32 imu_last_message_ms;

}gx4_15_diagnostic_device_status_field;


#pragma pack()

#endif
