/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_gx3_35.h 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP GX3-35 Specific Structures and Definitions
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


#ifndef _MIP_GX3_35_H
#define _MIP_GX3_35_H

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
// GX3-35 PARAMETERS
////////////////////////////////////////////////////////////////////////////////

//GX3-35 Status parameters

#define GX3_35_MODEL_NUMBER			6225





////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////

#pragma pack(1)

///
// Device Basic Status (returned in MIP_REPLY_DESC_3DM_DEVICE_STATUS field
// when status_selector = 3DM_GX3_35_BASIC_STATUS_SEL)
///

#define GX3_35_BASIC_STATUS_SEL		1

typedef struct _gx3_35_basic_status_field
{
 u16 device_model;		// always 3DM_GX3_35_MODEL_NUMBER
 u8  status_selector;	// always 3DM_GX3_35_BASIC_STATUS_SEL
 u8  com_mode;
 u8  com_device;
 u32 settings_flags;
 
 u16 com1_port_state;
 u32 com1_port_baudrate;
}gx3_35_basic_status_field;

///
// Device Diagnostic Status (returned in MIP_REPLY_DESC_3DM_DEVICE_STATUS field
// when status_selector = 3DM_GX3_35_DIAGNOSTICS_STATUS_SEL)
///

#define GX3_35_DIAGNOSTICS_STATUS_SEL	2

typedef struct _gx3_35_diagnostic_status_field
{
 u16 device_model;		// always 3DM_GX3_35_MODEL_NUMBER
 u8  status_selector;	// always 3DM_GX3_35_DIAGNOSTICS_STATUS_SEL
 u8  com_mode;
 u8  com_device;
 u32 settings_flags;
 
 u16 com1_port_state;
 u32 com1_port_baudrate;
 u32 com1_port_bytes_written,  com1_port_bytes_read;
 u32 com1_port_write_overruns, com1_port_read_overruns;
 
 u16 usb_port_state;
 u32 usb_port_bytes_written,  usb_port_bytes_read; 
 u32 usb_port_write_overruns, usb_port_read_overruns;

 u16 gps_driver_state;
 u16 gps_port_state;
 u32 gps_port_bytes_written,  gps_port_bytes_read;
 u32 gps_port_write_overruns, gps_port_read_overruns;
 u32 gps_messages_processed;
 u32 gps_messages_delayed;
  
 u16 imu_driver_state;
 u16 imu_port_state;
 u32 imu_port_bytes_written,  imu_port_bytes_read;
 u32 imu_port_write_overruns, imu_port_read_overruns;
 u32 imu_messages_processed;
 u32 imu_messages_delayed;
}gx3_35_device_status_field;


#pragma pack()

#endif