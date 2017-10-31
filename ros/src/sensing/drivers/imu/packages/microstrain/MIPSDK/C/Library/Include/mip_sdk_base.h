/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_base.h 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP BASE Descriptor Set Definitions
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

#ifndef _MIP_BASE_DESC_H
#define _MIP_BASE_DESC_H


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

#define MIP_BASE_COMMAND_DESC_SET						0x01

////////////////////////////////////////////////////////////////////////////////
// BASE COMMAND DESCRIPTORS (command desc are < 0x80)
////////////////////////////////////////////////////////////////////////////////

#define MIP_CMD_DESC_BASE_PING           				0x01	 
#define MIP_CMD_DESC_BASE_SET_TO_IDLE      				0x02	 
#define MIP_CMD_DESC_BASE_GET_DEVICE_INFO           	0x03	 
#define MIP_CMD_DESC_BASE_GET_DEVICE_DESCRIPTORS       	0x04	 
#define MIP_CMD_DESC_BASE_BUILT_IN_TEST              	0x05	
#define MIP_CMD_DESC_BASE_RESUME          				0x06

#define MIP_CMD_DESC_BASE_GPS_TIME_BROADCAST            0x71
#define MIP_CMD_DESC_BASE_FIRMWARE_UPDATE            	0x7D	 
#define MIP_CMD_DESC_BASE_SOFT_RESET                 	0x7E	 
#define MIP_CMD_DESC_BASE_PRODUCTION_TEST            	0x7F	 

////////////////////////////////////////////////////////////////////////////////
// BASE REPLY DESCRIPTORS (reply desc are >= 0x80)
////////////////////////////////////////////////////////////////////////////////

#define MIP_REPLY_DESC_BASE_DEVICE_INFO    				0x81	
#define MIP_REPLY_DESC_BASE_DEVICE_DESCRIPTORS    		0x82	
#define MIP_REPLY_DESC_BASE_BUILT_IN_TEST    			0x83	


////////////////////////////////////////////////////////////////////////////////
// COMMAND SECURITY VALUES
////////////////////////////////////////////////////////////////////////////////

#define MIP_CMD_FIRMWARE_UPDATE_SECURITY_VAL            0x374EA822
#define MIP_CMD_PRODUCTION_TEST_SECURITY_VAL            0xD3C4A133




////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////

#pragma pack(1)

#define BASE_DEVICE_INFO_PARAM_LENGTH 8  // 8 x 16 bits (16 characters)

///
// Device Information
///

typedef struct _base_device_info_field
{
 u16 firmware_version;
 u16 model_name[BASE_DEVICE_INFO_PARAM_LENGTH];
 u16 model_number[BASE_DEVICE_INFO_PARAM_LENGTH];
 u16 serial_number[BASE_DEVICE_INFO_PARAM_LENGTH];
 u16 lotnumber[BASE_DEVICE_INFO_PARAM_LENGTH];
 u16 device_options[BASE_DEVICE_INFO_PARAM_LENGTH];
}base_device_info_field;

#pragma pack()


////////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
////////////////////////////////////////////////////////////////////////////////


u16 mip_base_cmd_ping(mip_interface *device_interface);
u16 mip_base_cmd_idle(mip_interface *device_interface);
u16 mip_base_cmd_get_device_info(mip_interface *device_interface, base_device_info_field *device_info);
u16 mip_base_cmd_get_device_supported_descriptors(mip_interface *device_interface, u8 *response_buffer, u16 *response_size);
u16 mip_base_cmd_built_in_test(mip_interface *device_interface, u32 *bit_result);
u16 mip_base_cmd_resume(mip_interface *device_interface);
u16 mip_base_cmd_reset_device(mip_interface *device_interface);


#endif
