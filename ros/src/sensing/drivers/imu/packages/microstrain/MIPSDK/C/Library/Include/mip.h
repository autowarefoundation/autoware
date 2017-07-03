/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip.h 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP Definition, Packet Parsing, and Construction Functions
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


#ifndef _MIP_H
#define _MIP_H

#ifdef __cplusplus
 extern "C" {
#endif
   
////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include "mip_types.h"


////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// MIP 
////////////////////////////////////////////////////////////////////////////////

//! @def 

//Sync Bytes
#define MIP_SYNC_BYTE1         0x75 
#define MIP_SYNC_BYTE2         0x65

//MIP Packet Sizes
#define MIP_HEADER_SIZE	          sizeof(mip_header)		
#define MIP_CHECKSUM_SIZE         2
#define MIP_MAX_PAYLOAD_SIZE      255
#define MIP_MAX_PAYLOAD_DATA_SIZE 253
#define MIP_FIELD_HEADER_SIZE     sizeof(mip_field_header)
#define MIP_MAX_PACKET_SIZE       (MIP_HEADER_SIZE + MIP_MAX_PAYLOAD_SIZE + MIP_CHECKSUM_SIZE)


////////////////////////////////////////////////////////////////////////////////
// GLOBAL MIP DESCRIPTORS  (global desc are 0xF0 - 0xFF) - same usage in all descriptor sets
////////////////////////////////////////////////////////////////////////////////

#define MIP_REPLY_DESC_GLOBAL_ACK_NACK    				0xF1 // Ack/nack desc is same in all sets
#define MIP_DESC_GLOBAL_PRODUCTION_TEST    				0xFE // Production Test desc is same in all sets


////////////////////////////////////////////////////////////////////////////////
// GLOBAL ACK/NACK ERROR CODES
////////////////////////////////////////////////////////////////////////////////

#define MIP_ACK_NACK_ERROR_NONE                0x00

#define MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND        0x01
#define MIP_ACK_NACK_ERROR_CHECKSUM_INVALID       0x02
#define MIP_ACK_NACK_ERROR_PARAMETER_INVALID      0x03
#define MIP_ACK_NACK_ERROR_COMMAND_FAILED         0x04
#define MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT        0x05
#define MIP_ACK_NACK_ERROR_UNKNOWN_DESCRIPTOR_SET 0x06



///
//Function Return Codes
///

#define MIP_OK                   0
#define MIP_ERROR                1
#define MIP_MEMORY_ERROR         2
#define MIP_FIELD_NOT_AVAILABLE  3
#define MIP_INVALID_PACKET       4
#define MIP_CHECKSUM_ERROR       5


//Function Selector Byte for Settings 

#define MIP_FUNCTION_SELECTOR_WRITE        0x01
#define MIP_FUNCTION_SELECTOR_READ         0x02
#define MIP_FUNCTION_SELECTOR_STORE_EEPROM 0x03
#define MIP_FUNCTION_SELECTOR_LOAD_EEPROM  0x04
#define MIP_FUNCTION_SELECTOR_LOAD_DEFAULT 0x05

//AHRS Supported Descriptors Macro
#define IS_MIP_FUNCTION_SELECTOR(SELECTOR) (((SELECTOR) == MIP_FUNCTION_SELECTOR_WRITE)        || \
                                            ((SELECTOR) == MIP_FUNCTION_SELECTOR_READ)         || \
                                            ((SELECTOR) == MIP_FUNCTION_SELECTOR_STORE_EEPROM) || \
                                            ((SELECTOR) == MIP_FUNCTION_SELECTOR_LOAD_EEPROM)  || \
                                            ((SELECTOR) == MIP_FUNCTION_SELECTOR_LOAD_DEFAULT))



////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////

#pragma pack(1)

///
//MIP Packet Header 
///

typedef struct _mip_header
{
 u8 sync1, sync2;
 u8 descriptor_set, payload_size; 
}mip_header;

///
//MIP Field Header
///

typedef struct _mip_field_header
{
 u8 size;
 u8 descriptor;
}mip_field_header;


///
//MIP ACK/NACK Response
///

typedef struct _global_ack_nack_field
{
 u8 command_echo_byte;
 u8 error_code;
}global_ack_nack_field;


#pragma pack()



////////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
////////////////////////////////////////////////////////////////////////////////

u16 mip_init(u8 *mip_buffer, u16 buffer_size, u8 descriptor_set);
u16 mip_add_field(u8 *mip_buffer, u16 buffer_size, void *field_data, u16 data_size, u16 data_descriptor);
u16 mip_add_formatted_field(u8 *mip_buffer, u16 buffer_size, void *field);
u16 mip_finalize(u8 *mip_buffer);

u16 mip_is_initialized(u8 *mip_buffer, u8 descriptor_set);
u16 mip_is_mip_packet (u8 *mip_buffer);

u8  mip_get_packet_descriptor_set(u8 *mip_buffer);
u8  mip_get_payload_size(u8 *mip_buffer);
u8  *mip_get_payload_ptr(u8 *mip_buffer);

u16 mip_get_packet_size(u8 *mip_buffer);

u16 mip_get_first_field(u8 *mip_buffer, mip_field_header **field_header, u8 **field_data, u16 *field_offset);
u16 mip_get_next_field(u8 *mip_buffer, mip_field_header **field_header, u8 **field_data, u16 *field_offset);

u16 mip_calculate_checksum(u8 *mip_buffer);
u16 mip_is_checksum_valid(u8 *mip_buffer);

#ifdef __cplusplus
}
#endif

#endif
