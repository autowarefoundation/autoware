/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_interface.h 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description Implements an interface to a MIP device.  Performs stream parsing and 
//!              and command/reply handling.
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

#ifndef _MIP_SDK_INTERFACE_H
#define _MIP_SDK_INTERFACE_H

   
////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include "mip.h"
#include "mip_sdk_config.h"
#include "mip_sdk_user_functions.h"
#include "ring_buffer.h"
#include "byteswap_utilities.h"

////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
////////////////////////////////////////////////////////////////////////////////

//! @def 

#define MIP_INTERFACE_INPUT_RING_BUFFER_SIZE MIP_MAX_PACKET_SIZE


#define MIP_INTERFACE_CALLBACK_VALID_PACKET   0
#define MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR 1
#define MIP_INTERFACE_CALLBACK_TIMEOUT        2

#define MIP_INTERFACE_OK    0
#define MIP_INTERFACE_ERROR 1

#define MIP_INTERFACE_TIMEOUT    0
#define MIP_INTERFACE_NO_TIMEOUT 1

#define MIP_INTERFACE_INITIALIZED   1
#define MIP_INTERFACE_UNINITIALIZED 0

typedef void (*parser_callback_ptr)(void*, u8*, u16, u8);


////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////

typedef struct _mip_interface
{
 //State
 u8 state;
 
 //Port Handle
 void *port_handle;
   
 //Input Ring Buffer
 ring_buffer input_buffer;
 u8          input_buffer_bytes[MIP_INTERFACE_INPUT_RING_BUFFER_SIZE]; 

 //MIP Packet Parser variables
 u8  mip_packet[MIP_MAX_PACKET_SIZE];
 u16 mip_packet_byte_count; 
 u32 parser_start_time; 
 u32 parser_num_bad_checksums;
 u32 parser_timeouts;
 u8  parser_in_sync;
 u32 parser_headers_skipped;

 u32 packet_timeout;
 
 //Command/Response Variables
 u8  command_response_received;
 u8  command_id;
 u8  command_acknack_response;
 u8 *command_response_data;
 u8  command_response_data_size;
 
 //Callback variables
 u8                  callback_data_set_list[MIP_INTERFACE_MAX_CALLBACKS + 1];
 parser_callback_ptr callback_function_list[MIP_INTERFACE_MAX_CALLBACKS + 1];
 void               *callback_user_ptr_list[MIP_INTERFACE_MAX_CALLBACKS + 1];
 
}mip_interface;   


////////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
////////////////////////////////////////////////////////////////////////////////

u16 mip_interface_init(const char *portstr, u32 baudrate, mip_interface *device_interface, u32 packet_timeout_val);
u16 mip_interface_close(mip_interface *device_interface);

u16 mip_interface_add_descriptor_set_callback(mip_interface *device_interface, u8 data_set, void *user_ptr, void (*packet_callback)(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type));
u16 mip_interface_delete_descriptor_set_callback(mip_interface *device_interface, u8 data_set);

u16 mip_interface_update(mip_interface *device_interface);

u16 mip_interface_send_command(mip_interface *device_interface, u8 command_set, u8 command_descriptor, u8 *command_data, 
                               u16 command_data_size, u8 wait_for_response, u32 timeout_ms);
                               
u16 mip_interface_send_preformatted_command(mip_interface *device_interface, u8 *command, u16 command_size, u8 wait_for_response, u32 timeout_ms);

u16 mip_interface_send_command_with_response(mip_interface *device_interface, u8 command_set, u8 command_descriptor, u8 *command_data, 
                                             u16 command_data_size, u8 **response_data, u16 *response_data_size, u32 timeout_ms);
                                             
u16 mip_interface_send_preformatted_command_with_response(mip_interface *device_interface, u8 *command, u16 command_size, 
                                                          u8 **response_data, u16 *response_data_size, u32 timeout_ms);


u16 mip_interface_write(mip_interface *device_interface, u8 *data, u32 num_bytes, u32 *bytes_written);
u16 mip_interface_write_blocking(mip_interface *device_interface, u8 *data, u32 num_bytes, u32 *bytes_written, u32 timeout_ms);

u16 __mip_interface_parse_input_buffer(mip_interface *device_interface);
u16 __mip_interface_find_callback(mip_interface *device_interface, u8 data_set, void **callback_user_ptr, parser_callback_ptr *callback_function);
u16 __mip_interface_time_timeout(u32 initial_time, u32 timeout_ms);

u16  __mip_interface_wait_for_response(mip_interface *device_interface, u8 command_set, u8 command_descriptor, u8 *acknack_response, u8 **response_data, u16 *response_data_size, u32 timeout_ms);
void __mip_interface_command_response_handler(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);


#endif
