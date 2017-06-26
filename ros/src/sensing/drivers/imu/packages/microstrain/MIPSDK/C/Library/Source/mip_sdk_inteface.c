/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_interface.c
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


////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include "mip_sdk_interface.h"



/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_interface_init(u32 com_port, u32 baudrate, 
//!                        mip_interface *device_interface, u32 packet_timeout_val)
//
//! @section DESCRIPTION
//! MIP Interface Initialization function.
//
//! @section DETAILS
//!
//! @param [in] u32 com_port                    - The port to interface to.
//! @param [in] u32 baudrate                    - The baudrate of the port.
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u32 packet_timeout_val          - Timeout for the incoming packet in milliseconds.
//
//! @retval MIP_INTERFACE_ERROR  The interface was not initialized.\n
//! @retval MIP_INTERFACE_OK     The interface was successfully initialized.\n
//
//! @section NOTES
//! 
//! None
//!
/////////////////////////////////////////////////////////////////////////////

// BSB - changed way we describe serial port
u16 mip_interface_init(const char *portstr, u32 baudrate, mip_interface *device_interface, u32 packet_timeout_val)
{
 u16 i;
  
 device_interface->port_handle = NULL; 
  
 //Attempt to open the port
 if(mip_sdk_port_open(&device_interface->port_handle, portstr, baudrate) != MIP_USER_FUNCTION_OK)
  return MIP_INTERFACE_ERROR; 
  
  
 //Initialize the command parser ring buffer
 if(ring_buffer_init_static(&device_interface->input_buffer, device_interface->input_buffer_bytes, MIP_INTERFACE_INPUT_RING_BUFFER_SIZE, 1) != RING_BUFFER_OK)
   return MIP_INTERFACE_ERROR; 
 
 //MIP Packet Parser variables init
 memset(device_interface->mip_packet, 0, MIP_MAX_PACKET_SIZE);
 device_interface->mip_packet_byte_count    = 0; 
 device_interface->parser_start_time        = 0;  
 device_interface->parser_num_bad_checksums = 0;
 device_interface->parser_timeouts          = 0;
 device_interface->parser_in_sync           = 0;
 device_interface->parser_headers_skipped   = 0;
 device_interface->packet_timeout           = packet_timeout_val;  
 
 //Initialize the callback info
 for(i=0; i<MIP_INTERFACE_MAX_CALLBACKS + 1; i++)
 {
  device_interface->callback_data_set_list[i] = 0;
  device_interface->callback_function_list[i] = NULL;
  device_interface->callback_user_ptr_list[i] = NULL;
 }
 
 //Command/Response Variables
 device_interface->command_response_received  = 0;
 device_interface->command_acknack_response   = 0;
 device_interface->command_response_data      = 0;
 device_interface->command_response_data_size = 0;
 device_interface->command_id                 = 0;
 
 //Callback 0 is always the command response handler & needs a pointer to the interface
 device_interface->callback_function_list[0] = &__mip_interface_command_response_handler;
 device_interface->callback_user_ptr_list[0] = (void*)device_interface;
 
 //Set the MIP Parser state
 device_interface->state = MIP_INTERFACE_INITIALIZED;

 return MIP_INTERFACE_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_interface_close(mip_interface *device_interface)
//
//! @section DESCRIPTION
//! MIP Interface Close function.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//
//! @retval MIP_INTERFACE_ERROR  The interface was not closed.\n
//! @retval MIP_INTERFACE_OK     The interface was successfully closed.\n
//
//! @section NOTES
//! 
//! None
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_interface_close(mip_interface *device_interface)
{
  
 //Attempt to close the port
 if(mip_sdk_port_close(device_interface->port_handle) != MIP_USER_FUNCTION_OK)
  return MIP_INTERFACE_ERROR; 
  
 return MIP_INTERFACE_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_interface_update(mip_interface *device_interface)
//
//! @section DESCRIPTION
//! MIP Interface Update function.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//
//! @retval MIP_INTERFACE_ERROR  The interface was not initialized.\n
//! @retval MIP_INTERFACE_OK     The update step completed.\n
//
//! @section NOTES
//! 
//! This function should be called regularly (e.g. every step of a minor cycle.)\n
//! This is the main loop of the interface.
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_interface_update(mip_interface *device_interface)
{
 u32 num_bytes, bytes_read = 0, bytes_written = 0;
 u8  local_buffer[MIP_INTERFACE_INPUT_RING_BUFFER_SIZE];
 u32 port_bytes;
 
 //The parser must be initialized
 if(device_interface->state != MIP_INTERFACE_INITIALIZED)
  return MIP_INTERFACE_ERROR;
 
 //Determine the max number of bytes to read
 num_bytes = MIP_INTERFACE_INPUT_RING_BUFFER_SIZE;
 
 if(ring_buffer_remaining_entries(&device_interface->input_buffer) < num_bytes)
  num_bytes = ring_buffer_remaining_entries(&device_interface->input_buffer);
 
 port_bytes = mip_sdk_port_read_count(device_interface->port_handle);
 
 if(num_bytes > port_bytes)
  num_bytes = port_bytes;
 
 
 //Read up to max ring buffer size from the port
 if(num_bytes > 0)
  mip_sdk_port_read(device_interface->port_handle, local_buffer, num_bytes, &bytes_read, MIP_INTERFACE_PORT_READ_TIMEOUT_MS);

 //Write the local buffer to the ring buffer
 if(bytes_read > 0)
 { 
  //printf("Got %d bytes\n", bytes_read);
  ring_buffer_write_multi(&device_interface->input_buffer, local_buffer, bytes_read, &bytes_written);
 }
 
 //Parse the data
 __mip_interface_parse_input_buffer(device_interface);
 
 
 return MIP_INTERFACE_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_interface_add_descriptor_set_callback(mip_interface *device_interface, u8 data_set, void *user_ptr, 
//!                      void (*packet_callback)(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type))
//
//! @section DESCRIPTION
//! Add a callback for the provided descriptor set.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - pointer to the mip interface structure.
//! @param [in] u8 data_set                     - data set used to trigger the callback.
//! @param [in] void *user_ptr                  - pointer to the user data, which is passed to the callback.
//! @param [in] void (*packet_callback)         - function pointer for packet callback function.
//
//! @retval MIP_INTERFACE_ERROR  The callback could not be added.\n
//! @retval MIP_INTERFACE_OK     The callback was added successfully.\n
//
//! @section NOTES
//! 
//! None.
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_interface_add_descriptor_set_callback(mip_interface *device_interface, u8 data_set, void *user_ptr, void (*packet_callback)(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type))
{
 u16 i;
 
 //Loop through all of the entries and locate the first empty one 
 //(note: callback 0 is always reserved for commands/responses)
 for(i=1; i<MIP_INTERFACE_MAX_CALLBACKS + 1; i++)
 {
  if((device_interface->callback_data_set_list[i] == 0)    && 
     (device_interface->callback_function_list[i] == NULL) && 
     (device_interface->callback_user_ptr_list[i] == NULL))
  {
   device_interface->callback_data_set_list[i] = data_set;
   device_interface->callback_function_list[i] = packet_callback;
   device_interface->callback_user_ptr_list[i] = user_ptr;
   return MIP_INTERFACE_OK;
  }
 }
 
 //The list is full
 return MIP_INTERFACE_ERROR;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_interface_delete_descriptor_set_callback(mip_interface *device_interface, u8 data_set)
//
//! @section DESCRIPTION
//! Delete the callback for the provided descriptor set if it exists.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - pointer to the mip interface structure.
//! @param [in] u8 data_set                     - data set to remove callbacks.
//
//! @retval MIP_INTERFACE_ERROR  The callback could not be removed.\n
//! @retval MIP_INTERFACE_OK     The callback was removed successfully.\n
//
//! @section NOTES
//! 
//! None.
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_interface_delete_descriptor_set_callback(mip_interface *device_interface, u8 data_set)
{
 u16 i, found = 0;
 
 //Loop through all of the entries and remove any that reference the descriptor set
 //(note: callback 0 is always reserved for commands/responses)
 for(i=1; i<MIP_INTERFACE_MAX_CALLBACKS + 1; i++)
 {
  if(device_interface->callback_data_set_list[i] == data_set)
  {
   device_interface->callback_data_set_list[i]   = 0;
   device_interface->callback_function_list[i]   = NULL;
   device_interface->callback_user_ptr_list[i]   = NULL;
   found = 1;
  }
 }
 
 //Return an error if the descriptor set was not found
 if(!found)
  return MIP_INTERFACE_ERROR;
 else
  return MIP_INTERFACE_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_interface_write(mip_interface *device_interface, u8 *data, u32 num_bytes, u32 *bytes_written)
//
//! @section DESCRIPTION
//! Writes \c num_bytes of \c data to the device interface.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - pointer to the mip interface structure.
//! @param [in] u8 *data                        - pointer to data buffer to be written.
//! @param [in] u32 num_bytes                   - the number of bytes to be written.
//
//! @retval MIP_INTERFACE_ERROR  Data not written or less than num_bytes written.\n
//! @retval MIP_INTERFACE_OK     Data written.\n
//
//! @section NOTES
//! 
//! This function is used to write bytes from a device to the interface,\n
//! which will then be parsed.
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_interface_write(mip_interface *device_interface, u8 *data, u32 num_bytes, u32 *bytes_written)
{
 u32 i;
 
 //Check that the parser is initialized
 if(device_interface->state != MIP_INTERFACE_INITIALIZED)
   return MIP_INTERFACE_ERROR;

 //Set the feedback of bytes written to zero
 *bytes_written = 0;

 //Zero byte write short circuit
 if(num_bytes == 0)
   return MIP_INTERFACE_OK;
 
 //Loop through the bytes and copy them to the ring buffer
 for(i=0; i<num_bytes; i++)
 {
  if(ring_buffer_write(&device_interface->input_buffer, &data[i], 1) == RING_BUFFER_OK)
   (*bytes_written)++; 
  else
   break;
 }  
 
 if(num_bytes != *bytes_written)
   return MIP_INTERFACE_ERROR;
 
 return MIP_INTERFACE_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_interface_write_blocking(mip_interface *device_interface, u8 *data, u32 num_bytes, u32 *bytes_written, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Writes \c num_bytes of \c data to the device interface, blocks until the space is available.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - pointer to the mip interface structure.
//! @param [in] u8 *data                        - pointer to data buffer to be written.
//! @param [in] u32 num_bytes                   - the number of bytes to be written.
//! @param [in] u32 timeout_ms                  - the timeout for the write in milliseconds.
//
//! @retval MIP_INTERFACE_ERROR  Data not written or less than num_bytes written.\n
//! @retval MIP_INTERFACE_OK     Data written.\n
//
//! @section NOTES
//! 
//! This function is used to write bytes from a source into the interface,\n
//! which will then be parsed.
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_interface_write_blocking(mip_interface *device_interface, u8 *data, u32 num_bytes, u32 *bytes_written, u32 timeout_ms)
{
 u32 initial_time = mip_sdk_get_time_ms();
 
 //Check that the parser is initialized
 if(device_interface->state != MIP_INTERFACE_INITIALIZED)
   return MIP_INTERFACE_ERROR;

 //Cannot write more bytes than the buffer size
 if(num_bytes > MIP_INTERFACE_INPUT_RING_BUFFER_SIZE)
   return MIP_INTERFACE_ERROR;
 
 //Spin until the buffer has enough room 
 while(num_bytes > ring_buffer_remaining_entries(&device_interface->input_buffer))
 {
  if(__mip_interface_time_timeout(initial_time, timeout_ms) == MIP_INTERFACE_TIMEOUT)
    return MIP_INTERFACE_ERROR;
 }  
  
 return mip_interface_write(device_interface, data, num_bytes, bytes_written);  
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 __mip_interface_parse_input_buffer(mip_interface *device_interface)
//
//! @section DESCRIPTION
//! Processes the bytes in the input buffer and triggers the callback function.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - pointer to the mip interface structure.
//
//! @retval MIP_INTERFACE_ERROR  Interface not initialized.\n
//! @retval MIP_INTERFACE_OK     Parser ran.\n
//
//! @section NOTES
//! 
//! This is an internal function.
//!
/////////////////////////////////////////////////////////////////////////////

u16 __mip_interface_parse_input_buffer(mip_interface *device_interface)
{
 u16 i, ret;
 mip_header          *header_ptr = (mip_header*)device_interface->mip_packet;
 parser_callback_ptr  callback_function = NULL;
 void                *callback_user_ptr = NULL;
 
 //Check that the parser is initialized
 if(device_interface->state != MIP_INTERFACE_INITIALIZED)
   return MIP_INTERFACE_ERROR;
 
  
 ///
 //Are we searching for a start condition? Requires: SYNC byte 1 & 2, and a valid header
 ///
 
 if(device_interface->mip_packet_byte_count < sizeof(mip_header)) 
 {
   
  ///
  //Get a start byte
  ///
   
  while((device_interface->mip_packet_byte_count == 0) && ring_buffer_count(&device_interface->input_buffer))
  {
    ret = ring_buffer_read(&device_interface->input_buffer, &device_interface->mip_packet[0], 1);
    
    //Found a potential start byte
    if((ret == RING_BUFFER_OK) && (device_interface->mip_packet[0] == MIP_SYNC_BYTE1))
    {
     device_interface->mip_packet_byte_count = 1;
     device_interface->parser_start_time     = mip_sdk_get_time_ms();
    }
  }
  
  ///
  //Get the rest of the header
  ///
  
  if(device_interface->mip_packet_byte_count > 0)
  {
   //Need at least the size of (header - start_byte) in the buffer to start processing
   if(ring_buffer_count(&device_interface->input_buffer) >= sizeof(mip_header) - 1)
   {
    //Try to get a header
    for(i=0; i < sizeof(mip_header) - 1; i++)
    {
     ring_buffer_lookahead_read(&device_interface->input_buffer, i, &device_interface->mip_packet[i+1], 1);
    }
     
    //If the header is valid, then continue to the next step
    if((header_ptr->sync2 == MIP_SYNC_BYTE2) && 
       (header_ptr->payload_size + MIP_HEADER_SIZE + MIP_CHECKSUM_SIZE <= MIP_MAX_PACKET_SIZE))
    {
     device_interface->mip_packet_byte_count = sizeof(mip_header);
    }
    else
    {
     //Header not found
     device_interface->parser_in_sync        = 0;
     device_interface->mip_packet_byte_count = 0;
     device_interface->parser_headers_skipped++;
    }
   }
   //Check for timeout on incoming packet
   else
   {
    //If a timeout happens, we probably had a badly-formed packet, start over
    if(__mip_interface_time_timeout(device_interface->parser_start_time, device_interface->packet_timeout) == MIP_INTERFACE_TIMEOUT)
    {
     device_interface->parser_timeouts++;
     
     //Reset the parser  
     device_interface->mip_packet_byte_count = 0;
     device_interface->parser_in_sync        = 0;
    }
   }
  }
 }

 
 ///
 //Header located, get the rest of the potential MIP packet
 ///
 
 if(device_interface->mip_packet_byte_count >= sizeof(mip_header))
 {
  //Wait for the rest of the packet to be available in the buffer
  if(ring_buffer_count(&device_interface->input_buffer) >= (sizeof(mip_header) - 1 + header_ptr->payload_size + MIP_CHECKSUM_SIZE))
  {
   //Get the remaining packet bytes
   for(i=0; i < header_ptr->payload_size + MIP_CHECKSUM_SIZE; i++)
   {
    ring_buffer_lookahead_read(&device_interface->input_buffer, sizeof(mip_header) - 1 + i, &device_interface->mip_packet[MIP_HEADER_SIZE + i], 1);
   }
     
   ///
   //Valid MIP Packet Found
   ///
   
   if(mip_is_checksum_valid(device_interface->mip_packet) == MIP_OK)
   {
    device_interface->mip_packet_byte_count += header_ptr->payload_size + MIP_CHECKSUM_SIZE;
    
    //Trigger the callback with the valid packet
    if(__mip_interface_find_callback(device_interface, header_ptr->descriptor_set, &callback_user_ptr, &callback_function) == MIP_INTERFACE_OK)
    {
     (*callback_function)(callback_user_ptr, device_interface->mip_packet, device_interface->mip_packet_byte_count, MIP_INTERFACE_CALLBACK_VALID_PACKET);
    }
   
    ring_buffer_consume_entries(&device_interface->input_buffer, device_interface->mip_packet_byte_count - 1);
     
    device_interface->parser_in_sync = 1;
 
    //Reset the parser  
    device_interface->mip_packet_byte_count = 0;
   } 

   ///
   //Inalid MIP Packet: Bad checksum
   ///
   
   else
   {
    //Trigger the callback with a bad checksum
    if(__mip_interface_find_callback(device_interface, header_ptr->descriptor_set, &callback_user_ptr, &callback_function) == MIP_INTERFACE_OK)
    {
     (*callback_function)(callback_user_ptr, device_interface->mip_packet, device_interface->mip_packet_byte_count, MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR);
    }
    
    if(device_interface->parser_in_sync)
    {
     device_interface->parser_num_bad_checksums++;   
    }
    
    device_interface->parser_in_sync = 0;     
   }
   
   //Reset the parser  
   device_interface->mip_packet_byte_count = 0;
  }
  //Check for timeout on incoming packet
  else
  {
   //If a timeout happens, we probably had a bad-form packet, start over
   if(__mip_interface_time_timeout(device_interface->parser_start_time, device_interface->packet_timeout) == MIP_INTERFACE_TIMEOUT)
   {
    //Trigger the callback with a timeout
    if(__mip_interface_find_callback(device_interface, header_ptr->descriptor_set, &callback_user_ptr, &callback_function) == MIP_INTERFACE_OK)
    {
     (*callback_function)(callback_user_ptr, device_interface->mip_packet, device_interface->mip_packet_byte_count, MIP_INTERFACE_CALLBACK_TIMEOUT);
    }

    device_interface->parser_timeouts++;
    device_interface->parser_in_sync = 0;

    //Reset the parser  
    device_interface->mip_packet_byte_count = 0;    
   }
  }
 }  
 
 return MIP_INTERFACE_OK;
}



/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 __mip_interface_find_callback(mip_interface *device_interface, u8 data_set, 
//                                   void *callback_user_ptr, 
//                                   parser_callback_ptr callback_function)
//
//! @section DESCRIPTION
//! Find the callback info for the provided dataset.
//
//! @section DETAILS
//!
//! @param [in]  mip_interface *device_interface       - pointer to the mip interface structure.
//! @param [in]  u8 data_set                           - dataset to search for.
//! @param [out] void *callback_user_ptr               - pointer to the user data for the callback.
//! @param [out] parser_callback_ptr callback_function - pointer to callback function.
//
//! @retval MIP_INTERFACE_ERROR  The callback could not be located or there was an error.\n
//! @retval MIP_INTERFACE_OK     The callback was located.\n
//
//! @section NOTES
//! 
//! This is an internal function.
//!
/////////////////////////////////////////////////////////////////////////////

u16 __mip_interface_find_callback(mip_interface *device_interface, u8 data_set, void **callback_user_ptr, parser_callback_ptr *callback_function)
{
 u16 i;
 
 //Initialize callback variables
 *callback_user_ptr  = NULL;
 
 //Dataset = 0 may occur if there is a problem with the parsed packet
 if(data_set == 0)
  return MIP_INTERFACE_ERROR;
 
 
 //Loop through the callback information, looking for a match to the data set
 for(i=0; i<MIP_INTERFACE_MAX_CALLBACKS + 1; i++)
 {
  if((device_interface->callback_data_set_list[i] == data_set) &&
     (device_interface->callback_function_list[i] != NULL))
  {
   *callback_user_ptr = device_interface->callback_user_ptr_list[i];
   *callback_function = device_interface->callback_function_list[i];
   
   return MIP_INTERFACE_OK;
  }
 }

 return MIP_INTERFACE_ERROR;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 __mip_interface_time_timeout(u32 initial_time, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Check for a timeout condition.
//
//! @section DETAILS
//!
//! @param [in]  u32 initial_time  - the start time to check against in milliseconds.
//! @param [in]  u32 timeout_ms    - the timeout period in milliseconds.
//
//! @retval MIP_INTERFACE_TIMEOUT    The timeout has occured.\n
//! @retval MIP_INTERFACE_NO_TIMEOUT No timeout.\n
//
//! @section NOTES
//! 
//! This is an internal function.
//!
/////////////////////////////////////////////////////////////////////////////

u16 __mip_interface_time_timeout(u32 initial_time, u32 timeout_ms)
{
 u32 current_time = mip_sdk_get_time_ms();
 
 //Handle wrap-around case
 if(initial_time > current_time)
 {
  if(((0xFFFFFFFF - initial_time) + current_time) >= timeout_ms)
    return MIP_INTERFACE_TIMEOUT;
 }
 else
 {
  if(current_time - initial_time >= timeout_ms)
    return MIP_INTERFACE_TIMEOUT;
 }
 
 //No timeout
 return MIP_INTERFACE_NO_TIMEOUT; 
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void __mip_interface_command_response_handler(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
//
//! @section DESCRIPTION
//! Command-Response Handler.
//
//! @section DETAILS
//!
//! @param [in]  void *user_ptr   - Pointer to the user data for this callback.
//! @param [in]  u8 *packet       - The MIP packet that matches the waiting command set.
//! @param [in]  u16 packet_size  - Size of the MIP packet.
//! @param [in]  u8 callback_type - Type of callback.
//
//! @section NOTES
//! 
//! This is an internal function.
//!
/////////////////////////////////////////////////////////////////////////////

void __mip_interface_command_response_handler(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
 u16                    payload_size;
 global_ack_nack_field *ack_nack_field_ptr;
 mip_field_header      *field_header_ptr;
 u8                    *field_data_ptr;
 u16                    field_offset     = 0;
 mip_interface         *device_interface = (mip_interface *)user_ptr;
 
 if(device_interface == NULL)
  return;
 
 //Flag that the response was received
 device_interface->command_response_received = 1;
 
 ///
 //Successful Packet
 ///
  
 if(callback_type == MIP_INTERFACE_CALLBACK_VALID_PACKET)
 {
  payload_size = mip_get_payload_size(packet);
  
  //Get the ACK/NACK Field
  if(mip_get_first_field(packet, &field_header_ptr, &field_data_ptr, &field_offset) == MIP_OK)
  {
   ack_nack_field_ptr                         = (global_ack_nack_field *)field_data_ptr;
   device_interface->command_id               = ack_nack_field_ptr->command_echo_byte;
   device_interface->command_acknack_response = ack_nack_field_ptr->error_code;
  }
  
  //Get the remaining data & subtract the size of the first field
  if(mip_get_next_field(packet, &field_header_ptr, &field_data_ptr, &field_offset) == MIP_OK)
  { 
   device_interface->command_response_data      = field_header_ptr;
   device_interface->command_response_data_size = payload_size - (sizeof(mip_field_header) + sizeof(global_ack_nack_field));
  }
 }
 
 ///
 // Error
 ///
 
 else
 {
  device_interface->command_id                 = 0; 
  device_interface->command_acknack_response   = 0;
  device_interface->command_response_data      = 0;
  device_interface->command_response_data_size = 0;
 }
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_interface_send_command(mip_interface *device_interface, u8 command_set, u8 command_descriptor, 
//!                                u8 *command_data, u16 command_data_size, u8 wait_for_response, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Send a MIP command and optionally wait for the response data.
//
//! @section DETAILS
//!
//! @param [in]  mip_interface *device_interface - a pointer to the mip_interface structure.
//! @param [in]  u8 command_set                  - command set.
//! @param [in]  u8 command_descriptor           - command descriptor.
//! @param [in]  u8 *command_data                - preformatted command data.
//! @param [in]  u16 command_data_size           - size of command data.
//! @param [in]  u8 wait_for_response            - 1: The function will wait for the response, 0: it will not wait.
//! @param [in]  u32 timeout_ms                  - the timeout in milliseconds.
//
//! @retval MIP_INTERFACE_OK    If a response was received and ACK'd or no resonse requested.\n
//! @retval MIP_INTERFACE_ERROR If the response was not received in the timeout period or NACK'd.\n
//
//! @section NOTES
//! 
//! None.
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_interface_send_command(mip_interface *device_interface, u8 command_set, u8 command_descriptor, u8 *command_data, u16 command_data_size, u8 wait_for_response, u32 timeout_ms)
{
 u8  mip_packet[MIP_MAX_PACKET_SIZE];
 u16 packet_size;
 
 //Create the MIP Packet
 mip_init(mip_packet, MIP_MAX_PACKET_SIZE, command_set);
 mip_add_field(mip_packet, MIP_MAX_PACKET_SIZE, command_data, command_data_size, command_descriptor);
 packet_size = mip_finalize(mip_packet);

 //Send the packet to the device
 return mip_interface_send_preformatted_command(device_interface, mip_packet, packet_size, wait_for_response, timeout_ms);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_interface_send_preformatted_command(mip_interface *device_interface, u8 *command, 
//!                                             u16 command_size, u8 wait_for_response, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Send a pre-formatted command and wait for the response data.
//
//! @section DETAILS
//!
//! @param [in]  mip_interface *device_interface - a pointer to the mip_interface structure.
//! @param [in]  u8 *command                     - preformatted MIP command.
//! @param [in]  u16 command_size                - size of MIP command.
//! @param [in]  u8 wait_for_response            - 1: The function will wait for the response, 0: it will not wait.
//! @param [in]  u32 timeout_ms                  - the timeout in milliseconds.
//
//! @retval MIP_INTERFACE_OK    If a response was received and ACK'd or no resonse requested.\n
//! @retval MIP_INTERFACE_ERROR If the response was not received in the timeout period or NACK'd.\n
//
//! @section NOTES
//! 
//! None.
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_interface_send_preformatted_command(mip_interface *device_interface, u8 *command, u16 command_size, u8 wait_for_response, u32 timeout_ms)
{
 u8                acknack_response = MIP_ACK_NACK_ERROR_COMMAND_FAILED;
 u16               return_code;  
 u32               bytes_written;
 u8                command_set;
 u8                command_descriptor;
 mip_header       *header_ptr = (mip_header *)command;
 mip_field_header *field_header_ptr;
 u8               *field_data_ptr;
 u16               field_offset = 0;
 u8               *response_data;
 u16               response_data_size;
 
 //Null-check inputs
 if((device_interface == NULL) ||(command == NULL) || (command_size == 0))
  return MIP_INTERFACE_ERROR; 
 
 //Send the packet
 if(mip_sdk_port_write(device_interface->port_handle, command, command_size, &bytes_written, timeout_ms) != MIP_INTERFACE_OK)
  return MIP_INTERFACE_ERROR;

 //Return if we are not waiting for a response
 if(!wait_for_response)
  return MIP_INTERFACE_OK;

     
 //Set the command set and descriptor information
 command_set        = header_ptr->descriptor_set;
 
 if(mip_get_first_field(command, &field_header_ptr, &field_data_ptr, &field_offset) != MIP_OK)
 {
  return MIP_INTERFACE_ERROR;
 }
 
 command_descriptor = field_header_ptr->descriptor;
                                             
 //Wait for the response from the device
 return_code = __mip_interface_wait_for_response(device_interface, command_set, command_descriptor, 
                                                 &acknack_response, &response_data, &response_data_size, timeout_ms);
                                                 
 if((return_code != MIP_INTERFACE_OK) || (acknack_response != MIP_ACK_NACK_ERROR_NONE))
  return MIP_INTERFACE_ERROR;
  
  
 return MIP_INTERFACE_OK;
}




/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_interface_send_command_with_response(mip_interface *device_interface, u8 command_set, u8 command_descriptor, u8 *command_data, 
//!                                              u16 command_data_size, u8 **response_data, u16 *response_data_size, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Send a MIP command and wait for the response data.
//
//! @section DETAILS
//!
//! @param [in]  mip_interface *device_interface - a pointer to the mip_interface structure.
//! @param [in]  u8 command_set                  - command set.
//! @param [in]  u8 command_descriptor           - command descriptor.
//! @param [in]  u8 *command_data                - preformatted command data.
//! @param [in]  u16 command_data_size           - size of command data.
//! @param [out] u8 **response_data              - pointer to a pointer that will point to the beginning of the response.
//! @param [out] u16 *response_data_size         - size of the response data.
//! @param [in]  u32 timeout_ms                  - the timeout in milliseconds.
//
//! @retval MIP_INTERFACE_OK    If a response was received and ACK'd.\n
//! @retval MIP_INTERFACE_ERROR If the response was not received in the timeout period or NACK'd.\n
//
//! @section NOTES
//! 
//! \c response_data will point to an internal buffer within the MIP interface.\n
//! The user should copy the information to their own buffer before manipulation.
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_interface_send_command_with_response(mip_interface *device_interface, u8 command_set, u8 command_descriptor, u8 *command_data, 
                                             u16 command_data_size, u8 **response_data, u16 *response_data_size, u32 timeout_ms)
{
 u8  mip_packet[MIP_MAX_PACKET_SIZE];
 u16 packet_size;
 
 //Create the MIP Packet
 mip_init(mip_packet, MIP_MAX_PACKET_SIZE, command_set);
 mip_add_field(mip_packet, MIP_MAX_PACKET_SIZE, command_data, command_data_size, command_descriptor);
 packet_size = mip_finalize(mip_packet);

 //Send the packet to the device
 return mip_interface_send_preformatted_command_with_response(device_interface, mip_packet, packet_size, 
                                                              response_data, response_data_size, timeout_ms);
}



/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_interface_send_preformatted_command_with_response(mip_interface *device_interface, u8 *command, u16 command_size, 
//!                                                           u8 *response_data, u16 **response_data_size, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Send a pre-formatted command and wait for the response data.
//
//! @section DETAILS
//!
//! @param [in]  mip_interface *device_interface - a pointer to the mip_interface structure.
//! @param [in]  u8 *command                     - preformatted MIP command.
//! @param [in]  u16 command_size                - size of MIP command.
//! @param [out] u8 **response_data              - pointer to a pointer that will point to the beginning of the response.
//! @param [out] u16 *response_data_size         - size of the response data.
//! @param [in]  u32 timeout_ms                  - the timeout in milliseconds.
//
//! @retval MIP_INTERFACE_OK    If a response was received and ACK'd.\n
//! @retval MIP_INTERFACE_ERROR If the response was not received in the timeout period or NACK'd.\n
//
//! @section NOTES
//! 
//! \c response_data will point to an internal buffer within the MIP interface.\n
//! The user should copy the information to their own buffer before manipulation.
//!
/////////////////////////////////////////////////////////////////////////////


u16 mip_interface_send_preformatted_command_with_response(mip_interface *device_interface, u8 *command, u16 command_size, 
                                                          u8 **response_data, u16 *response_data_size, u32 timeout_ms)
{
 u8                acknack_response = MIP_ACK_NACK_ERROR_COMMAND_FAILED;
 u16               return_code;  
 u32               bytes_written;
 u8                command_set;
 u8                command_descriptor;
 mip_header       *header_ptr = (mip_header *)command;
 mip_field_header *field_header_ptr;
 u8               *field_data_ptr;
 u16               field_offset = 0;
 
 //Null-check inputs
 if((device_interface == NULL) ||(command == NULL) || (command_size == 0))
  return MIP_INTERFACE_ERROR;
  
 
 //Send the packet
 if(mip_sdk_port_write(device_interface->port_handle, command, command_size, &bytes_written, timeout_ms) != MIP_INTERFACE_OK)
  return MIP_INTERFACE_ERROR;
   
 
 //Set the command set and descriptor information
 command_set        = header_ptr->descriptor_set;
 
 if(mip_get_first_field(command, &field_header_ptr, &field_data_ptr, &field_offset) != MIP_OK)
 {
  return MIP_INTERFACE_ERROR;
 }
 
 command_descriptor = field_header_ptr->descriptor;

                                             
 //Wait for the response from the device
 return_code = __mip_interface_wait_for_response(device_interface, command_set, command_descriptor, 
                                                 &acknack_response, response_data, response_data_size, timeout_ms);
                                                 
 if((return_code != MIP_INTERFACE_OK) || (acknack_response != MIP_ACK_NACK_ERROR_NONE))
  return MIP_INTERFACE_ERROR;
  
  
 return MIP_INTERFACE_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16  __mip_interface_wait_for_response(mip_interface *device_interface, u8 command_set, u8 command_descriptor, 
//!                                        u8 *acknack_response, u8 **response_data, u16 *response_data_size, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Wait for a Command-Response.
//
//! @section DETAILS
//!
//! @param [in]  mip_interface *device_interface - a pointer to the mip_interface structure.
//! @param [in]  u8 command_set                  - the command set we are waiting for.
//! @param [in]  u8 command_descriptor           - the command descriptor we are waiting for.
//! @param [out] u8  *acknack_response           - the resulting ack/nack response from the device.
//! @param [out] u8 **response_data              - a pointer to a pointer to the response data, NULL if no data field exists.
//! @param [out] u16 *response_data_size         - size of the response data, 0 if no data field exists.
//! @param [in]  u32 timeout_ms                  - the timeout in milliseconds.
//
//! @retval MIP_INTERFACE_OK    If a response was received.\n
//! @retval MIP_INTERFACE_ERROR If the response was not received in the timeout period.\n
//
//! @section NOTES
//! 
//! This is an internal function.
//!
/////////////////////////////////////////////////////////////////////////////

u16  __mip_interface_wait_for_response(mip_interface *device_interface, u8 command_set, u8 command_descriptor, 
                                       u8 *acknack_response, u8 **response_data, u16 *response_data_size, u32 timeout_ms)
{
 u32 start_time  = mip_sdk_get_time_ms();
 u16 return_code = MIP_INTERFACE_ERROR;
 
 //Setup response callback variables
 device_interface->callback_data_set_list[0]  = command_set;
 device_interface->command_id                 = 0; 
 device_interface->command_acknack_response   = 0;
 device_interface->command_response_data      = 0;
 device_interface->command_response_data_size = 0;
 
 
 //Try to get the response within the timeout period
 while(__mip_interface_time_timeout(start_time, timeout_ms) == MIP_INTERFACE_NO_TIMEOUT)
 {
  //Allow the parser to run
  mip_interface_update(device_interface);
  
  //Got a response
  if((device_interface->command_response_received == 1) && (device_interface->command_id == command_descriptor))
  {
   //Set the feedback
   *acknack_response   = device_interface->command_acknack_response;
   *response_data      = device_interface->command_response_data;
   *response_data_size = device_interface->command_response_data_size;
      
   //Break from the while loop & return ok
   return_code = MIP_INTERFACE_OK;
   break;
  }
 }

 //Reset the response callback variables
 device_interface->callback_data_set_list[0]  = 0;
 device_interface->command_id                 = 0; 
 device_interface->command_acknack_response   = 0;
 device_interface->command_response_data      = 0;
 device_interface->command_response_data_size = 0;

 return return_code;
}
