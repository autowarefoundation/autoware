/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_base.c 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP BASE Descriptor Set Definitions
//
// External dependencies:
//
//  mip.h
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


#include "mip_sdk_base.h"
#include "mip_sdk_user_functions.h"


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_base_cmd_ping(mip_interface *device_interface)
//
//! @section DESCRIPTION
//! Ping the device.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_base_cmd_ping(mip_interface *device_interface)
{
 return mip_interface_send_command(device_interface, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_PING, 
                                   NULL, 0, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_base_cmd_idle(mip_interface *device_interface)
//
//! @section DESCRIPTION
//! Put the device in idle mode.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_base_cmd_idle(mip_interface *device_interface)
{
 return mip_interface_send_command(device_interface, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_SET_TO_IDLE, 
                                   NULL, 0, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_base_cmd_get_device_info(mip_interface *device_interface, base_device_info_field *device_info)
//
//! @section DESCRIPTION
//! Get the device information structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface       - The device interface.
//! @param [out] base_device_info_field *device_info  - The returned device info structure.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_base_cmd_get_device_info(mip_interface *device_interface, base_device_info_field *device_info)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 mip_field_header *field_header_ptr;

 memset(device_info, 0, sizeof(base_device_info_field));

 return_code = mip_interface_send_command_with_response(device_interface, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_GET_DEVICE_INFO, NULL, 
                                                        0, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success
 if(return_code == MIP_INTERFACE_OK)
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_REPLY_DESC_BASE_DEVICE_INFO) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(base_device_info_field)))
  {
   memcpy(device_info, response_data + sizeof(mip_field_header), response_data_size - sizeof(mip_field_header));
   
   //Byteswap if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(&device_info->firmware_version, 2);
   }
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_base_cmd_get_device_supported_descriptors(mip_interface *device_interface, u8 *response_buffer, u16 *response_size)
//
//! @section DESCRIPTION
//! Get the descriptors supported by the device.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [out] u8 *response_buffer            - The returned array of descriptors.
//! @param [in, out] u16 *response_size         - On entry, the size of the buffer; On exit, the size of the data returned.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! The provided buffer should be large enough to hold the maximum number of\n
//! supported descriptors from any device (currently 126.)  This would be a\n
//! buffer of at least 252 bytes.  The returned descriptors are in the format\n
//! of u16 = [descriptor set (MSB), descriptor (LSB)].
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_base_cmd_get_device_supported_descriptors(mip_interface *device_interface, u8 *response_buffer, u16 *response_size)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 mip_field_header *field_header_ptr;
 u16 *short_ptr;
 u16 user_buffer_size = *response_size;
 u16 i;
 
 //Initialize the response size
 *response_size = 0;
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_GET_DEVICE_DESCRIPTORS, NULL, 
                                                        0, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->size - sizeof(mip_field_header)) <= user_buffer_size)
  {
   memcpy(response_buffer, response_data + sizeof(mip_field_header), field_header_ptr->size - sizeof(mip_field_header));
   *response_size = field_header_ptr->size - sizeof(mip_field_header);
  
   //Byteswap the descriptors if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    short_ptr = (u16*)response_buffer;
    
    for(i=0; i<*response_size/2; i++)
     byteswap_inplace(&short_ptr[i],  sizeof(u16));
   }   
  }
  else 
  {
   *response_size = 0;
   return_code = MIP_INTERFACE_ERROR;
  }
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_base_cmd_built_in_test(mip_interface *device_interface, u32 *bit_result)
//
//! @section DESCRIPTION
//! Perform the device's built-in test.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [out] u32 *bit_result                - The returned bitfield result.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Please consult the device DCP for descriptions of BIT result values.\n
//! A value of 0x00000000 is always success, other values indicate an error.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_base_cmd_built_in_test(mip_interface *device_interface, u32 *bit_result)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 mip_field_header *field_header_ptr;

 return_code = mip_interface_send_command_with_response(device_interface, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_BUILT_IN_TEST, NULL, 
                                                        0, &response_data, &response_data_size, MIP_INTERFACE_LONG_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success
 if(return_code == MIP_INTERFACE_OK)
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_REPLY_DESC_BASE_BUILT_IN_TEST) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u32)))
  {
   memcpy(bit_result, response_data + sizeof(mip_field_header), response_data_size - sizeof(mip_field_header));

   //Byteswap the base rate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(bit_result, sizeof(u32));
   }
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_base_cmd_resume(mip_interface *device_interface)
//
//! @section DESCRIPTION
//! Resume any enabled continuous data streams.  This is the reverse of \c idle.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_base_cmd_resume(mip_interface *device_interface)
{
 return mip_interface_send_command(device_interface, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_RESUME, 
                                   NULL, 0, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_base_cmd_reset_device(mip_interface *device_interface)
//
//! @section DESCRIPTION
//! Reset the attached device.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! If communicating via USB, it is important to close the port immediately after\n
//! calling this function.  Failure to do so may result in the designated port\n
//! hanging.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_base_cmd_reset_device(mip_interface *device_interface)
{
 return mip_interface_send_command(device_interface, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_SOFT_RESET, 
                                   NULL, 0, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


