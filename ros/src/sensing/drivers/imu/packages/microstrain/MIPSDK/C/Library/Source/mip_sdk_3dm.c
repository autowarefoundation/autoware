/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_3dm.c 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP 3DM Descriptor Set Definition File
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


#include "mip_sdk_3dm.h"
#include "mip_sdk_user_functions.h"


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_poll_ahrs(mip_interface *device_interface, u8 option_selector, 
//!                           u8 num_descriptors, u8 *descriptor_list)
//
//! @section DESCRIPTION
//! Poll for an AHRS data packet given the provided descriptor format.  If no 
//! format is provided (i.e. num_descriptors = 0 and descriptor_list = NULL)
//! the device will output the AHRS packet given the current format stored in 
//! the device.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 option_selector              - ACK/NACK packet generation option.
//! @param [in] u8 num_descriptors              - The number of descriptors provided.
//! @param [in] u8 *descriptor_list             - A descriptor list containing \c num_descriptors descriptor values.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! The user receives the AHRS packet using the callback function for the AHRS data class. 
//!
//! Possible \c option_selector values:\n\n
//!    \li 0x00 - normal ACK/NACK reply\n
//!    \li 0x01 - Suppress the ACK/NACK reply\n\n
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_poll_ahrs(mip_interface *device_interface, u8 option_selector, u8 num_descriptors, u8 *descriptor_list)
{
 u8  command_data[MIP_MAX_PAYLOAD_SIZE]= {0};
 u8  wait_for_response = 0;
 u8  i;
 
 //Check the number of descriptors does not exceed the maximum that will fit into the command data
 if(num_descriptors >= MIP_MAX_PAYLOAD_SIZE)
  return MIP_INTERFACE_ERROR;
 
 //Fill-in the command data
 command_data[0] = option_selector;
 command_data[1] = num_descriptors;
 
 for(i=0; i<num_descriptors; i++)
 {
  command_data[2+i*3] = descriptor_list[i]; 
  command_data[3+i*3] = 0; 
  command_data[4+i*3] = 0;  
 }
 
 //Determine if we have to wait for a response
 if(option_selector == MIP_3DM_POLLING_ENABLE_ACK_NACK)
  wait_for_response = 1;
  
 //Send the command
 return mip_interface_send_command(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_POLL_AHRS_MESSAGE, 
                                   command_data, sizeof(u8)*2 + num_descriptors*3, wait_for_response, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_poll_gps(mip_interface *device_interface, u8 option_selector, 
//!                          u8 num_descriptors, u8 *descriptor_list)
//
//! @section DESCRIPTION
//! Poll for a GPS data packet given the provided descriptor format.  If no 
//! format is provided (i.e. num_descriptors = 0 and descriptor_list = NULL)
//! the device will output the AHRS packet given the current format stored in 
//! the device.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 option_selector              - ACK/NACK packet generation option.
//! @param [in] u8 num_descriptors              - The number of descriptors provided.
//! @param [in] u8 *descriptor_list             - A descriptor list containing \c num_descriptors descriptor values.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! The user receives the GPS packet using the callback function for the GPS data class. 
//!
//! Possible \c option_selector values:\n\n
//!    \li 0x00 - normal ACK/NACK reply\n
//!    \li 0x01 - Suppress the ACK/NACK reply\n\n
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_poll_gps(mip_interface *device_interface, u8 option_selector, u8 num_descriptors, u8 *descriptor_list)
{
 u8  command_data[MIP_MAX_PAYLOAD_SIZE]= {0};
 u8  wait_for_response = 0;
 u8  i;
 
 //Check the number of descriptors does not exceed the maximum that will fit into the command data
 if(num_descriptors >= MIP_MAX_PAYLOAD_SIZE)
  return MIP_INTERFACE_ERROR;
 
 //Fill-in the command data
 command_data[0] = option_selector;
 command_data[1] = num_descriptors;
 
 for(i=0; i<num_descriptors; i++)
 {
  command_data[2+i*3] = descriptor_list[i]; 
  command_data[3+i*3] = 0; 
  command_data[4+i*3] = 0;  
 }
 
 //Determine if we have to wait for a response
 if(option_selector == MIP_3DM_POLLING_ENABLE_ACK_NACK)
  wait_for_response = 1;
  
 //Send the command
 return mip_interface_send_command(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_POLL_GPS_MESSAGE, 
                                   command_data, sizeof(u8)*2 + num_descriptors*3, wait_for_response, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_poll_filter(mip_interface *device_interface, u8 option_selector, 
//!                          u8 num_descriptors, u8 *descriptor_list)
//
//! @section DESCRIPTION
//! Poll for a NAV data packet given the provided descriptor format.  If no 
//! format is provided (i.e. num_descriptors = 0 and descriptor_list = NULL)
//! the device will output the NAV packet given the current format stored in 
//! the device.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 option_selector              - ACK/NACK packet generation option.
//! @param [in] u8 num_descriptors              - The number of descriptors provided.
//! @param [in] u8 *descriptor_list             - A descriptor list containing \c num_descriptors descriptor values.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! The user receives the NAV packet using the callback function for the NAV data class. 
//!
//! Possible \c option_selector values:\n\n
//!    \li 0x00 - normal ACK/NACK reply\n
//!    \li 0x01 - Suppress the ACK/NACK reply\n\n
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_poll_filter(mip_interface *device_interface, u8 option_selector, u8 num_descriptors, u8 *descriptor_list)
{
 u8  command_data[MIP_MAX_PAYLOAD_SIZE]= {0};
 u8  wait_for_response = 0;
 u8  i;
 
 //Check the number of descriptors does not exceed the maximum that will fit into the command data
 if(num_descriptors >= MIP_MAX_PAYLOAD_SIZE)
  return MIP_INTERFACE_ERROR;
 
 //Fill-in the command data
 command_data[0] = option_selector;
 command_data[1] = num_descriptors;
 
 for(i=0; i<num_descriptors; i++)
 {
  command_data[2+i*3] = descriptor_list[i]; 
  command_data[3+i*3] = 0; 
  command_data[4+i*3] = 0;  
 }
 
 //Determine if we have to wait for a response
 if(option_selector == MIP_3DM_POLLING_ENABLE_ACK_NACK)
  wait_for_response = 1;
  
 //Send the command
 return mip_interface_send_command(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_POLL_NAV_MESSAGE, 
                                   command_data, sizeof(u8)*2 + num_descriptors*3, wait_for_response, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_get_ahrs_base_rate(mip_interface *device_interface, u16 *base_rate)
//
//! @section DESCRIPTION
//! Request the base rate of the AHRS data.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [out] u16 *base_rate                 - buffer for the returned base rate.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Rate is reported in Hz.
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_get_ahrs_base_rate(mip_interface *device_interface, u16 *base_rate)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 mip_field_header *field_header_ptr;

 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_GET_AHRS_BASE_RATE, NULL, 
                                                        0, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success
 if(return_code == MIP_INTERFACE_OK)
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_AHRS_BASE_RATE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u16)))
  {
   memcpy(base_rate, response_data + sizeof(mip_field_header), sizeof(u16));
   
   //Byteswap the base rate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(base_rate, sizeof(u16));
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
//! u16 mip_3dm_cmd_get_gps_base_rate(mip_interface *device_interface, u16 *base_rate)
//
//! @section DESCRIPTION
//! Request the base rate of the GPS data.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [out] u16 *base_rate                 - buffer for the returned base rate.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Rate is reported in Hz.
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_get_gps_base_rate(mip_interface *device_interface, u16 *base_rate)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 mip_field_header *field_header_ptr;

 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_GET_GPS_BASE_RATE, NULL, 
                                                        0, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success
 if(return_code == MIP_INTERFACE_OK)
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_GPS_BASE_RATE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u16)))
  {
   memcpy(base_rate, response_data + sizeof(mip_field_header), sizeof(u16));
   
   //Byteswap the base rate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(base_rate, sizeof(u16));
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
//! u16 mip_3dm_cmd_get_filter_base_rate(mip_interface *device_interface, u16 *base_rate)
//
//! @section DESCRIPTION
//! Request the base rate of the NAV data.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [out] u16 *base_rate                 - buffer for the returned base rate.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Rate is reported in Hz.
//!
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_get_filter_base_rate(mip_interface *device_interface, u16 *base_rate)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 mip_field_header *field_header_ptr;

 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_GET_NAV_BASE_RATE, NULL, 
                                                        0, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success
 if(return_code == MIP_INTERFACE_OK)
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_NAV_BASE_RATE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u16)))
  {
   memcpy(base_rate, response_data + sizeof(mip_field_header), sizeof(u16));
   
   //Byteswap the base rate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(base_rate, sizeof(u16));
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
//! u16 mip_3dm_cmd_ahrs_message_format(mip_interface *device_interface, u8 function_selector, 
//!                                     u8 *num_entries, u8 *descriptors, u16 *descimation)
//
//! @section DESCRIPTION
//! Set or read the current AHRS message format.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *num_entries             - The number of descriptors provided or the number read back from the device. (used to set or get depending on \c function_selector)
//! @param [in,out] u8 *descriptors             - A descriptor list containing \c num_descriptors descriptor values. (used to set or get depending on \c function_selector)
//! @param [in,out] u8 *decimation              - A decimation list containing \c num_descriptors decimation values. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! The user receives the AHRS packet using the callback function for the AHRS data class. 
//!
//! Possible \c function_selector values:\n\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c num_entries may be 0, descriptors = NULL, and decimation = NULL for the following \c function_selector values:
//!
//!    \li 0x01 - Use New Settings (will clear the current format)
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_ahrs_message_format(mip_interface *device_interface, u8 function_selector, u8 *num_entries, u8 *descriptors, u16 *decimation)
{
 u8  i;
 u8 *response_data;
 u8  local_num_entries;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[MIP_MAX_PAYLOAD_SIZE];
 u16 command_data_size = 0;
 u16 *short_ptr;
 mip_field_header *field_header_ptr;
 
 
 //Set the function selector
 command_data[0]   = function_selector;
 command_data_size = 1;
 
 //Fill-in the command data
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  //Add the number of entries 
  command_data[1]   = *num_entries;
  command_data_size = 2;
   
  //Loop through and add the descriptors and descimation
  for(i=0; i<*num_entries; i++)
  {
   command_data[2+i*3] = descriptors[i];
   short_ptr           = (u16*)&command_data[3+i*3];
   *short_ptr          = decimation[i];
   
   //Byteswap the descimation if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(short_ptr, sizeof(u16));
   }
   
   command_data_size += 3;
  }
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_AHRS_MESSAGE_FORMAT, command_data, 
                                                        command_data_size, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  //Must have at least the number of entries in the message format to continue
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_AHRS_MESSAGE_FORMAT) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   //Move the response data from the header to the data
   response_data += sizeof(mip_field_header);
   
   //Determine the number of entries
   local_num_entries = response_data[0];

   //Check to make sure the user's buffers have enough space   
   if(local_num_entries > *num_entries)
   {
    *num_entries = local_num_entries;
    return MIP_INTERFACE_ERROR;
   }
   
   *num_entries = local_num_entries;
   
   for(i=0; i<*num_entries; i++)
   {
    descriptors[i] = response_data[1+i*3];
    short_ptr      = (u16*)&response_data[2+i*3];
    decimation[i]  = *short_ptr;
   
    //Byteswap the descimation if enabled
    if(MIP_SDK_CONFIG_BYTESWAP)
    {
     byteswap_inplace(&decimation[i], sizeof(u16));
    }   
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
//! u16 mip_3dm_cmd_gps_message_format(mip_interface *device_interface, u8 function_selector, 
//!                                    u8 *num_entries, u8 *descriptors, u16 *descimation)
//
//! @section DESCRIPTION
//! Set or read the current GPS message format.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *num_entries             - The number of descriptors provided/available or the number read back from the device. (used to set or get depending on \c function_selector)
//! @param [in,out] u8 *descriptors             - A descriptor list containing \c num_descriptors descriptor values. (used to set or get depending on \c function_selector)
//! @param [in,out] u8 *decimation              - A decimation list containing \c num_descriptors decimation values. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! The user receives the GPS packet using the callback function for the GPS data class. 
//!
//! Possible \c function_selector values:\n\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c num_entries may be 0, descriptors = NULL, and decimation = NULL for the following \c function_selector values:
//!
//!    \li 0x01 - Use New Settings (will clear the current format)
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_gps_message_format(mip_interface *device_interface, u8 function_selector, u8 *num_entries, u8 *descriptors, u16 *decimation)
{
 u8  i;
 u8 *response_data;
 u8  local_num_entries;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[MIP_MAX_PAYLOAD_SIZE];
 u16 command_data_size = 0;
 u16 *short_ptr;
 mip_field_header *field_header_ptr;
 
 
 //Set the function selector
 command_data[0]   = function_selector;
 command_data_size = 1;
 
 //Fill-in the command data
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  //Add the number of entries 
  command_data[1]   = *num_entries;
  command_data_size = 2;
   
  //Loop through and add the descriptors and descimation
  for(i=0; i<*num_entries; i++)
  {
   command_data[2+i*3] = descriptors[i];
   short_ptr           = (u16*)&command_data[3+i*3];
   *short_ptr          = decimation[i];
   
   //Byteswap the descimation if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(short_ptr, sizeof(u16));
   }
   
   command_data_size += 3;
  }
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_GPS_MESSAGE_FORMAT, command_data, 
                                                        command_data_size, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  //Must have at least the number of entries in the message format to continue
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_GPS_MESSAGE_FORMAT) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   //Move the response data from the header to the data
   response_data += sizeof(mip_field_header);
   
   //Determine the number of entries
   local_num_entries = response_data[0];

   //Check to make sure the user's buffers have enough space   
   if(local_num_entries > *num_entries)
   {
    *num_entries = local_num_entries;
    return MIP_INTERFACE_ERROR;
   }
   
   *num_entries = local_num_entries;
   
   for(i=0; i<*num_entries; i++)
   {
    descriptors[i] = response_data[1+i*3];
    short_ptr      = (u16*)&response_data[2+i*3];
    decimation[i]  = *short_ptr;
   
    //Byteswap the descimation if enabled
    if(MIP_SDK_CONFIG_BYTESWAP)
    {
     byteswap_inplace(&decimation[i], sizeof(u16));
    }   
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
//! u16 mip_3dm_cmd_filter_message_format(mip_interface *device_interface, u8 function_selector, 
//!                                    u8 *num_entries, u8 *descriptors, u16 *descimation)
//
//! @section DESCRIPTION
//! Set or read the current NAV message format.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *num_entries             - The number of descriptors provided or the number read back from the device. (used to set or get depending on \c function_selector)
//! @param [in,out] u8 *descriptors             - A descriptor list containing \c num_descriptors descriptor values. (used to set or get depending on \c function_selector)
//! @param [in,out] u8 *decimation              - A decimation list containing \c num_descriptors decimation values. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! The user receives the NAV packet using the callback function for the NAV data class. 
//!
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c num_entries may be 0, descriptors = NULL, and decimation = NULL for the following \c function_selector values:
//!
//!    \li 0x01 - Use New Settings (will clear the current format)
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_filter_message_format(mip_interface *device_interface, u8 function_selector, u8 *num_entries, u8 *descriptors, u16 *decimation)
{
 u8  i;
 u8 *response_data;
 u8  local_num_entries;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[MIP_MAX_PAYLOAD_SIZE];
 u16 command_data_size = 0;
 u16 *short_ptr;
 mip_field_header *field_header_ptr;
 
 
 //Set the function selector
 command_data[0]   = function_selector;
 command_data_size = 1;
 
 //Fill-in the command data
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  //Add the number of entries 
  command_data[1]   = *num_entries;
  command_data_size = 2;
   
  //Loop through and add the descriptors and descimation
  for(i=0; i<*num_entries; i++)
  {
   command_data[2+i*3] = descriptors[i];
   short_ptr           = (u16*)&command_data[3+i*3];
   *short_ptr          = decimation[i];
   
   //Byteswap the descimation if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(short_ptr, sizeof(u16));
   }
   
   command_data_size += 3;
  }
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_NAV_MESSAGE_FORMAT, command_data, 
                                                        command_data_size, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  //Must have at least the number of entries in the message format to continue
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_NAV_MESSAGE_FORMAT) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   //Move the response data from the header to the data
   response_data += sizeof(mip_field_header);
   
   //Determine the number of entries
   local_num_entries = response_data[0];

   //Check to make sure the user's buffers have enough space   
   if(local_num_entries > *num_entries)
   {
    *num_entries = local_num_entries;
    return MIP_INTERFACE_ERROR;
   }
 
   *num_entries = local_num_entries;
  
   for(i=0; i<*num_entries; i++)
   {
    descriptors[i] = response_data[1+i*3];
    short_ptr      = (u16*)&response_data[2+i*3];
    decimation[i]  = *short_ptr;
   
    //Byteswap the descimation if enabled
    if(MIP_SDK_CONFIG_BYTESWAP)
    {
     byteswap_inplace(&decimation[i], sizeof(u16));
    }   
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
//! u16 mip_3dm_cmd_continuous_data_stream(mip_interface *device_interface, 
//!                                        u8 function_selector, u8 device_selector, u8 *enable)
//
//! @section DESCRIPTION
//! Control the streaming of AHRS, GPS, and NAV data packets.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in] u8 device_selector              - Selects which device is affected.
//! @param [in,out] u8 *enable                  - the enable/disable flag. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c enable may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \n Possible \c device_selector values:\n
//!    \li 0x01 - AHRS
//!    \li 0x02 - GPS
//!    \li 0x03 - NAV
//!
//! \n Possible \c enable values:\n
//!    \li 0x00 - Disable the datastream
//!    \li 0x01 - Enable the datastream
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_continuous_data_stream(mip_interface *device_interface, u8 function_selector, u8 device_selector, u8 *enable)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[3] = {0};
 mip_field_header *field_header_ptr;
 
 
 //Fill-in the command data
 command_data[0] = function_selector;
 command_data[1] = device_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[2] = *enable;
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_CONTROL_DATA_STREAM, command_data, 
                                                        3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_DATASTREAM_ENABLE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)*2))
  {
   memcpy(enable, response_data + sizeof(mip_field_header) + 1, sizeof(u8));
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_gps_dynamics_mode(mip_interface *device_interface, 
//!                                   u8 function_selector, u8 *dynamics_mode)
//
//! @section DESCRIPTION
//! Set or read the GPS Dynamics Mode on the GX3-35.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *dynamics_mode           - The dynamics mode. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c dynamics_mode may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \n Possible \c dynamics_mode values:\n
//!    \li 0x00 - Portable
//!    \li 0x02 - Stationary
//!    \li 0x03 - Pedestrian
//!    \li 0x04 - Automotive
//!    \li 0x05 - Sea
//!    \li 0x06 - Airborne < 1G
//!    \li 0x07 - Airborne < 2G
//!    \li 0x08 - Airborne < 4G
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_gps_dynamics_mode(mip_interface *device_interface, u8 function_selector, u8 *dynamics_mode)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[2]={0};
 mip_field_header *field_header_ptr;
 
 
 //Fill-in the command data
 command_data[0] = function_selector;

 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[1] = *dynamics_mode;
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_SET_GPS_DYNAMICS_MODE, command_data, 
                                                        2, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_GPS_DYNAMICS_MODE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   memcpy(dynamics_mode, response_data + sizeof(mip_field_header), sizeof(u8));
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_gps_advanced_settings(mip_interface *device_interface, u8 function_selector)
//
//! @section DESCRIPTION
//! Save, Load, or Restore the advanced GPS settings on the GX3-35.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_gps_advanced_settings(mip_interface *device_interface, u8 function_selector)
{
 return mip_interface_send_command(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_SAVE_RESTORE_GPS_SETTINGS, 
                                   &function_selector, 1, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);

}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_ahrs_signal_conditioning(mip_interface *device_interface, 
//!                                          u8 function_selector, 
//!                                          mip_ahrs_signal_settings *settings)
//
//! @section DESCRIPTION
//! Set or read the AHRS signal conditioning settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface        - The device interface.
//! @param [in] u8 function_selector                   - Selects which function to perform.
//! @param [in,out] mip_ahrs_signal_settings *settings - The AHRS signal conditioning settings structure. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c settings may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! This function only affects inertial and orientation values on the AHRS datastream.
//! If the NAV datastream is available, the values remain unaffected.
//! Please reference the device DCP for further information.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_ahrs_signal_conditioning(mip_interface *device_interface, u8 function_selector, mip_ahrs_signal_settings *settings)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(mip_ahrs_signal_settings)] = {0};
 mip_field_header         *field_header_ptr;
 mip_ahrs_signal_settings *settings_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 settings_ptr    = (mip_ahrs_signal_settings*)&command_data[1];
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  memcpy(settings_ptr, settings, sizeof(mip_ahrs_signal_settings));
  
  //Byteswap the baudrate if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   byteswap_inplace(&settings_ptr->orientation_decimation,  sizeof(u16));
   byteswap_inplace(&settings_ptr->data_conditioning_flags, sizeof(u16));
   byteswap_inplace(&settings_ptr->up_compensation,         sizeof(u16));
   byteswap_inplace(&settings_ptr->north_compensation,      sizeof(u16));
   byteswap_inplace(&settings_ptr->reserved,                sizeof(u16));
  }   
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_SET_AHRS_SIGNAL_COND, command_data, 
                                                        sizeof(u8) + sizeof(mip_ahrs_signal_settings), &response_data, &response_data_size, 
                                                        MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_AHRS_SIGNAL_SETTINGS) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(mip_ahrs_signal_settings)))
  {
   memcpy(settings, response_data + sizeof(mip_field_header), sizeof(mip_ahrs_signal_settings));
   
   //Byteswap the baudrate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(&settings->orientation_decimation,  sizeof(u16));
    byteswap_inplace(&settings->data_conditioning_flags, sizeof(u16));
    byteswap_inplace(&settings->up_compensation,         sizeof(u16));
    byteswap_inplace(&settings->north_compensation,      sizeof(u16));
    byteswap_inplace(&settings->reserved,                sizeof(u16));
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
//! u16 mip_3dm_cmd_ahrs_timestamp(mip_interface *device_interface, u8 function_selector, 
//!                                u8 *time_selector, u32 *time)
//
//! @section DESCRIPTION
//! Set or read the AHRS signal conditioning settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *time_selector           - The selection for the format of the time value.
//! @param [in,out] u32 *time                   - The current time value. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!
//! \n Possible \c time_selector values:\n
//!    \li 0x01 - Beacon Timestamp (Seconds)
//!    \li 0x02 - Beacon Timestamp (Nanoseconds)
//!    \li 0x02 - AHRS Internal Tick Counter
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_ahrs_timestamp(mip_interface *device_interface, u8 function_selector, u8 *time_selector, u32 *time)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[6] = {0};
 u32 *word_ptr;
 mip_field_header *field_header_ptr;
 
 
 //Fill-in the command data
 command_data[0] = function_selector;
 command_data[1] = *time_selector;
 word_ptr        = (u32*)&command_data[2];
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  *word_ptr = *time;
  
  //Byteswap the baudrate if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   byteswap_inplace(word_ptr, sizeof(u32));
  }   
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_SET_AHRS_TIMESTAMP, command_data, 
                                                        6, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_AHRS_TIMESTAMP_VALUE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8) + sizeof(u32)))
  {
   memcpy(time_selector, response_data + sizeof(mip_field_header),     sizeof(u8));
   memcpy(time,          response_data + sizeof(mip_field_header) + 1, sizeof(u32));
   
   //Byteswap the baudrate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(time, sizeof(u32));
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
//! u16 mip_3dm_cmd_startup_settings(mip_interface *device_interface, u8 function_selector)
//
//! @section DESCRIPTION
//! Set or read the device startup settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! The affected startup settings are listed in each device's DCP.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_startup_settings(mip_interface *device_interface, u8 function_selector)
{
 return mip_interface_send_command(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_SAVE_DEVICE_SETTINGS, 
                                   &function_selector, 1, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_uart_baudrate(mip_interface *device_interface, u8 function_selector, u32 *baudrate)
//
//! @section DESCRIPTION
//! Set or read the primary com port baudrate.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u32 *baudrate               - The baudrate value. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c baudrate may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! The valid baudrates are listed in each device's DCP.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_uart_baudrate(mip_interface *device_interface, u8 function_selector, u32 *baudrate)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[5] = {0};
 u32 *word_ptr;
 mip_field_header *field_header_ptr;
 
 
 //Fill-in the command data
 command_data[0] = function_selector;
 word_ptr        = (u32*)&command_data[1];
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  *word_ptr = *baudrate;
  
  //Byteswap the baudrate if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   byteswap_inplace(word_ptr, sizeof(u32));
  }   
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_UART_BAUDRATE, command_data, 
                                                        5, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_UART_BAUDRATE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u32)))
  {
   memcpy(baudrate, response_data + sizeof(mip_field_header), sizeof(u32));
   
   //Byteswap the baudrate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(baudrate, sizeof(u32));
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
//! u16 mip_3dm_cmd_datastream_format(mip_interface *device_interface, u8 function_selector, u8 device_selector, u8 *format)
//
//! @section DESCRIPTION
//! Set or read the datastream format on a particular datastream.  Valid on the GX3-35 only.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in] u8 device_selector              - Selects which device is affected.
//! @param [in,out] u8 *format                  - The selected format of the data. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c format may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c Possible \c device_selector values:
//!
//!    \li 0x01 - AHRS
//!    \li 0x02 - GPS
//!
//! \c Possible \c format values:
//!
//!    \li 0x01 - Standard MIP (default)
//!    \li 0x02 - Wrapped Raw (MIP wrapper around raw sensor data)
//! 
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_datastream_format(mip_interface *device_interface, u8 function_selector, u8 device_selector, u8 *format)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[3] = {0};
 mip_field_header *field_header_ptr;
 
 
 //Fill-in the command data
 command_data[0] = function_selector;
 command_data[1] = device_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[2] = *format;
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_DATASTREAM_FORMAT, command_data, 
                                                        3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_DATASTREAM_FORMAT) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)*2))
  {
   memcpy(format, response_data + sizeof(mip_field_header) + 1, sizeof(u8));
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_power_state(mip_interface *device_interface, u8 function_selector, u8 device_selector, u8 *power_state)
//
//! @section DESCRIPTION
//! Set or read the power state on a particular datastream.  Valid on the GX3-25 and GX3-35 only.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in] u8 device_selector              - Selects which device is affected.
//! @param [in,out] u8 *power_state             - The selected power state. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c power_state may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c Possible \c device_selector values:
//!
//!    \li 0x01 - AHRS
//!    \li 0x02 - GPS
//!
//! \c Possible \c format values:
//!
//!    \li 0x01 - On (full performance)
//!    \li 0x02 - On (low power mode)
//!    \li 0x03 - Sleep (very low power, fast startup)
//!    \li 0x04 - Off/Deep Sleep
//! 
//! Please see the device DCP for valid parameter combinations.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_power_state(mip_interface *device_interface, u8 function_selector, u8 device_selector, u8 *power_state)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[3] = {0};
 mip_field_header *field_header_ptr;
 
 
 //Fill-in the command data
 command_data[0] = function_selector;
 command_data[1] = device_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[2] = *power_state;
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_DEVICE_POWER_STATE, command_data, 
                                                        3, &response_data, &response_data_size, MIP_INTERFACE_LONG_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_POWER_STATE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)*2))
  {
   memcpy(power_state, response_data + sizeof(mip_field_header) + 1, sizeof(u8));
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_device_status(mip_interface *device_interface, u16 model_number, 
//!                               u8 status_selector, u8 *response_buffer, u16 *response_size)
//
//! @section DESCRIPTION
//! Get the current device status.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u16 model_number                - The model number of the device.
//! @param [in] u8 status_selector              - The type of status desired.
//! @param [out] u8 *response_buffer            - A buffer to hold the response.
//! @param [in,out] u16 *response_size          - On entry, the size of the buffer.  On exit, the size of the returned status message.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Due to the variable response from the device, this function \b DOES \b NOT byteswap the output.  This 
//! is the responsibility of the developer.
//!
//! \n Possible \c status_selector values:\n
//!    \li 0x01 - Basic Status
//!    \li 0x02 - Diagnostic Status
//!
//! 
//! Please see the device DCP for valid status selector values and the format of the returned status message.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer, u16 *response_size)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[3];
 mip_field_header *field_header_ptr;
 u16 *short_ptr;
 u16 user_buffer_size = *response_size;
 
 //Fill-in the command data
 short_ptr       = (u16*)&command_data[0];
 *short_ptr      = model_number;
 command_data[2] = status_selector;
 
 //Byteswap the model number, if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  byteswap_inplace(short_ptr, sizeof(u16));
 }   

 //Intialize the response size
 *response_size = 0;
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_DEVICE_STATUS, command_data, 
                                                        3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->size - sizeof(mip_field_header)) <= user_buffer_size)
  {
   memcpy(response_buffer, response_data + sizeof(mip_field_header), field_header_ptr->size - sizeof(mip_field_header));
   *response_size = field_header_ptr->size - sizeof(mip_field_header);
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
//! u16 mip_3dm_cmd_low_pass_filter_settings(mip_interface *device_interface, u8 function_selector, mip_low_pass_filter_settings *filter_settings)
//
//! @section DESCRIPTION
//! Get or set low pass filter settings for scaled data quantities.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface	- The device interface.
//! @param [in] u8 function_selector            - The model number of the device.
//! @param [in,out] mip_low_pass_filter_settings *filter_settings - structure specifying the data type to be filtered and the filter parameters. 
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Please see the device DCP for valid status selector values and the format of the returned message.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_low_pass_filter_settings(mip_interface *device_interface, u8 function_selector, mip_low_pass_filter_settings *filter_settings){

 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(mip_low_pass_filter_settings)] = {0};
 mip_field_header         *field_header_ptr;
 mip_low_pass_filter_settings *settings_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 settings_ptr    = (mip_low_pass_filter_settings*)&command_data[1];
 
 memcpy(settings_ptr, filter_settings, sizeof(mip_low_pass_filter_settings));
  
 //Byteswap the baudrate if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  byteswap_inplace(&settings_ptr->cutoff_frequency, sizeof(u16));
 }  
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_LOW_PASS_FILTER_SETTINGS, command_data, 
                                                        sizeof(u8) + sizeof(mip_low_pass_filter_settings), &response_data, &response_data_size, 
                                                        MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_ADVANCED_DATA_FILTER) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(mip_low_pass_filter_settings)))
  {
   memcpy(filter_settings, response_data + sizeof(mip_field_header), sizeof(mip_low_pass_filter_settings));
   
   //Byteswap the baudrate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(&filter_settings->cutoff_frequency,  sizeof(u16));
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
//! u16 mip_3dm_cmd_accel_bias(mip_interface *device_interface, u8 function_selector, float *bias_vector)
//
//! @section DESCRIPTION
//! Get or set Accelerometer x, y, z bias vector.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface	- The device interface.
//! @param [in] u8 function_selector            - The model number of the device.
//! @param [in,out] float *bias_vector		- pointer to array containing input bias vector on write commands or to contain output bias vector on read commands.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Please see the device DCP for valid status selector values and the format of the returned message.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_accel_bias(mip_interface *device_interface, u8 function_selector, float *bias_vector){

 u8 *response_data, i;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + 3*sizeof(float)] = {0};
 mip_field_header         *field_header_ptr;
 float *bias_element_pointer;
 
 //Fill-in the command data
 command_data[0] = function_selector;
  
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE){

  memcpy(&command_data[1], &bias_vector[0], 3*sizeof(float));

  bias_element_pointer = (float *)(&command_data[1]);

  //Byteswap if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0;i<3;i++)
    byteswap_inplace(&bias_element_pointer[i], sizeof(float));
  }  

 }

 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_ACCEL_BIAS, command_data, 
                                                        sizeof(u8) + 3*sizeof(float), &response_data, &response_data_size, 
                                                        MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_ACCEL_BIAS_VECTOR) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + 3*sizeof(float)))
  {
   memcpy(bias_vector, response_data + sizeof(mip_field_header), 3*sizeof(float));
   
   //Byteswap the baudrate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0;i<3;i++)
     byteswap_inplace(&bias_vector[i], sizeof(float));
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
//! u16 mip_3dm_cmd_gyro_bias(mip_interface *device_interface, u8 function_selector, float *bias_vector)
//
//! @section DESCRIPTION
//! Get or set Accelerometer x, y, z bias vector.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface	- The device interface.
//! @param [in] u8 function_selector            - The model number of the device.
//! @param [in,out] float *bias_vector		- pointer to array containing input bias vector on write commands or to contain output bias vector on read commands.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Please see the device DCP for valid status selector values and the format of the returned message.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_gyro_bias(mip_interface *device_interface, u8 function_selector, float *bias_vector)
{
 u8 *response_data, i;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + 3*sizeof(float)] = {0};
 mip_field_header         *field_header_ptr;
 float *bias_element_pointer;

 //Fill-in the command data
 command_data[0] = function_selector;
  
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE){

  memcpy(&command_data[1], &bias_vector[0], 3*sizeof(float));

  bias_element_pointer = (float *)(&command_data[1]);

  //Byteswap if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0;i<3;i++)
    byteswap_inplace(&bias_element_pointer[i], sizeof(float));
  }  

 }

 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_GYRO_BIAS, command_data, 
                                                        sizeof(u8) + 3*sizeof(float), &response_data, &response_data_size, 
                                                        MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_GYRO_BIAS_VECTOR) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + 3*sizeof(float)))
  {
   memcpy(bias_vector, response_data + sizeof(mip_field_header), 3*sizeof(float));
   
   //Byteswap the baudrate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0;i<3;i++)
     byteswap_inplace(&bias_vector[i], sizeof(float));
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
//! u16 mip_3dm_cmd_coning_sculling_compensation(mip_interface *device_interface, u8 function_selector, u8 *enable)
//
//! @section DESCRIPTION
//! Set or read setting of coning and sculling compensation enable.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *enable                  - the enable/disable flag. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c enable may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \n Possible \c enable values:\n
//!    \li 0x00 - Disable Coning and Sculling compensation
//!    \li 0x01 - Enable Coning and Sculling compensation
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_coning_sculling_compensation(mip_interface *device_interface, u8 function_selector, u8 *enable)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[2] = {0};
 mip_field_header *field_header_ptr;
 
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[1] = *enable;
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CDM_CONING_AND_SCULLING_ENABLE, command_data, 
                                                        2, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_CONING_AND_SCULLING_ENABLE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   memcpy(enable, response_data + sizeof(mip_field_header), sizeof(u8));
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_capture_gyro_bias(mip_interface *device_interface, u8 function_selector, float *bias_vector)
//
//! @section DESCRIPTION
//! Get or set Accelerometer x, y, z bias vector.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface	- The device interface.
//! @param [in] u8 function_selector            - The model number of the device.
//! @param [in,out] float *bias_vector		    - Pointer to array containing input bias vector on write commands or to contain output bias vector on read commands.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Please see the device DCP for valid status selector values and the format of the returned message.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_capture_gyro_bias(mip_interface *device_interface, u16 duration_ms, float *bias_vector)
{
 u8 *response_data, i;
 u16 response_data_size;
 u16 return_code;
 u16 command_data = duration_ms;
 mip_field_header         *field_header_ptr;

 //Byteswap if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  byteswap_inplace(&command_data, sizeof(u16));
 }  

 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_CAPTURE_GYRO_BIAS, (u8 *)&command_data, 
                                                        sizeof(u16), &response_data, &response_data_size, 
                                                        MIP_INTERFACE_GYRO_BIAS_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_GYRO_BIAS_VECTOR) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + 3*sizeof(float)))
  {
   memcpy(bias_vector, response_data + sizeof(mip_field_header), 3*sizeof(float));
   
   //Byteswap the baudrate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0;i<3;i++)
     byteswap_inplace(&bias_vector[i], sizeof(float));
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
//! u16 mip_3dm_cmd_hard_iron(mip_interface *device_interface, u8 function_selector, float *vector)
//
//! @section DESCRIPTION
//! Get or set the Hard Iron x, y, z vector.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface	- The device interface.
//! @param [in] u8 function_selector            - The model number of the device.
//! @param [in,out] float *vector		        - Pointer to array containing input hard iron vector on write commands or to contain output hard iron vector on read commands.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Please see the device DCP for valid status selector values and the format of the returned message.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_hard_iron(mip_interface *device_interface, u8 function_selector, float *vector)
{
 u8 *response_data, i;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + 3*sizeof(float)] = {0};
 mip_field_header *field_header_ptr;
 float *vector_pointer;

 //Fill-in the command data
 command_data[0] = function_selector;
  
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  memcpy(&command_data[1], &vector[0], 3*sizeof(float));

  vector_pointer = (float *)(&command_data[1]);

  //Byteswap if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0;i<3;i++)
    byteswap_inplace(&vector_pointer[i], sizeof(float));
  }  

 }

 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_HARD_IRON_VECTOR, command_data, 
                                                        sizeof(u8) + 3*sizeof(float), &response_data, &response_data_size, 
                                                        MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_HARD_IRON_VECTOR) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + 3*sizeof(float)))
  {
   memcpy(vector, response_data + sizeof(mip_field_header), 3*sizeof(float));
   
   //Byteswap the baudrate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0;i<3;i++)
     byteswap_inplace(&vector[i], sizeof(float));
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
//! u16 mip_3dm_cmd_soft_iron(mip_interface *device_interface, u8 function_selector, float *matrix)
//
//! @section DESCRIPTION
//! Get or set the Soft Iron 3x3 matrix.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface	- The device interface.
//! @param [in] u8 function_selector            - The model number of the device.
//! @param [in,out] float *vector		        - Pointer to array containing input soft iron 3x3 matrix on write commands or to contain output soft iron 3x3 matrix on read commands.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Please see the device DCP for valid status selector values and the format of the returned message.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_soft_iron(mip_interface *device_interface, u8 function_selector, float *matrix)
{
 u8 *response_data, i;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + 9*sizeof(float)] = {0};
 mip_field_header *field_header_ptr;
 float *matrix_pointer;

 //Fill-in the command data
 command_data[0] = function_selector;
  
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  memcpy(&command_data[1], &matrix[0], 9*sizeof(float));

  matrix_pointer = (float *)(&command_data[1]);

  //Byteswap if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0;i<9;i++)
    byteswap_inplace(&matrix_pointer[i], sizeof(float));
  }  

 }

 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_SOFT_IRON_MATRIX, command_data, 
                                                        sizeof(u8) + 9*sizeof(float), &response_data, &response_data_size, 
                                                        MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_SOFT_IRON_MATRIX) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + 9*sizeof(float)))
  {
   memcpy(matrix, response_data + sizeof(mip_field_header), 9*sizeof(float));
   
   //Byteswap the baudrate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0;i<9;i++)
     byteswap_inplace(&matrix[i], sizeof(float));
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
//! u16 mip_3dm_sensor2vehicle_tranformation(mip_interface *device_interface, u8 function_selector, float euler_angles[3])
//
//! @section DESCRIPTION
//! Get or set the sensor-to-vehicle transformation in Euler angle format.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface	- The device interface.
//! @param [in] u8 function_selector            - The model number of the device.
//! @param [in,out] float *vector		        - Pointer to array containing input Euler angles on write commands or to contain output Euler angles on read commands.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Please see the device DCP for valid status selector values and the format of the returned message.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_sensor2vehicle_tranformation(mip_interface *device_interface, u8 function_selector, float euler_angles[3])
{
 u8 *response_data, i;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + 3*sizeof(float)] = {0};
 mip_field_header *field_header_ptr;
 float *angle_pointer;

 //Fill-in the command data
 command_data[0] = function_selector;
  
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  memcpy(&command_data[1], &euler_angles[0], 3*sizeof(float));

  angle_pointer = (float *)(&command_data[1]);

  //Byteswap if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0;i<3;i++)
    byteswap_inplace(&angle_pointer[i], sizeof(float));
  }  

 }

 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CDM_SENSOR2VEHICLE_TRANSFORMATION, command_data, 
                                                        sizeof(u8) + 3*sizeof(float), &response_data, &response_data_size, 
                                                        MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_SENSOR2VEHICLE_TRANSFORMATION) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + 3*sizeof(float)))
  {
   memcpy(euler_angles, response_data + sizeof(mip_field_header), 3*sizeof(float));
   
   //Byteswap the baudrate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0;i<3;i++)
     byteswap_inplace(&euler_angles[i], sizeof(float));
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
//! u16 mip_3dm_cmd_complementary_filter_settings(mip_interface *device_interface, u8 function_selector, mip_complementary_filter_settings *settings)
//
//! @section DESCRIPTION
//! Get or set complementary filter settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface	- The device interface.
//! @param [in] u8 function_selector            - The model number of the device.
//! @param [in,out] mip_complementary_filter_settings *settings - structure for the complementary filter parameters. 
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Please see the device DCP for valid status selector values and the format of the returned message.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_complementary_filter_settings(mip_interface *device_interface, u8 function_selector, mip_complementary_filter_settings *settings)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(mip_complementary_filter_settings)] = {0};
 mip_field_header *field_header_ptr;
 mip_complementary_filter_settings *settings_pointer;

 //Fill-in the command data
 command_data[0] = function_selector;
  
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  memcpy(&command_data[1], settings, sizeof(mip_complementary_filter_settings));

  settings_pointer = (mip_complementary_filter_settings*)(&command_data[1]);

  //Byteswap if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   byteswap_inplace(&settings_pointer->north_compensation_time_constant, sizeof(float));
   byteswap_inplace(&settings_pointer->up_compensation_time_constant,    sizeof(float));
  }  
 }

 return_code = mip_interface_send_command_with_response(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_COMPLEMENTARY_FILTER, command_data, 
                                                        sizeof(u8) + sizeof(mip_complementary_filter_settings), &response_data, &response_data_size, 
                                                        MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_3DM_REPLY_COMPLEMENTARY_FILTER) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(mip_complementary_filter_settings)))
  {
   memcpy(settings, response_data + sizeof(mip_field_header), sizeof(mip_complementary_filter_settings));
   
   //Byteswap the baudrate if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(&settings->north_compensation_time_constant, sizeof(float));
    byteswap_inplace(&settings->up_compensation_time_constant,    sizeof(float)); 
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
//! u16 mip_3dm_cmd_rtcm_23_message(mip_interface *device_interface, u8* raw_data, u16 num_bytes)
//
//! @section DESCRIPTION
//! Send a raw RTCM 2.3 Message to the device
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface	- The device interface.
//! @param [in] u8* raw_data                    - Buffer containing "num_bytes" of a RTCM 2.3 message.
//! @param [in] u16 num_bytes                   - Size of the RTCM message. 
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! This function will send multiple commands when the number of bytes in "raw_data"\n
//! is larger than MIP_MAX_PAYLOAD_DATA_SIZE.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_rtcm_23_message(mip_interface *device_interface, u8* raw_data, u16 num_bytes)
{
 u16 curr_byte = 0, bytes_to_send;

 //Loop through the raw data bytes and send in MIP_MAX_PAYLOAD_DATA_SIZE size chunks
 while(curr_byte < num_bytes)
 {
  bytes_to_send = num_bytes - curr_byte;

  if(bytes_to_send > MIP_MAX_PAYLOAD_DATA_SIZE)
	bytes_to_send = MIP_MAX_PAYLOAD_DATA_SIZE;
 
  //Send the data
  if(mip_interface_send_command(device_interface, MIP_3DM_COMMAND_SET, MIP_3DM_CMD_RAW_RTCM_2_3_MESSAGE, &raw_data[curr_byte], 
                                bytes_to_send, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS) != MIP_INTERFACE_OK)
  {
   return MIP_INTERFACE_ERROR;
  }
  else
  {
   //Increment the current byte index
   curr_byte += bytes_to_send;
  }
 }

 return MIP_INTERFACE_OK;
}