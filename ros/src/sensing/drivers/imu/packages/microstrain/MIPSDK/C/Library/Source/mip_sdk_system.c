/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_system.c 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP SYSTEM Descriptor Set Definition File
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


#include "mip_sdk_system.h"
#include "mip_sdk_user_functions.h"


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_system_com_mode(mip_interface *device_interface, u8 function_selector, u8 *com_mode)
//
//! @section DESCRIPTION
//! Set or read the communications mode.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *com_mode                - The communications mode. (used to set or get depending on \c function_selector)
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
//! \c com_mode may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \n Possible \c com_mode values:\n
//!    \li 0x01 - Standard Mode
//!    \li 0x02 - AHRS Direct (or Device 1)
//!    \li 0x03 - GPS Direct (or Device 2)
//!
//! Please check the device DCP for available communications modes
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_system_com_mode(mip_interface *device_interface, u8 function_selector, u8 *com_mode)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[2];
 mip_field_header *field_header_ptr;
 
 
 //Fill-in the command data
 command_data[0] = function_selector;
 command_data[1] = *com_mode;
 
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_SYSTEM_COMMAND_SET, MIP_SYSTEM_CMD_COM_MODE, command_data, 
                                                        2, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_SYSTEM_REPLY_COM_MODE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   memcpy(com_mode, response_data + sizeof(mip_field_header), sizeof(u8));
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


