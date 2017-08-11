/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_config.h 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description Target-Specific Configuration
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

#ifndef _MIP_SDK_CONFIG_H
#define _MIP_SDK_CONFIG_H

   
////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
////////////////////////////////////////////////////////////////////////////////
//! @def 

///
//The MIP protocol is big-endian, if the user needs little-endian, MIP_SDK_CONFIG_BYTESWAP should be 1.
///

#define MIP_SDK_CONFIG_BYTESWAP  1


///
//The maximum number of callbacks the interface structure can hold
//Notes:
//
//   Each callback takes up 9 bytes.  Several loops search the callback list for
//   valid callbacks, increasing the number will result in slightly slower execution.
//
///

#define MIP_INTERFACE_MAX_CALLBACKS 10


///
//The maximum time (in milliseconds) the MIP interface will wait for bytes to become available on the port
///

#define MIP_INTERFACE_PORT_READ_TIMEOUT_MS 10


///
//The maximum time the (in milliseconds) to wait for most command/response cycles to complete
///

#define MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS 1000


///
//The maximum time the (in milliseconds) to wait for command/response cycles that require long processing on the device to complete
//(e.g. turning sub-device power on/off, running a BIT, etc.)
///

#define MIP_INTERFACE_LONG_COMMAND_RESPONSE_TIMEOUT_MS 10000

///
//The maximum time in milliseconds to wait for the capture gyro bias command to return.
///

#define MIP_INTERFACE_GYRO_BIAS_COMMAND_RESPONSE_TIMEOUT_MS	45000


#endif