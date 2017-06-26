/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_3dm.h
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP 3DM Descriptor Set Definitions
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

#ifndef _MIP_SDK_3DM_H
#define _MIP_SDK_3DM_H


////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include "mip.h"
#include "mip_sdk_interface.h"
#include "mip_sdk_config.h"
#include "mip_sdk_gps.h"
#include "mip_sdk_ahrs.h"


////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
////////////////////////////////////////////////////////////////////////////////

//! @def

////////////////////////////////////////////////////////////////////////////////
//
// Descriptor Set designators - used in the Desc Set field of the MIP header
//
////////////////////////////////////////////////////////////////////////////////

#define MIP_3DM_COMMAND_SET						0x0C


////////////////////////////////////////////////////////////////////////////////
// 3DM COMMAND DESCRIPTORS (command desc are < 0x80)
////////////////////////////////////////////////////////////////////////////////

#define MIP_3DM_CMD_POLL_AHRS_MESSAGE			  0x01
#define MIP_3DM_CMD_POLL_GPS_MESSAGE			  0x02
#define MIP_3DM_CMD_POLL_NAV_MESSAGE			  0x03
#define MIP_3DM_CMD_GET_AHRS_BASE_RATE			  0x06
#define MIP_3DM_CMD_GET_GPS_BASE_RATE			  0x07
#define MIP_3DM_CMD_AHRS_MESSAGE_FORMAT			  0x08
#define MIP_3DM_CMD_GPS_MESSAGE_FORMAT			  0x09
#define MIP_3DM_CMD_NAV_MESSAGE_FORMAT			  0x0A
#define MIP_3DM_CMD_GET_NAV_BASE_RATE			  0x0B
#define MIP_3DM_CMD_CONTROL_DATA_STREAM			  0x11
#define MIP_3DM_CMD_RAW_RTCM_2_3_MESSAGE          0x20
#define MIP_3DM_CMD_SAVE_DEVICE_SETTINGS		  0x30
#define MIP_3DM_CMD_SET_GPS_DYNAMICS_MODE		  0x34
#define MIP_3DM_CMD_SET_AHRS_SIGNAL_COND		  0x35
#define MIP_3DM_CMD_SET_AHRS_TIMESTAMP			  0x36
#define MIP_3DM_CMD_ACCEL_BIAS				      0x37
#define MIP_3DM_CMD_GYRO_BIAS				      0x38
#define MIP_3DM_CMD_CAPTURE_GYRO_BIAS			  0x39
#define MIP_3DM_CMD_HARD_IRON_VECTOR			  0x3A
#define MIP_3DM_CMD_SOFT_IRON_MATRIX			  0x3B
#define MIP_3DM_CDM_CONING_AND_SCULLING_ENABLE	  0x3E
#define MIP_3DM_CDM_SENSOR2VEHICLE_TRANSFORMATION 0x3F
#define MIP_3DM_CMD_UART_BAUDRATE			      0x40
#define MIP_3DM_CMD_LOW_PASS_FILTER_SETTINGS	  0x50
#define MIP_3DM_CMD_COMPLEMENTARY_FILTER          0x51
#define MIP_3DM_CMD_DATASTREAM_FORMAT			  0x60
#define MIP_3DM_CMD_DEVICE_POWER_STATE			  0x61
#define MIP_3DM_CMD_SAVE_RESTORE_GPS_SETTINGS	  0x62
#define MIP_3DM_CMD_DEVICE_SETTINGS			      0x63
#define MIP_3DM_CMD_DEVICE_STATUS			      0x64



////////////////////////////////////////////////////////////////////////////////
// 3DM REPLY DESCRIPTORS (reply desc are >= 0x80)
////////////////////////////////////////////////////////////////////////////////

#define MIP_3DM_REPLY_AHRS_MESSAGE_FORMAT		    0x80
#define MIP_3DM_REPLY_GPS_MESSAGE_FORMAT  		    0x81
#define MIP_3DM_REPLY_NAV_MESSAGE_FORMAT   		    0x82
#define MIP_3DM_REPLY_AHRS_BASE_RATE  			    0x83
#define MIP_3DM_REPLY_GPS_BASE_RATE   			    0x84
#define MIP_3DM_REPLY_DATASTREAM_ENABLE			    0x85
#define MIP_3DM_REPLY_AHRS_SIGNAL_SETTINGS		    0x86
#define MIP_3DM_REPLY_UART_BAUDRATE			        0x87
#define MIP_3DM_REPLY_DATASTREAM_FORMAT    		    0x88
#define MIP_3DM_REPLY_POWER_STATE 			        0x89
#define MIP_3DM_REPLY_NAV_BASE_RATE   			    0x8A
#define MIP_3DM_REPLY_ADVANCED_DATA_FILTER		    0x8B
#define MIP_3DM_REPLY_DEVICE_STATUS  			    0x90
#define MIP_3DM_REPLY_COMMUNICATIONS_MODE		    0x91
#define MIP_3DM_REPLY_GPS_DYNAMICS_MODE			    0x92
#define MIP_3DM_REPLY_AHRS_TIMESTAMP_VALUE		    0x93
#define MIP_3DM_REPLY_COMPLEMENTARY_FILTER          0x97
#define MIP_3DM_REPLY_ACCEL_BIAS_VECTOR			    0x9A
#define MIP_3DM_REPLY_GYRO_BIAS_VECTOR			    0x9B
#define MIP_3DM_REPLY_HARD_IRON_VECTOR		        0x9C
#define MIP_3DM_REPLY_SOFT_IRON_MATRIX		        0x9D
#define MIP_3DM_REPLY_CONING_AND_SCULLING_ENABLE    0x9E
#define MIP_3DM_REPLY_SENSOR2VEHICLE_TRANSFORMATION 0x9F

////////////////////////////////////////////////////////////////////////////////
// 3DM PARAMETERS
////////////////////////////////////////////////////////////////////////////////

//Data Poll Option Byte
#define MIP_3DM_POLLING_ENABLE_ACK_NACK			0x00
#define MIP_3DM_POLLING_SUPPRESS_ACK_NACK		0x01


//Device Power State Device Mask Byte
#define MIP_3DM_POWER_STATE_DEVICE_AHRS			0x01
#define MIP_3DM_POWER_STATE_DEVICE_GPS			0x02


//Device Power State Byte
#define MIP_3DM_POWER_STATE_ON				0x01
#define MIP_3DM_POWER_STATE_LOW_POWER			0x02
#define MIP_3DM_POWER_STATE_SLEEP			0x03
#define MIP_3DM_POWER_STATE_OFF				0x04


//3DM Device Settings Flags
#define MIP_3DM_AHRS_STREAM_ENABLED_FLAG		0x00000001
#define MIP_3DM_AHRS_STREAM_RAW_FORMAT_FLAG		0x00000002
#define MIP_3DM_GPS_STREAM_ENABLED_FLAG			0x00000100
#define MIP_3DM_GPS_STREAM_RAW_FORMAT_FLAG		0x00000200
#define MIP_3DM_NAV_STREAM_ENABLED_FLAG			0x00010000


//Data stream IDs
#define MIP_3DM_AHRS_DATASTREAM				0x01
#define MIP_3DM_GPS_DATASTREAM				0x02
#define MIP_3DM_INS_DATASTREAM				0x03


//Data stream communication format
#define MIP_3DM_DATASTREAM_NATIVE_MIP_FORMAT		0x01
#define MIP_3DM_DATASTREAM_WRAPPED_RAW_FORMAT		0x02


//GPS Dynamics Modes
#define MIP_3DM_GPS_DYNAMICS_MODE_PORTABLE		0x00
#define MIP_3DM_GPS_DYNAMICS_MODE_FIXED			0x01
#define MIP_3DM_GPS_DYNAMICS_MODE_STATIONARY	0x02
#define MIP_3DM_GPS_DYNAMICS_MODE_PEDESTRIAN	0x03
#define MIP_3DM_GPS_DYNAMICS_MODE_AUTOMOTIVE	0x04
#define MIP_3DM_GPS_DYNAMICS_MODE_SEA			0x05
#define MIP_3DM_GPS_DYNAMICS_MODE_AIRBORNE_1G	0x06
#define MIP_3DM_GPS_DYNAMICS_MODE_AIRBORNE_2G	0x07
#define MIP_3DM_GPS_DYNAMICS_MODE_AIRBORNE_4G	0x08

//IMU Low-Pass Filter Parameters
#define MIP_3DM_AHRS_LPF_DISABLED			0x00
#define MIP_3DM_AHRS_LPF_IIR_SINGLE_POLE	0x01
#define MIP_3DM_AHRS_LPF_MAX_FILTER			MIP_3DM_AHRS_LPF_IIR_SINGLE_POLE
#define MIP_3DM_AHRS_LPF_CUTOFF_MANUAL		0x01
#define MIP_3DM_AHRS_LPF_CUTOFF_AUTOMATIC	0x00

//Coning and Sculling Compensation parameters
#define MIP_3DM_CONING_AND_SCULLING_DISABLE		0x00
#define MIP_3DM_CONING_AND_SCULLING_ENABLE		0x01

////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
////////////////////////////////////////////////////////////////////////////////


u16 mip_3dm_cmd_poll_ahrs(mip_interface *device_interface, u8 option_selector, u8 num_descriptors, u8 *descriptor_list);
u16 mip_3dm_cmd_poll_gps(mip_interface *device_interface,  u8 option_selector, u8 num_descriptors, u8 *descriptor_list);
u16 mip_3dm_cmd_poll_filter(mip_interface *device_interface,  u8 option_selector, u8 num_descriptors, u8 *descriptor_list);

u16 mip_3dm_cmd_get_ahrs_base_rate(mip_interface *device_interface, u16 *base_rate);
u16 mip_3dm_cmd_get_gps_base_rate(mip_interface *device_interface,  u16 *base_rate);
u16 mip_3dm_cmd_get_filter_base_rate(mip_interface *device_interface,  u16 *base_rate);

u16 mip_3dm_cmd_ahrs_message_format(mip_interface *device_interface, u8 function_selector, u8 *num_entries, u8 *descriptors, u16 *decimation);
u16 mip_3dm_cmd_gps_message_format( mip_interface *device_interface, u8 function_selector, u8 *num_entries, u8 *descriptors, u16 *decimation);
u16 mip_3dm_cmd_filter_message_format( mip_interface *device_interface, u8 function_selector, u8 *num_entries, u8 *descriptors, u16 *decimation);

u16 mip_3dm_cmd_continuous_data_stream(mip_interface *device_interface, u8 function_selector, u8 device_selector, u8 *enable);
u16 mip_3dm_cmd_startup_settings(mip_interface *device_interface, u8 function_selector);

u16 mip_3dm_cmd_gps_dynamics_mode(mip_interface *device_interface, u8 function_selector, u8 *dynamics_mode);
u16 mip_3dm_cmd_gps_advanced_settings(mip_interface *device_interface, u8 function_selector);

u16 mip_3dm_cmd_ahrs_signal_conditioning(mip_interface *device_interface, u8 function_selector, mip_ahrs_signal_settings *settings);
u16 mip_3dm_cmd_ahrs_timestamp(mip_interface *device_interface, u8 function_selector, u8 *time_selector, u32 *time);

u16 mip_3dm_cmd_uart_baudrate(mip_interface *device_interface, u8 function_selector, u32 *baudrate);
u16 mip_3dm_cmd_datastream_format(mip_interface *device_interface, u8 function_selector, u8 device_selector, u8 *format);
u16 mip_3dm_cmd_power_state(mip_interface *device_interface, u8 function_selector, u8 device_selector, u8 *power_state);

u16 mip_3dm_cmd_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer, u16 *response_size);
u16 mip_3dm_cmd_low_pass_filter_settings(mip_interface *device_interface, u8 function_selector, mip_low_pass_filter_settings *filter_settings);

u16 mip_3dm_cmd_accel_bias(mip_interface *device_interface, u8 function_selector, float *bias_vector);
u16 mip_3dm_cmd_gyro_bias(mip_interface *device_interface, u8 function_selector, float *bias_vector);

u16 mip_3dm_cmd_coning_sculling_compensation(mip_interface *device_interface, u8 function_selector, u8 *enable);
u16 mip_3dm_cmd_capture_gyro_bias(mip_interface *device_interface, u16 duration_ms, float *bias_vector);

u16 mip_3dm_cmd_hard_iron(mip_interface *device_interface, u8 function_selector, float *vector);
u16 mip_3dm_cmd_soft_iron(mip_interface *device_interface, u8 function_selector, float *matrix);

u16 mip_3dm_sensor2vehicle_tranformation(mip_interface *device_interface, u8 function_selector, float euler_angles[3]);

u16 mip_3dm_cmd_complementary_filter_settings(mip_interface *device_interface, u8 function_selector, mip_complementary_filter_settings *settings);

u16 mip_3dm_cmd_rtcm_23_message(mip_interface *device_interface, u8* raw_data, u16 num_bytes);


#endif
