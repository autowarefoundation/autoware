/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_gps.h 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP GPS Descriptor Set Definitions
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

#ifndef _MIP_SDK_GPS_H
#define _MIP_SDK_GPS_H

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

#define MIP_GPS_DATA_SET					0x81


////////////////////////////////////////////////////////////////////////////////
//
// Descriptors 
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// GPS LEGACY (RAW) DATA DESCRIPTORS
////////////////////////////////////////////////////////////////////////////////

#define MIP_GPS_DATA_NMEA	           		0x01		// uBlox NMEA packet 
#define MIP_GPS_DATA_UBX		   			0x02		// uBlox packet 

////////////////////////////////////////////////////////////////////////////////
// GPS DATA DESCRIPTORS
////////////////////////////////////////////////////////////////////////////////

#define MIP_GPS_DATA_LLH_POS   			   	0x03
#define MIP_GPS_DATA_ECEF_POS		   		0x04
#define MIP_GPS_DATA_NED_VELOCITY		   	0x05
#define MIP_GPS_DATA_ECEF_VELOCITY	  		0x06
#define MIP_GPS_DATA_DOP		           	0x07
#define MIP_GPS_DATA_UTC_TIME	           	0x08
#define MIP_GPS_DATA_GPS_TIME  	           	0x09
#define MIP_GPS_DATA_CLOCK_INFO  	      	0x0A
#define MIP_GPS_DATA_FIX_INFO            	0x0B
#define MIP_GPS_DATA_SV_INFO             	0x0C
#define MIP_GPS_DATA_HW_STATUS	           	0x0D


////////////////////////////////////////////////////////////////////////////////
//
// Valid Data Flag Definitions
//
////////////////////////////////////////////////////////////////////////////////


///
// Position: Latitude, Longitude, Height
///

#define MIP_GPS_LLH_POS_LAT_LON_VALID             0x0001
#define MIP_GPS_LLH_POS_ELLIPSOID_HEIGHT_VALID    0x0002
#define MIP_GPS_LLH_MSL_HEIGHT_VALID              0x0004
#define MIP_GPS_LLH_POS_HORIZONTAL_ACCURACY_VALID 0x0008
#define MIP_GPS_LLH_POS_VERTICAL_ACCURACY_VALID   0x0010

#define MIP_GPS_LLH_POS_PACKET_VALID   (MIP_GPS_LLH_POS_LAT_LON_VALID | MIP_GPS_LLH_POS_ELLIPSOID_HEIGHT_VALID    |\
                                        MIP_GPS_LLH_MSL_HEIGHT_VALID  | MIP_GPS_LLH_POS_HORIZONTAL_ACCURACY_VALID |\
                                        MIP_GPS_LLH_POS_VERTICAL_ACCURACY_VALID)


///
// Position: ECEF (Earth-Centered, Earth-Fixed)
///

#define MIP_GPS_ECEF_POS_POSITION_VALID            0x0001
#define MIP_GPS_ECEF_POS_ACCURACY_ESTIMATE_VALID   0x0002

#define MIP_GPS_ECEF_POS_PACKET_VALID (MIP_GPS_ECEF_POS_POSITION_VALID | MIP_GPS_ECEF_POS_ACCURACY_ESTIMATE_VALID)


///
// Velocity: NED (North, East, Down)
///

#define MIP_GPS_NED_VEL_VELOCITY_VALID            0x0001
#define MIP_GPS_NED_VEL_SPEED_3D_VALID            0x0002
#define MIP_GPS_NED_VEL_GROUND_SPEED_VALID        0x0004
#define MIP_GPS_NED_VEL_HEADING_VALID             0x0008
#define MIP_GPS_NED_VEL_SPEED_ACCURACY_VALID      0x0010
#define MIP_GPS_NED_VEL_HEADING_ACCURACY_VALID    0x0020

#define MIP_GPS_NED_VEL_PACKET_VALID (MIP_GPS_NED_VEL_VELOCITY_VALID       | MIP_GPS_NED_VEL_SPEED_3D_VALID | \
                                      MIP_GPS_NED_VEL_GROUND_SPEED_VALID   | MIP_GPS_NED_VEL_HEADING_VALID  | \
                                      MIP_GPS_NED_VEL_SPEED_ACCURACY_VALID | MIP_GPS_NED_VEL_HEADING_ACCURACY_VALID)


///
// Velocity: ECEF (Earth-Centered, Earth-Fixed)
///

#define MIP_GPS_ECEF_VEL_VELOCITY_VALID            0x0001
#define MIP_GPS_ECEF_VEL_ACCURACY_ESTIMATE_VALID   0x0002

#define MIP_GPS_ECEF_VEL_PACKET_VALID (MIP_GPS_ECEF_VEL_VELOCITY_VALID | MIP_GPS_ECEF_VEL_ACCURACY_ESTIMATE_VALID)


///
// GPS Fix Information
///

#define MIP_GPS_FIX_INFO_FIX_TYPE_VALID  0x0001
#define MIP_GPS_FIX_INFO_NUM_SV_VALID    0x0002
#define MIP_GPS_FIX_INFO_FIX_FLAGS_VALID 0x0004

#define MIP_GPS_FIX_INFO_PACKET_VALID (MIP_GPS_FIX_INFO_FIX_TYPE_VALID | MIP_GPS_FIX_INFO_NUM_SV_VALID | \
                                       MIP_GPS_FIX_INFO_FIX_FLAGS_VALID)

#define MIP_GPS_FIX_TYPE_3D        0x00
#define MIP_GPS_FIX_TYPE_2D        0x01
#define MIP_GPS_FIX_TYPE_TIME_ONLY 0x02
#define MIP_GPS_FIX_TYPE_NONE      0x03
#define MIP_GPS_FIX_TYPE_INVALID   0x04



///
// GPS SV (Space Vehicle) Information
///

#define MIP_GPS_SV_INFO_MAX_SV_NUMBER 32

#define MIP_GPS_SV_INFO_CHANNEL_VALID              0x0001
#define MIP_GPS_SV_INFO_SV_ID_VALID                0x0002
#define MIP_GPS_SV_INFO_CARRIER_NOISE_RATIO_VALID  0x0008
#define MIP_GPS_SV_INFO_AZIMUTH_VALID              0x0010
#define MIP_GPS_SV_INFO_ELEVATION_VALID            0x0020
#define MIP_GPS_SV_INFO_SV_FLAGS_VALID             0x0040

#define MIP_GPS_SV_INFO_PACKET_VALID (MIP_GPS_SV_INFO_CHANNEL_VALID             | MIP_GPS_SV_INFO_SV_ID_VALID   | \
                                      MIP_GPS_SV_INFO_CARRIER_NOISE_RATIO_VALID | MIP_GPS_SV_INFO_AZIMUTH_VALID | \
                                      MIP_GPS_SV_INFO_ELEVATION_VALID           | MIP_GPS_SV_INFO_SV_FLAGS_VALID)

#define MIP_GPS_SV_FLAG_USED_FOR_NAVIGATION 0x01
#define MIP_GPS_SV_FLAG_HEALTH              0x02


///
// GPS DOP (Dilution of Precision) Information
///

#define MIP_GPS_DOP_GDOP_VALID              0x0001
#define MIP_GPS_DOP_PDOP_VALID              0x0002
#define MIP_GPS_DOP_HDOP_VALID              0x0004
#define MIP_GPS_DOP_VDOP_VALID              0x0008
#define MIP_GPS_DOP_TDOP_VALID              0x0010
#define MIP_GPS_DOP_NDOP_VALID              0x0020
#define MIP_GPS_DOP_EDOP_VALID              0x0040

#define MIP_GPS_DOP_PACKET_VALID  (MIP_GPS_DOP_GDOP_VALID | MIP_GPS_DOP_PDOP_VALID | MIP_GPS_DOP_HDOP_VALID | \
                                   MIP_GPS_DOP_VDOP_VALID | MIP_GPS_DOP_TDOP_VALID | MIP_GPS_DOP_NDOP_VALID | \
                                   MIP_GPS_DOP_EDOP_VALID)

  
///
// GPS UTC (Coordinated Universal Time) Time
///

#define MIP_GPS_UTC_TIME_GPS_TIME_DATE_VALID       0x0001
#define MIP_GPS_UTC_TIME_LEAP_SECONDS_KNOWN_VALID  0x0002

#define MIP_GPS_UTC_TIME_PACKET_VALID (MIP_GPS_UTC_TIME_GPS_TIME_DATE_VALID | MIP_GPS_UTC_TIME_LEAP_SECONDS_KNOWN_VALID)


///
// GPS Time
///

#define MIP_GPS_TIME_TOW_VALID          0x0001
#define MIP_GPS_TIME_WEEK_NUMBER_VALID  0x0002

#define MIP_GPS_TIME_PACKET_VALID (MIP_GPS_TIME_TOW_VALID | MIP_GPS_TIME_WEEK_NUMBER_VALID)


///
// GPS Clock Information
///

#define MIP_GPS_CLOCK_INFO_BIAS_VALID               0x0001
#define MIP_GPS_CLOCK_INFO_DRIFT_VALID              0x0002
#define MIP_GPS_CLOCK_INFO_ACCURACY_ESTIMATE_VALID  0x0004

#define MIP_GPS_CLOCK_INFO_PACKET_VALID (MIP_GPS_CLOCK_INFO_BIAS_VALID | MIP_GPS_CLOCK_INFO_DRIFT_VALID | \
                                         MIP_GPS_CLOCK_INFO_ACCURACY_ESTIMATE_VALID)

///
// GPS Hardware Status
///

#define MIP_GPS_HW_STATUS_SENSOR_STATE_VALID   0x0001
#define MIP_GPS_HW_STATUS_ANTENNA_STATE_VALID  0x0002
#define MIP_GPS_HW_STATUS_ANTENNA_POWER_VALID  0x0004

#define MIP_GPS_HW_STATUS_PACKET_VALID (MIP_GPS_HW_STATUS_SENSOR_STATE_VALID  | \
                                        MIP_GPS_HW_STATUS_ANTENNA_STATE_VALID | \
                                        MIP_GPS_HW_STATUS_ANTENNA_POWER_VALID)


#define MIP_GPS_SENSOR_STATE_OFF      0x00
#define MIP_GPS_SENSOR_STATE_ON       0x01
#define MIP_GPS_SENSOR_STATE_UNKNOWN  0x02

#define MIP_GPS_ANTENNA_STATE_INIT    0x01
#define MIP_GPS_ANTENNA_STATE_SHORT   0x02
#define MIP_GPS_ANTENNA_STATE_OPEN    0x03
#define MIP_GPS_ANTENNA_STATE_GOOD    0x04
#define MIP_GPS_ANTENNA_STATE_UNKNOWN 0x05

#define MIP_GPS_ANTENNA_POWER_OFF     0x00
#define MIP_GPS_ANTENNA_POWER_ON      0x01
#define MIP_GPS_ANTENNA_POWER_UNKNOWN 0x02


////////////////////////////////////////////////////////////////////////////////
//
// Field Data Structures
//
////////////////////////////////////////////////////////////////////////////////

#pragma pack(1)

///
// Position: Latitude, Longitude, Height
///

typedef struct _mip_gps_llh_pos
{
 double latitude, longitude;                    //(deg, deg)
 double ellipsoid_height, msl_height;           //(m)
 float  horizontal_accuracy, vertical_accuracy; //(m)
 u16    valid_flags;
}mip_gps_llh_pos;


///
// Position: ECEF (Earth-Centered, Earth-Fixed)
///

typedef struct _mip_gps_ecef_pos
{
 double x[3];       //(m)
 float  x_accuracy; //(m)
 u16    valid_flags;
}mip_gps_ecef_pos;


///
// Velocity: NED (North, East, Down)
///

typedef struct _mip_gps_ned_vel
{
 float v[3];             //(m/s)
 float speed;            //(m/s)
 float ground_speed;     //(m/s)
 float heading;          //(deg)
 float speed_accuracy;   //(m/s)
 float heading_accuracy; //(deg)
 u16   valid_flags;
}mip_gps_ned_vel;


///
// Velocity: ECEF (Earth-Centered, Earth-Fixed)
///

typedef struct _mip_ecef_vel
{
 float v[3];       //(m/s)
 float v_accuracy; //(m/s)
 u16   valid_flags;
}mip_gps_ecef_vel;


///
// GPS Fix Information
///

typedef struct _mip_fix_info
{
 u8  fix_type;
 u8  num_sv;
 u16 fix_flags; 
 u16 valid_flags;
}mip_gps_fix_info;


///
// GPS SV (Space Vehicle) Information
///

typedef struct _mip_gps_sv_info
{
 u8  channel, sv_id;
 u16 carrier_noise_ratio; //(dBHz)
 s16 azimuth, elevation;  //(deg) 
 u16 sv_flags;
 u16 valid_flags;
}mip_gps_sv_info;


///
// GPS DOP (Dilution of Precision) Information
///

typedef struct _mip_gps_dop
{
 float gdop;        //Geometric  DOP
 float pdop;        //Position   DOP
 float hdop;        //Horizontal DOP
 float vdop;        //Vertical   DOP
 float tdop;        //Time       DOP
 float ndop;        //Northing   DOP
 float edop;        //Easting    DOP
 u16   valid_flags;
}mip_gps_dop;


///
// GPS UTC (Coordinated Universal Time) Time
///

typedef struct _mip_gps_utc_time
{
 u16 year;
 u8  month, day;
 u8  hour, min, sec;  
 u32 msec;            //milliseconds
 u16 valid_flags;
}mip_gps_utc_time;


///
// GPS Time
///

typedef struct _mip_gps_time
{
 double tow;  //Time of Week (seconds)
 u16 week_number; 
 u16 valid_flags;
}mip_gps_time;


///
// GPS Clock Information
///

typedef struct _mip_gps_clock_info
{
 double bias;              //sec
 double drift;             //sec/sec
 double accuracy_estimate; //sec
 u16    valid_flags;
}mip_gps_clock_info;


///
// GPS Hardware Status
///

typedef struct _mip_gps_hw_status
{
 u8  sensor_state;
 u8  antenna_state;
 u8  antenna_power; 
 u16 valid_flags;
}mip_gps_hw_status;


///
// DGPS Information
///

typedef struct _mip_gps_dgps_info
{
 float age;
 s16   base_station_id;
 s16   base_station_status;
 u16   num_dgps_channels;
 u16   valid_flags;
}mip_gps_dgps_info;


///
// DGPS Channel Status
///

typedef struct _mip_gps_dgps_channel_status
{
 u8    sv_id;
 float age;
 float pseudorange_correction;
 float pseudorange_rate_correction;
 u16   valid_flags;
}mip_gps_dgps_channel_status;



#pragma pack()



////////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
////////////////////////////////////////////////////////////////////////////////

void mip_gps_llh_pos_byteswap(mip_gps_llh_pos *llh_pos);
void mip_gps_ecef_pos_byteswap(mip_gps_ecef_pos *ecef_pos);
void mip_gps_ned_vel_byteswap(mip_gps_ned_vel *ned_vel);
void mip_gps_ecef_vel_byteswap(mip_gps_ecef_vel *ecef_vel);
void mip_gps_fix_info_byteswap(mip_gps_fix_info *fix_info);
void mip_gps_sv_info_byteswap(mip_gps_sv_info *sv_info);
void mip_gps_dop_byteswap(mip_gps_dop *dop);
void mip_gps_utc_time_byteswap(mip_gps_utc_time *utc_time);
void mip_gps_time_byteswap(mip_gps_time *gps_time);
void mip_gps_clock_info_byteswap(mip_gps_clock_info *clock_info);
void mip_gps_hw_status_byteswap(mip_gps_hw_status *hw_status);

void mip_gps_dgps_info_byteswap(mip_gps_dgps_info *dgps_info);
void mip_gps_dgps_channel_status_byteswap(mip_gps_dgps_channel_status *dgps_channel_status);

#endif
