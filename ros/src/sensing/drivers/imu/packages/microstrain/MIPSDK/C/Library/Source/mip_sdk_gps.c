/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_gps.c 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP GPS Descriptor Set Definition File
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


#include "mip_sdk_gps.h"
#include "byteswap_utilities.h"

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_gps_llh_pos_byteswap(mip_gps_llh_pos *llh_pos)
//
//! @section DESCRIPTION
//! Byteswap a GPS LLH Position Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_gps_llh_pos *llh_pos - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_gps_llh_pos_byteswap(mip_gps_llh_pos *llh_pos)
{
 byteswap_inplace(&llh_pos->latitude,            sizeof(double));
 byteswap_inplace(&llh_pos->longitude,           sizeof(double));
 byteswap_inplace(&llh_pos->ellipsoid_height,    sizeof(double));
 byteswap_inplace(&llh_pos->msl_height,          sizeof(double));
 byteswap_inplace(&llh_pos->horizontal_accuracy, sizeof(float));
 byteswap_inplace(&llh_pos->vertical_accuracy,   sizeof(float));
 byteswap_inplace(&llh_pos->valid_flags,         sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_gps_ecef_pos_byteswap(mip_gps_ecef_pos *ecef_pos)
//
//! @section DESCRIPTION
//! Byteswap a GPS ECEF Position Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_gps_ecef_pos *ecef_pos - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_gps_ecef_pos_byteswap(mip_gps_ecef_pos *ecef_pos)
{
 u8 i;

 for(i=0; i<3; i++)
  byteswap_inplace(&ecef_pos->x[i],       sizeof(double));
 
 byteswap_inplace(&ecef_pos->x_accuracy,  sizeof(float));
 byteswap_inplace(&ecef_pos->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_gps_ned_vel_byteswap(mip_gps_ned_vel *ned_vel)
//
//! @section DESCRIPTION
//! Byteswap a GPS NED Velocity Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_gps_ned_vel *ned_vel - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_gps_ned_vel_byteswap(mip_gps_ned_vel *ned_vel)
{
 u8 i;

 for(i=0; i<3; i++)
  byteswap_inplace(&ned_vel->v[i],            sizeof(float));
 
 byteswap_inplace(&ned_vel->speed,            sizeof(float));
 byteswap_inplace(&ned_vel->ground_speed,     sizeof(float));
 byteswap_inplace(&ned_vel->heading,          sizeof(float));
 byteswap_inplace(&ned_vel->speed_accuracy,   sizeof(float));
 byteswap_inplace(&ned_vel->heading_accuracy, sizeof(float));
 byteswap_inplace(&ned_vel->valid_flags,      sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_gps_ecef_vel_byteswap(mip_gps_ecef_vel *ecef_vel)
//
//! @section DESCRIPTION
//! Byteswap a GPS ECEF Velocity Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_gps_ecef_vel *ecef_vel - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_gps_ecef_vel_byteswap(mip_gps_ecef_vel *ecef_vel)
{ 
 u8 i;

 for(i=0; i<3; i++)
  byteswap_inplace(&ecef_vel->v[i],       sizeof(float));
 
 byteswap_inplace(&ecef_vel->v_accuracy,  sizeof(float));
 byteswap_inplace(&ecef_vel->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_gps_fix_info_byteswap(mip_gps_fix_info *fix_info)
//
//! @section DESCRIPTION
//! Byteswap a GPS Fix Info Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_gps_fix_info *fix_info - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_gps_fix_info_byteswap(mip_gps_fix_info *fix_info)
{
 byteswap_inplace(&fix_info->fix_flags,   sizeof(u16));
 byteswap_inplace(&fix_info->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_gps_sv_info_byteswap(mip_gps_sv_info *sv_info)
//
//! @section DESCRIPTION
//! Byteswap a GPS SV Info Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_gps_sv_info *sv_info - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_gps_sv_info_byteswap(mip_gps_sv_info *sv_info)
{
 byteswap_inplace(&sv_info->carrier_noise_ratio, sizeof(u16));
 byteswap_inplace(&sv_info->azimuth,             sizeof(s16));
 byteswap_inplace(&sv_info->elevation,           sizeof(s16));
 byteswap_inplace(&sv_info->sv_flags,            sizeof(u16));
 byteswap_inplace(&sv_info->valid_flags,         sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_gps_dop_byteswap(mip_gps_dop *dop)
//
//! @section DESCRIPTION
//! Byteswap a GPS DOP Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_gps_dop *dop - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_gps_dop_byteswap(mip_gps_dop *dop)
{
 byteswap_inplace(&dop->gdop,        sizeof(float));
 byteswap_inplace(&dop->pdop,        sizeof(float));
 byteswap_inplace(&dop->hdop,        sizeof(float));
 byteswap_inplace(&dop->vdop,        sizeof(float));
 byteswap_inplace(&dop->tdop,        sizeof(float));
 byteswap_inplace(&dop->ndop,        sizeof(float));
 byteswap_inplace(&dop->edop,        sizeof(float));
 byteswap_inplace(&dop->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_gps_utc_time_byteswap(mip_gps_utc_time *utc_time)
//
//! @section DESCRIPTION
//! Byteswap a GPS UTC Time Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_gps_utc_time *utc_time - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_gps_utc_time_byteswap(mip_gps_utc_time *utc_time)
{
 byteswap_inplace(&utc_time->year,        sizeof(u16));
 byteswap_inplace(&utc_time->msec,        sizeof(u32));
 byteswap_inplace(&utc_time->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_gps_time_byteswap(mip_gps_time *gps_time)
//
//! @section DESCRIPTION
//! Byteswap a GPS Time Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_gps_time *gps_time - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_gps_time_byteswap(mip_gps_time *gps_time)
{
 byteswap_inplace(&gps_time->tow,         sizeof(double));
 byteswap_inplace(&gps_time->week_number, sizeof(u16));
 byteswap_inplace(&gps_time->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_gps_clock_info_byteswap(mip_gps_clock_info *clock_info)
//
//! @section DESCRIPTION
//! Byteswap a GPS Clock Info Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_gps_clock_info *clock_info - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_gps_clock_info_byteswap(mip_gps_clock_info *clock_info)
{
 byteswap_inplace(&clock_info->bias,              sizeof(double));
 byteswap_inplace(&clock_info->drift,             sizeof(double));
 byteswap_inplace(&clock_info->accuracy_estimate, sizeof(double));
 byteswap_inplace(&clock_info->valid_flags,       sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_gps_hw_status_byteswap(mip_gps_hw_status *hw_status)
//
//! @section DESCRIPTION
//! Byteswap a GPS Hardware Status Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_gps_hw_status *hw_status - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_gps_hw_status_byteswap(mip_gps_hw_status *hw_status)
{
 byteswap_inplace(&hw_status->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_gps_dgps_info_byteswap(mip_gps_dgps_info *dgps_info)
//
//! @section DESCRIPTION
//! Byteswap a DGPS Info Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_gps_dgps_info *dgps_info - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_gps_dgps_info_byteswap(mip_gps_dgps_info *dgps_info)
{
 byteswap_inplace(&dgps_info->age,                 sizeof(float));
 byteswap_inplace(&dgps_info->base_station_id,     sizeof(s16));
 byteswap_inplace(&dgps_info->base_station_status, sizeof(s16));
 byteswap_inplace(&dgps_info->num_dgps_channels,   sizeof(u16));
 byteswap_inplace(&dgps_info->valid_flags,         sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_gps_dgps_channel_status_byteswap(mip_gps_dgps_channel_status *dgps_channel_status)
//
//! @section DESCRIPTION
//! Byteswap a DGPS Channel Status Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_gps_dgps_channel_status *dgps_channel_status - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_gps_dgps_channel_status_byteswap(mip_gps_dgps_channel_status *dgps_channel_status)
{
 byteswap_inplace(&dgps_channel_status->age,                         sizeof(float));
 byteswap_inplace(&dgps_channel_status->pseudorange_correction,      sizeof(float));
 byteswap_inplace(&dgps_channel_status->pseudorange_rate_correction, sizeof(float));
 byteswap_inplace(&dgps_channel_status->valid_flags,                 sizeof(u16));
}

