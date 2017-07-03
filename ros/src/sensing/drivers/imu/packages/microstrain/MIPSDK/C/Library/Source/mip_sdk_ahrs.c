/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_ahrs.c 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP AHRS Descriptor Set Definition File
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


#include "mip_sdk_ahrs.h"
#include "byteswap_utilities.h"


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_raw_accel_byteswap(mip_ahrs_raw_accel *raw_accel)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Raw Accel Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_raw_accel *raw_accel - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_raw_accel_byteswap(mip_ahrs_raw_accel *raw_accel)
{
 u8 i;

 for(i=0; i<3; i++)
  byteswap_inplace(&raw_accel->raw_accel[i], sizeof(float));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_raw_gyro_byteswap(mip_ahrs_raw_gyro *raw_gyro)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Raw Gyro Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_raw_gyro *raw_gyro - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_raw_gyro_byteswap(mip_ahrs_raw_gyro *raw_gyro)
{
 u8 i;

 for(i=0; i<3; i++)
  byteswap_inplace(&raw_gyro->raw_gyro[i], sizeof(float));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_raw_mag_byteswap(mip_ahrs_raw_mag *raw_mag)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Raw Mag Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_raw_mag *raw_mag - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_raw_mag_byteswap(mip_ahrs_raw_mag *raw_mag)
{
 u8 i;

 for(i=0; i<3; i++)
  byteswap_inplace(&raw_mag->raw_mag[i], sizeof(float));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_scaled_accel_byteswap(mip_ahrs_scaled_accel *scaled_accel)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Scaled Accel Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_scaled_accel *scaled_accel - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_scaled_accel_byteswap(mip_ahrs_scaled_accel *scaled_accel)
{
 u8 i;

 for(i=0; i<3; i++)
  byteswap_inplace(&scaled_accel->scaled_accel[i], sizeof(float));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_scaled_gyro_byteswap(mip_ahrs_scaled_gyro *scaled_gyro)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Scaled Gyro Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_scaled_gyro *scaled_gyro - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_scaled_gyro_byteswap(mip_ahrs_scaled_gyro *scaled_gyro)
{
 u8 i;

 for(i=0; i<3; i++)
  byteswap_inplace(&scaled_gyro->scaled_gyro[i], sizeof(float));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_scaled_mag_byteswap(mip_ahrs_scaled_mag *scaled_mag)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Scaled Mag Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_scaled_mag *scaled_mag - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_scaled_mag_byteswap(mip_ahrs_scaled_mag *scaled_mag)
{
 u8 i;

 for(i=0; i<3; i++)
  byteswap_inplace(&scaled_mag->scaled_mag[i], sizeof(float));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_delta_theta_byteswap(mip_ahrs_delta_theta *delta_theta)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Delta Theta Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_delta_theta *delta_theta - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_delta_theta_byteswap(mip_ahrs_delta_theta *delta_theta)
{
 u8 i;

 for(i=0; i<3; i++)
  byteswap_inplace(&delta_theta->delta_theta[i], sizeof(float));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_delta_velocity_byteswap(mip_ahrs_delta_velocity *delta_velocity)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Delta Velocity Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_delta_velocity *delta_velocity - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_delta_velocity_byteswap(mip_ahrs_delta_velocity *delta_velocity)
{
 u8 i;

 for(i=0; i<3; i++)
  byteswap_inplace(&delta_velocity->delta_velocity[i], sizeof(float));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_orientation_matrix_byteswap(mip_ahrs_orientation_matrix *orientation_matrix)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Orientation Matrix Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_orientation_matrix *orientation_matrix - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_orientation_matrix_byteswap(mip_ahrs_orientation_matrix *orientation_matrix)
{
 u8 i, j;

 for(i=0; i<3; i++)
 {
  for(j=0; j<3; j++)
   byteswap_inplace(&orientation_matrix->m[i][j], sizeof(float));
 }
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_quaternion_byteswap(mip_ahrs_quaternion *quaternion)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Quaternion Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_quaternion *quaternion - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_quaternion_byteswap(mip_ahrs_quaternion *quaternion)
{
 u8 i;

 for(i=0; i<4; i++)
  byteswap_inplace(&quaternion->q[i], sizeof(float));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_euler_angles_byteswap(mip_ahrs_euler_angles *euler_angles)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Euler Angle Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_euler_angles *euler_angles - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_euler_angles_byteswap(mip_ahrs_euler_angles *euler_angles)
{
 byteswap_inplace(&euler_angles->roll,  sizeof(float));
 byteswap_inplace(&euler_angles->pitch, sizeof(float));
 byteswap_inplace(&euler_angles->yaw,   sizeof(float));   
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_orientation_update_matrix_byteswap(mip_ahrs_orientation_update_matrix *orientation_update_matrix)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Orientation Update Matrix Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_orientation_update_matrix *orientation_update_matrix - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_orientation_update_matrix_byteswap(mip_ahrs_orientation_update_matrix *orientation_update_matrix)
{
 u8 i, j;

 for(i=0; i<3; i++)
 {
  for(j=0; j<3; j++)
   byteswap_inplace(&orientation_update_matrix->m[i][j], sizeof(float));
 }
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_raw_temp_byteswap(mip_ahrs_raw_temp *raw_temp)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Raw Temp Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_raw_temp *raw_temp - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_raw_temp_byteswap(mip_ahrs_raw_temp *raw_temp)
{
 u8 i;

 for(i=0; i<4; i++)
  byteswap_inplace(&raw_temp->raw_temp, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_internal_timestamp_byteswap(mip_ahrs_internal_timestamp *internal_timestamp)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Internal Timestamp Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_internal_timestamp *internal_timestamp - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_internal_timestamp_byteswap(mip_ahrs_internal_timestamp *internal_timestamp)
{
 byteswap_inplace(&internal_timestamp->counts, sizeof(u32));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_1pps_timestamp_byteswap(mip_ahrs_1pps_timestamp *pps_timestamp)
//
//! @section DESCRIPTION
//! Byteswap an AHRS 1PPS Timestamp Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_1pps_timestamp *pps_timestamp - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_1pps_timestamp_byteswap(mip_ahrs_1pps_timestamp *pps_timestamp)
{
 byteswap_inplace(&pps_timestamp->seconds,     sizeof(u32));
 byteswap_inplace(&pps_timestamp->nanoseconds, sizeof(u32));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_gps_timestamp_byteswap(mip_ahrs_gps_timestamp *gps_timestamp)
//
//! @section DESCRIPTION
//! Byteswap an AHRS GPS Timestamp Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_gps_timestamp *gps_timestamp - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_gps_timestamp_byteswap(mip_ahrs_gps_timestamp *gps_timestamp)
{
 byteswap_inplace(&gps_timestamp->tow,         sizeof(double));
 byteswap_inplace(&gps_timestamp->week_number, sizeof(u16));
 byteswap_inplace(&gps_timestamp->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_up_vector_byteswap(mip_ahrs_up_vector *up_vector)
//
//! @section DESCRIPTION
//! Byteswap an AHRS Up Vector Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_up_vector *up_vector - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_up_vector_byteswap(mip_ahrs_up_vector *up_vector)
{
 u8 i;

 for(i=0; i<3; i++)
  byteswap_inplace(&up_vector->up, sizeof(float));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_north_vector_byteswap(mip_ahrs_north_vector *north_vector)
//
//! @section DESCRIPTION
//! Byteswap an AHRS North Vector Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_north_vector *north_vector - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_north_vector_byteswap(mip_ahrs_north_vector *north_vector)
{
 u8 i;

 for(i=0; i<3; i++)
  byteswap_inplace(&north_vector->north, sizeof(float));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_ahrs_signal_settings_byteswap(mip_ahrs_signal_settings *signal_settings)
//
//! @section DESCRIPTION
//! Byteswap an AHRS signal conditioning Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_ahrs_signal_settings *signal_settings - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_ahrs_signal_settings_byteswap(mip_ahrs_signal_settings *signal_settings)
{
 byteswap_inplace(&signal_settings->orientation_decimation,  sizeof(u16));
 byteswap_inplace(&signal_settings->data_conditioning_flags, sizeof(u16));
 byteswap_inplace(&signal_settings->up_compensation,         sizeof(u16));
 byteswap_inplace(&signal_settings->north_compensation,      sizeof(u16));
 byteswap_inplace(&signal_settings->reserved,                sizeof(u16));
}

