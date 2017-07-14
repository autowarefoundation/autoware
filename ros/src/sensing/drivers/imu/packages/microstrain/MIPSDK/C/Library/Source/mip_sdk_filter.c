/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_filter.c 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description MIP Filter Descriptor Set Definitions
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


#include "mip_sdk_filter.h"
#include "mip_sdk_system.h"
#include "mip_sdk_user_functions.h"
#include "byteswap_utilities.h"





/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_llh_pos_byteswap(mip_filter_llh_pos *llh_pos)
//
//! @section DESCRIPTION
//! Byteswap a FILTER LLH Position Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_llh_pos *llh_pos - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_llh_pos_byteswap(mip_filter_llh_pos *llh_pos)
{
 byteswap_inplace(&llh_pos->latitude,         sizeof(double));
 byteswap_inplace(&llh_pos->longitude,        sizeof(double));
 byteswap_inplace(&llh_pos->ellipsoid_height, sizeof(double));
 byteswap_inplace(&llh_pos->valid_flags,      sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_ned_velocity_byteswap(mip_filter_ned_velocity *ned_velocity)
//
//! @section DESCRIPTION
//! Byteswap a FILTER NED Velocity Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_ned_velocity *ned_velocity - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_ned_velocity_byteswap(mip_filter_ned_velocity *ned_velocity)
{
 byteswap_inplace(&ned_velocity->north,       sizeof(float));
 byteswap_inplace(&ned_velocity->east,        sizeof(float));
 byteswap_inplace(&ned_velocity->down,        sizeof(float));
 byteswap_inplace(&ned_velocity->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_attitude_quaternion_byteswap(mip_filter_attitude_quaternion *attitude_quaternion)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Quaternion Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_attitude_quaternion *attitude_quaternion - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_attitude_quaternion_byteswap(mip_filter_attitude_quaternion *attitude_quaternion)
{
 u8 i;

 for(i=0; i<4; i++)
  byteswap_inplace(&attitude_quaternion->q[i],       sizeof(float));
 
 byteswap_inplace(&attitude_quaternion->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_attitude_dcm_byteswap(mip_filter_attitude_dcm *attitude_dcm)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Direction Cosine Matrix (DCM) Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_attitude_dcm *attitude_dcm - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_attitude_dcm_byteswap(mip_filter_attitude_dcm *attitude_dcm)
{
 u8 i, j;

 for(i=0; i<3; i++)
 {
  for(j=0; j<3; j++)
   byteswap_inplace(&attitude_dcm->dcm[i][j], sizeof(float));
 }
 
 byteswap_inplace(&attitude_dcm->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_attitude_euler_angles_byteswap(mip_filter_attitude_euler_angles *attitude_euler_angles)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Euler Angle Attitude Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_attitude_euler_angles *attitude_euler_angles - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_attitude_euler_angles_byteswap(mip_filter_attitude_euler_angles *attitude_euler_angles)
{
 byteswap_inplace(&attitude_euler_angles->roll,        sizeof(float));
 byteswap_inplace(&attitude_euler_angles->pitch,       sizeof(float));
 byteswap_inplace(&attitude_euler_angles->yaw,         sizeof(float));
 byteswap_inplace(&attitude_euler_angles->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_gyro_bias_byteswap(mip_filter_gyro_bias *gyro_bias)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Gyro Bias Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_gyro_bias *gyro_bias - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_gyro_bias_byteswap(mip_filter_gyro_bias *gyro_bias)
{
 byteswap_inplace(&gyro_bias->x,           sizeof(float));
 byteswap_inplace(&gyro_bias->y,           sizeof(float));
 byteswap_inplace(&gyro_bias->z,           sizeof(float));
 byteswap_inplace(&gyro_bias->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_accel_bias_byteswap(mip_filter_accel_bias *accel_bias)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Accel Bias Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_accel_bias *accel_bias - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_accel_bias_byteswap(mip_filter_accel_bias *accel_bias)
{
 byteswap_inplace(&accel_bias->x,           sizeof(float));
 byteswap_inplace(&accel_bias->y,           sizeof(float));
 byteswap_inplace(&accel_bias->z,           sizeof(float));
 byteswap_inplace(&accel_bias->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_llh_pos_uncertainty_byteswap(mip_filter_llh_pos_uncertainty *llh_pos_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a FILTER LLH Position Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_llh_pos_uncertainty *llh_pos_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_llh_pos_uncertainty_byteswap(mip_filter_llh_pos_uncertainty *llh_pos_uncertainty)
{
 byteswap_inplace(&llh_pos_uncertainty->north,       sizeof(float));
 byteswap_inplace(&llh_pos_uncertainty->east,        sizeof(float));
 byteswap_inplace(&llh_pos_uncertainty->down,        sizeof(float));
 byteswap_inplace(&llh_pos_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_ned_vel_uncertainty_byteswap(mip_filter_ned_vel_uncertainty *ned_vel_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a FILTER NED Velocity Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_ned_vel_uncertainty *ned_vel_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_ned_vel_uncertainty_byteswap(mip_filter_ned_vel_uncertainty *ned_vel_uncertainty)
{
 byteswap_inplace(&ned_vel_uncertainty->north,       sizeof(float));
 byteswap_inplace(&ned_vel_uncertainty->east,        sizeof(float));
 byteswap_inplace(&ned_vel_uncertainty->down,        sizeof(float));
 byteswap_inplace(&ned_vel_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_euler_attitude_uncertainty_byteswap(mip_filter_euler_attitude_uncertainty *euler_attitude_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Euler Attitude Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_euler_attitude_uncertainty *euler_attitude_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_euler_attitude_uncertainty_byteswap(mip_filter_euler_attitude_uncertainty *euler_attitude_uncertainty)
{
 byteswap_inplace(&euler_attitude_uncertainty->roll,        sizeof(float));
 byteswap_inplace(&euler_attitude_uncertainty->pitch,       sizeof(float));
 byteswap_inplace(&euler_attitude_uncertainty->yaw,         sizeof(float));
 byteswap_inplace(&euler_attitude_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_gyro_bias_uncertainty_byteswap(mip_filter_gyro_bias_uncertainty *gyro_bias_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Gyro Bias Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_gyro_bias_uncertainty *gyro_bias_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_gyro_bias_uncertainty_byteswap(mip_filter_gyro_bias_uncertainty *gyro_bias_uncertainty)
{
 byteswap_inplace(&gyro_bias_uncertainty->x,           sizeof(float));
 byteswap_inplace(&gyro_bias_uncertainty->y,           sizeof(float));
 byteswap_inplace(&gyro_bias_uncertainty->z,           sizeof(float));
 byteswap_inplace(&gyro_bias_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_accel_bias_uncertainty_byteswap(mip_filter_accel_bias_uncertainty *accel_bias_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Accel Bias Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_accel_bias_uncertainty *accel_bias_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_accel_bias_uncertainty_byteswap(mip_filter_accel_bias_uncertainty *accel_bias_uncertainty)
{
 byteswap_inplace(&accel_bias_uncertainty->x,           sizeof(float));
 byteswap_inplace(&accel_bias_uncertainty->y,           sizeof(float));
 byteswap_inplace(&accel_bias_uncertainty->z,           sizeof(float));
 byteswap_inplace(&accel_bias_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_timestamp_byteswap(mip_filter_timestamp *timestamp)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Timestamp Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_timestamp *timestamp - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_timestamp_byteswap(mip_filter_timestamp *timestamp)
{
 byteswap_inplace(&timestamp->tow,         sizeof(double));
 byteswap_inplace(&timestamp->week_number, sizeof(u16));
 byteswap_inplace(&timestamp->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_status_byteswap(mip_filter_status *status)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Status Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_status *status - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_status_byteswap(mip_filter_status *status)
{
 byteswap_inplace(&status->filter_state,  sizeof(u16));
 byteswap_inplace(&status->dynamics_mode, sizeof(u16));
 byteswap_inplace(&status->status_flags,  sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_linear_acceleration_byteswap(mip_filter_linear_acceleration *acceleration)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Acceleration Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_linear_acceleration *acceleration - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_linear_acceleration_byteswap(mip_filter_linear_acceleration *acceleration)
{
 byteswap_inplace(&acceleration->x,           sizeof(float));
 byteswap_inplace(&acceleration->y,           sizeof(float));
 byteswap_inplace(&acceleration->z,           sizeof(float));
 byteswap_inplace(&acceleration->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_compensated_acceleration_byteswap(mip_filter_compensated_acceleration *acceleration)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Acceleration Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_compensated_acceleration *acceleration - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_compensated_acceleration_byteswap(mip_filter_compensated_acceleration *acceleration)
{
 byteswap_inplace(&acceleration->x,           sizeof(float));
 byteswap_inplace(&acceleration->y,           sizeof(float));
 byteswap_inplace(&acceleration->z,           sizeof(float));
 byteswap_inplace(&acceleration->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_gravity_vector_byteswap(mip_filter_gravity_vector *gravity_vector)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Gravity Vector Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_gravity_vector *gravity_vector - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_gravity_vector_byteswap(mip_filter_gravity_vector *gravity_vector)
{
 byteswap_inplace(&gravity_vector->x,           sizeof(float));
 byteswap_inplace(&gravity_vector->y,           sizeof(float));
 byteswap_inplace(&gravity_vector->z,           sizeof(float));
 byteswap_inplace(&gravity_vector->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_compensated_angular_rate_byteswap(mip_filter_compensated_angular_rate *angular_rate)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Angular Rate Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_angular_rate *angular_rate - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_compensated_angular_rate_byteswap(mip_filter_compensated_angular_rate *angular_rate)
{
 byteswap_inplace(&angular_rate->x,           sizeof(float));
 byteswap_inplace(&angular_rate->y,           sizeof(float));
 byteswap_inplace(&angular_rate->z,           sizeof(float));
 byteswap_inplace(&angular_rate->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_quaternion_attitude_uncertainty_byteswap(mip_filter_quaternion_attitude_uncertainty *quaternion_attitude_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Quaternion Attitude Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_quaternion_attitude_uncertainty *quaternion_attitude_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_quaternion_attitude_uncertainty_byteswap(mip_filter_quaternion_attitude_uncertainty *quaternion_attitude_uncertainty)
{
 byteswap_inplace(&quaternion_attitude_uncertainty->q0,          sizeof(float));
 byteswap_inplace(&quaternion_attitude_uncertainty->q1,          sizeof(float));
 byteswap_inplace(&quaternion_attitude_uncertainty->q2,          sizeof(float));
 byteswap_inplace(&quaternion_attitude_uncertainty->q3,          sizeof(float));
 byteswap_inplace(&quaternion_attitude_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_wgs84_gravity_mag_byteswap(mip_filter_wgs84_gravity_mag *wgs84_gravity_mag)
//
//! @section DESCRIPTION
//! Byteswap a FILTER WGS84 Gravity Magnitude Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_wgs84_gravity_mag *wgs84_gravity_mag - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_wgs84_gravity_mag_byteswap(mip_filter_wgs84_gravity_mag *wgs84_gravity_mag)
{
 byteswap_inplace(&wgs84_gravity_mag->magnitude,   sizeof(float));
 byteswap_inplace(&wgs84_gravity_mag->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_heading_update_state_byteswap(mip_filter_heading_update_state *heading_update_state)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Heading Update State Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_heading_update_state *heading_update_state - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_heading_update_state_byteswap(mip_filter_heading_update_state *heading_update_state)
{
 byteswap_inplace(&heading_update_state->heading,        sizeof(float));
 byteswap_inplace(&heading_update_state->heading_1sigma, sizeof(float));
 byteswap_inplace(&heading_update_state->source,         sizeof(u16));
 byteswap_inplace(&heading_update_state->valid_flags,    sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_magnetic_model_byteswap(mip_filter_magnetic_model *magnetic_model)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Magnetic Model Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_magnetic_model *magnetic_model - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_magnetic_model_byteswap(mip_filter_magnetic_model *magnetic_model)
{
 byteswap_inplace(&magnetic_model->intensity_north, sizeof(float));
 byteswap_inplace(&magnetic_model->intensity_east,  sizeof(float));
 byteswap_inplace(&magnetic_model->intensity_down,  sizeof(float));
 byteswap_inplace(&magnetic_model->inclination,     sizeof(float));
 byteswap_inplace(&magnetic_model->declination,     sizeof(float));
 byteswap_inplace(&magnetic_model->valid_flags,     sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_external_heading_update_with_time_byteswap(mip_filter_external_heading_with_time_command *external_heading_command);
//
//! @section DESCRIPTION
//! Byteswap a FILTER External Heading Update With Timestamp Command Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_magnetic_model *magnetic_model - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_external_heading_update_with_time_byteswap(mip_filter_external_heading_with_time_command *external_heading_command)
{
 byteswap_inplace(&external_heading_command->gps_tow, sizeof(double));
 byteswap_inplace(&external_heading_command->gps_week_number, sizeof(u16));
 byteswap_inplace(&external_heading_command->heading_angle_rads, sizeof(float));
 byteswap_inplace(&external_heading_command->heading_angle_sigma_rads, sizeof(float));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_accel_scale_factor_byteswap(mip_filter_accel_scale_factor_mip_field *accel_scale_factor)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Accel Scale Factor Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_accel_scale_factor_mip_field *accel_scale_factor - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_accel_scale_factor_byteswap(mip_filter_accel_scale_factor_mip_field *accel_scale_factor)
{
 byteswap_inplace(&accel_scale_factor->x, sizeof(float));
 byteswap_inplace(&accel_scale_factor->y, sizeof(float));
 byteswap_inplace(&accel_scale_factor->z, sizeof(float));
 byteswap_inplace(&accel_scale_factor->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_accel_scale_factor_uncertainty_byteswap(mip_filter_accel_scale_factor_uncertainty_mip_field *accel_scale_factor_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Accel Scale Factor Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_accel_scale_factor_uncertainty_mip_field *accel_scale_factor_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_accel_scale_factor_uncertainty_byteswap(mip_filter_accel_scale_factor_uncertainty_mip_field *accel_scale_factor_uncertainty)
{
 byteswap_inplace(&accel_scale_factor_uncertainty->x, sizeof(float));
 byteswap_inplace(&accel_scale_factor_uncertainty->y, sizeof(float));
 byteswap_inplace(&accel_scale_factor_uncertainty->z, sizeof(float));
 byteswap_inplace(&accel_scale_factor_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_gyro_scale_factor_byteswap(mip_filter_gyro_scale_factor_mip_field *gyro_scale_factor)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Gyro Scale Factor Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_gyro_scale_factor_mip_field *gyro_scale_factor - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_gyro_scale_factor_byteswap(mip_filter_gyro_scale_factor_mip_field *gyro_scale_factor)
{
 byteswap_inplace(&gyro_scale_factor->x, sizeof(float));
 byteswap_inplace(&gyro_scale_factor->y, sizeof(float));
 byteswap_inplace(&gyro_scale_factor->z, sizeof(float));
 byteswap_inplace(&gyro_scale_factor->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_gyro_scale_factor_uncertainty_byteswap(mip_filter_gyro_scale_factor_uncertainty_mip_field *gyro_scale_factor_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Gyro Scale Factor Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_gyro_scale_factor_uncertainty_mip_field *gyro_scale_factor_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_gyro_scale_factor_uncertainty_byteswap(mip_filter_gyro_scale_factor_uncertainty_mip_field *gyro_scale_factor_uncertainty)
{
 byteswap_inplace(&gyro_scale_factor_uncertainty->x, sizeof(float));
 byteswap_inplace(&gyro_scale_factor_uncertainty->y, sizeof(float));
 byteswap_inplace(&gyro_scale_factor_uncertainty->z, sizeof(float));
 byteswap_inplace(&gyro_scale_factor_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_mag_bias_byteswap(mip_filter_mag_bias_mip_field *mag_bias)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Mag Bias Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_mag_bias_mip_field *mag_bias - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_mag_bias_byteswap(mip_filter_mag_bias_mip_field *mag_bias)
{
 byteswap_inplace(&mag_bias->x, sizeof(float));
 byteswap_inplace(&mag_bias->y, sizeof(float));
 byteswap_inplace(&mag_bias->z, sizeof(float));
 byteswap_inplace(&mag_bias->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_mag_bias_uncertainty_byteswap(mip_filter_mag_bias_uncertainty_mip_field *mag_bias_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Mag Bias Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_mag_bias_uncertainty_mip_field *mag_bias_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_mag_bias_uncertainty_byteswap(mip_filter_mag_bias_uncertainty_mip_field *mag_bias_uncertainty)
{
 byteswap_inplace(&mag_bias_uncertainty->x, sizeof(float));
 byteswap_inplace(&mag_bias_uncertainty->y, sizeof(float));
 byteswap_inplace(&mag_bias_uncertainty->z, sizeof(float));
 byteswap_inplace(&mag_bias_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_compensated_mag_vector_byteswap(mip_filter_compensated_mag_vector_mip_field *compensated_mag_vector)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Compensated Mag Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_compensated_mag_vector_mip_field *compensated_mag_vector - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_compensated_mag_vector_byteswap(mip_filter_compensated_mag_vector_mip_field *compensated_mag_vector)
{
 byteswap_inplace(&compensated_mag_vector->x, sizeof(float));
 byteswap_inplace(&compensated_mag_vector->y, sizeof(float));
 byteswap_inplace(&compensated_mag_vector->z, sizeof(float));
 byteswap_inplace(&compensated_mag_vector->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_standard_atmosphere_byteswap(mip_filter_standard_atmosphere_mip_field *standard_atmosphere)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Standard Atmosphere Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_standard_atmosphere_mip_field *standard_atmosphere - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_standard_atmosphere_byteswap(mip_filter_standard_atmosphere_mip_field *standard_atmosphere)
{
 byteswap_inplace(&standard_atmosphere->geometric_altitude, sizeof(float));
 byteswap_inplace(&standard_atmosphere->geopotential_altitude, sizeof(float));
 byteswap_inplace(&standard_atmosphere->standard_temperature, sizeof(float));
 byteswap_inplace(&standard_atmosphere->standard_pressure, sizeof(float));
 byteswap_inplace(&standard_atmosphere->standard_density, sizeof(float));
 byteswap_inplace(&standard_atmosphere->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_pressure_altitude_byteswap(mip_filter_pressure_altitude_mip_field *pressure_altitude)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Pressure Altitude Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_pressure_altitude_mip_field *pressure_altitude - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_pressure_altitude_byteswap(mip_filter_pressure_altitude_mip_field *pressure_altitude)
{
 byteswap_inplace(&pressure_altitude->pressure_altitude, sizeof(float));
 byteswap_inplace(&pressure_altitude->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_density_altitude_byteswap(mip_filter_density_altitude_mip_field *density_altitude)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Density Altitude Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_density_altitude_mip_field *density_altitude - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_density_altitude_byteswap(mip_filter_density_altitude_mip_field *density_altitude)
{
 byteswap_inplace(&density_altitude->density_altitude, sizeof(float));
 byteswap_inplace(&density_altitude->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_antenna_offset_correction_byteswap(mip_filter_antenna_offset_correction_mip_field *antenna_offset_correction)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Antenna Offset Correction Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_antenna_offset_correction_mip_field *antenna_offset_correction - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_antenna_offset_correction_byteswap(mip_filter_antenna_offset_correction_mip_field *antenna_offset_correction)
{
 byteswap_inplace(&antenna_offset_correction->x, sizeof(float));
 byteswap_inplace(&antenna_offset_correction->y, sizeof(float));
 byteswap_inplace(&antenna_offset_correction->z, sizeof(float));
 byteswap_inplace(&antenna_offset_correction->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_filter_antenna_offset_correction_uncertainty_byteswap(mip_filter_antenna_offset_correction_uncertainty_mip_field *antenna_offset_correction_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a FILTER Antenna Offset Correction Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_filter_antenna_offset_correction_uncertainty_mip_field *antenna_offset_correction_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_filter_antenna_offset_correction_uncertainty_byteswap(mip_filter_antenna_offset_correction_uncertainty_mip_field *antenna_offset_correction_uncertainty)
{
 byteswap_inplace(&antenna_offset_correction_uncertainty->x, sizeof(float));
 byteswap_inplace(&antenna_offset_correction_uncertainty->y, sizeof(float));
 byteswap_inplace(&antenna_offset_correction_uncertainty->z, sizeof(float));
 byteswap_inplace(&antenna_offset_correction_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_reset_filter(mip_interface *device_interface)
//
//! @section DESCRIPTION
//! Reset the Kalman Filter.  
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

u16 mip_filter_reset_filter(mip_interface *device_interface)
{
 return mip_interface_send_command(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_RESET_FILTER, 
                                   NULL, 0, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_set_init_attitude(mip_interface *device_interface, float euler_angles[3])
//
//! @section DESCRIPTION
//! Initialize the Kalman Filter with the provided Euler angles.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] float euler_angles[3]           - The Euler angles in radians.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Order of angles is [roll, pitch, yaw] in radians.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_set_init_attitude(mip_interface *device_interface, float euler_angles[3])
{
 u8 i;
 float local_angles[3];
 
 //Copy the angles to a local buffer
 memcpy(local_angles, euler_angles, sizeof(float)*3);
 
 //Byteswap the angles if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  for(i=0; i<3; i++)
   byteswap_inplace(&local_angles[i], sizeof(float));
 }   


 return mip_interface_send_command(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_SET_INITIAL_ATTITUDE, 
                                   (u8*)local_angles, sizeof(float)*3, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_set_init_heading(mip_interface *device_interface, float heading)
//
//! @section DESCRIPTION
//! Initialize the Kalman Filter with the provided true heading angle.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] float heading                   - The true heading angle in radians.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Roll and Pitch will be calculated by the device using the accelerometers.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_set_init_heading(mip_interface *device_interface, float heading)
{
 float local_heading;
 
 //Copy the angles to a local buffer
 memcpy(&local_heading, &heading, sizeof(float));
 
 //Byteswap the angles if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  byteswap_inplace(&local_heading, sizeof(float));
 }   


 return mip_interface_send_command(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_SET_INITIAL_HEADING, 
                                   (u8*)&local_heading, sizeof(float), 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_set_init_attitude_from_ahrs(mip_interface *device_interface, float declination)
//
//! @section DESCRIPTION
//! Initialize the Kalman Filter from the AHRS algorithm output, taking into account the magnetic declination angle povided.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] float declination               - The local magnetic declination angle in radians.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! The output of the internal AHRS orientation algorithm will be used to initialize\n
//! the filter, taking into account the user-provided magnetic declination angle.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_set_init_attitude_from_ahrs(mip_interface *device_interface, float declination)
{
 float local_declination;
 
 //Copy the angles to a local buffer
 memcpy(&local_declination, &declination, sizeof(float));
 
 //Byteswap the angles if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  byteswap_inplace(&local_declination, sizeof(float));
 }   


 return mip_interface_send_command(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_SET_INITIAL_HEADING_FROM_AHRS, 
                                   (u8*)&local_declination, sizeof(float), 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);

}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_vehicle_dynamics_mode(mip_interface *device_interface, u8 function_selector, u8 *dynamics_mode)
//
//! @section DESCRIPTION
//! Set or read the filter's vehicle dynamics mode settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface        - The device interface.
//! @param [in] u8 function_selector                   - Selects which function to perform.
//! @param [in,out] u8 *dynamics_mode                   - The vehicle dynamics mode. (used to set or get depending on \c function_selector)
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
//! Please reference the device DCP for valid \c dynamics_mode values.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_vehicle_dynamics_mode(mip_interface *device_interface, u8 function_selector, u8 *dynamics_mode)
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
  command_data[1] = *dynamics_mode;
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_VEHICLE_DYNAMICS_MODE, command_data, 
                                                        2, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_VEHICLE_DYNAMICS_MODE) &&
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
//! u16 mip_filter_sensor2vehicle_tranformation(mip_interface *device_interface, u8 function_selector, float euler_angles[3])
//
//! @section DESCRIPTION
//! Set or read the filter's sensor to vehicle transformation settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface        - The device interface.
//! @param [in] u8 function_selector                   - Selects which function to perform.
//! @param [in,out] float euler_angles[3]              - The sensor to vehicle transformation expressed as Euler angles\n
//!                                                      in radians. (used to set or get depending on \c function_selector)
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
//! The angles are [roll, pitch, yaw] in radians from the sensor frame to the vehicle frame.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_sensor2vehicle_tranformation(mip_interface *device_interface, u8 function_selector, float euler_angles[3])
{
 u8 i;
 u8 *response_data      = NULL;
 u16 response_data_size = 0;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*3] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  float_ptr = (float*)&command_data[1];

  //Copy the angles to a local buffer
  memcpy(float_ptr, euler_angles, sizeof(float)*3);
 
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<3; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }  
 }
 

                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_SENSOR2VEHICLE_TRANSFORMATION, command_data, 
                                                        sizeof(u8) + sizeof(float)*3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_SENSOR2VEHICLE_TRANSFORMATION) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*3))
  {
   memcpy(euler_angles, response_data + sizeof(mip_field_header), sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
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
//! u16 mip_filter_sensor2vehicle_offset(mip_interface *device_interface, u8 function_selector, float offset[3])
//
//! @section DESCRIPTION
//! Set or read the filter's sensor to vehicle frame offset settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] float offset                - The sensor to vehicle frame offset in meters. (used to set or get depending on \c function_selector)
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
//! The offset is [x, y, z] in meters from the sensor frame to the vehicle frame, expressed in the sensor frame.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_sensor2vehicle_offset(mip_interface *device_interface, u8 function_selector, float offset[3])
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*3]={0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  float_ptr = (float*)&command_data[1];
  
  //Copy the angles to a local buffer
  memcpy(float_ptr, offset, sizeof(float)*3);
 
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<3; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }   
 }
 
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_SENSOR2VEHICLE_OFFSET, command_data, 
                                                        sizeof(u8) + sizeof(float)*3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                                                     
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_SENSOR2VEHICLE_OFFSET) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*3))
  {
   memcpy(offset, response_data + sizeof(mip_field_header), sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
     byteswap_inplace(&offset[i], sizeof(float));
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
//! u16 mip_filter_antenna_offset(mip_interface *device_interface, u8 function_selector, float offset[3])
//
//! @section DESCRIPTION
//! Set or read the filter's antenna offset settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] float offset                - The offset of the antenna with-respect-to the sensor in meters. (used to set or get depending on \c function_selector)
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
//! The offset is [x, y, z] in meters from the sensor to the antenna, expressed in the sensor frame.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_antenna_offset(mip_interface *device_interface, u8 function_selector, float offset[3])
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*3]={0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;

 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  float_ptr       = (float*)&command_data[1];
 
  //Copy the angles to a local buffer
  memcpy(float_ptr, offset, sizeof(float)*3);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  { 
   for(i=0; i<3; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }
 }   
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_ANTENNA_OFFSET, command_data, 
                                                        sizeof(u8) + sizeof(float)*3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                   
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_ANTENNA_OFFSET) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*3))
  {
   memcpy(offset, response_data + sizeof(mip_field_header), sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
     byteswap_inplace(&offset[i], sizeof(float));
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
//! u16 mip_filter_gps_source(mip_interface *device_interface, u8 function_selector, u8 *gps_source)
//
//! @section DESCRIPTION
//! Set or read the filter's GPS source settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *gps_source              - The source of GPS updates to the filter. (used to set or get depending on \c function_selector)
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
//! Please consult the device DCP for valid GPS source values.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_gps_source(mip_interface *device_interface, u8 function_selector, u8 *gps_source)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[2];
 mip_field_header *field_header_ptr;
 
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[1] = *gps_source;
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_GPS_SOURCE_CONTROL, command_data, 
                                                        2, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_GPS_SOURCE_CONTROL) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   memcpy(gps_source, response_data + sizeof(mip_field_header), sizeof(u8));
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_external_gps_update(mip_interface *device_interface, mip_filter_external_gps_update_command *command)
//
//! @section DESCRIPTION
//! Provide an external GPS update to the filter.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface              - The device interface.
//! @param [in] mip_filter_external_gps_update_command *command - The external GPS update command.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//!
//! The device will only accept this command when external GPS is selected\n
//! as the GPS source.  Please consult the device DCP for how to configure this setting.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_external_gps_update(mip_interface *device_interface, mip_filter_external_gps_update_command *command)
{
 u8 i;
 mip_filter_external_gps_update_command local_command;
 
 //Copy the command to the local buffer
 memcpy(&local_command, command, sizeof(mip_filter_external_gps_update_command));
 
 //Byteswap the angles if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  byteswap_inplace(&local_command.tow,            sizeof(double));
  byteswap_inplace(&local_command.week_number,    sizeof(u16));

  for(i=0; i<3; i++)
  {
   byteswap_inplace(&local_command.pos[i],        sizeof(double));
   byteswap_inplace(&local_command.vel[i],        sizeof(float));
   byteswap_inplace(&local_command.pos_1sigma[i], sizeof(float));
   byteswap_inplace(&local_command.vel_1sigma[i], sizeof(float));
  }
 }   
 
 return mip_interface_send_command(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_EXTERNAL_GPS_UPDATE, 
                                   (u8*)&local_command, sizeof(mip_filter_external_gps_update_command), 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_external_heading_update(mip_interface *device_interface, mip_filter_external_heading_update_command *command)
//
//! @section DESCRIPTION
//! Provide an external heading update to the filter.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface                  - The device interface.
//! @param [in] mip_filter_external_heading_update_command *command - The external heading update command.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//!
//! The device will only accept this command when external heading is selected\n
//! as the heading update source.  Please consult the device DCP for how to configure this setting.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_external_heading_update(mip_interface *device_interface, mip_filter_external_heading_update_command *command)
{
 mip_filter_external_heading_update_command local_command;
 
 //Copy the command to the local buffer
 memcpy(&local_command, command, sizeof(mip_filter_external_heading_update_command));

 //Byteswap the angles if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  byteswap_inplace(&local_command.heading_angle,           sizeof(float));
  byteswap_inplace(&local_command.heading_angle_1sigma,    sizeof(float));   
 }   
 

 return mip_interface_send_command(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_EXTERNAL_HEADING_UPDATE, 
                                   (u8*)&local_command, sizeof(mip_filter_external_heading_update_command), 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_heading_source(mip_interface *device_interface, u8 function_selector, u8 *heading_source)
//
//! @section DESCRIPTION
//! Set or read the filter's heading update source settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *heading_source          - The source of heading updates to the filter. (used to set or get depending on \c function_selector)
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
//! Please consult the device DCP for valid heading update source values.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_heading_source(mip_interface *device_interface, u8 function_selector, u8 *heading_source)
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
  command_data[1] = *heading_source;
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_HEADING_UPDATE_CONTROL, command_data, 
                                                        2, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_HEADING_UPDATE_CONTROL) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   memcpy(heading_source, response_data + sizeof(mip_field_header), sizeof(u8));
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_auto_initialization(mip_interface *device_interface, u8 function_selector, u8 *enable)
//
//! @section DESCRIPTION
//! Set or read the filter's auto-initialization setting.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *enable                  - The auto-initialization setting. (used to set or get depending on \c function_selector)
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
//! Please consult the device DCP for valid auto-initialization values.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_auto_initialization(mip_interface *device_interface, u8 function_selector, u8 *enable)
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
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_AUTOINIT_CONTROL, command_data, 
                                                        2, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_AUTOINIT_CONTROL) &&
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
//! u16 mip_filter_accel_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3])
//
//! @section DESCRIPTION
//! Set or read the filter's accelerometer process/measurement noise values.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] float noise_1sigma[3]       - The accelerometer 1-sigma process/measurement noise values in m/(s^2). (used to set or get depending on \c function_selector)
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
//! The accelerometer 1-sigma process/measurement noise values are ordered [x, y, z] in the\n
//! sensor frame in m/(s^2).  These values are used as process/measurement noise and allow the user to tune\n
//! the behavior of the Kalman filter for their particular application.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_accel_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3])
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*3] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  float_ptr       = (float*)&command_data[1];
 
  //Copy the angles to a local buffer
  memcpy(float_ptr, noise_1sigma, sizeof(float)*3);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<3; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }   
 }
  
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_ACCEL_NOISE, command_data, 
                                                        sizeof(u8) + sizeof(float)*3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_ACCEL_NOISE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*3))
  {
   memcpy(noise_1sigma, response_data + sizeof(mip_field_header), sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
     byteswap_inplace(&noise_1sigma[i], sizeof(float));
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
//! u16 mip_filter_gyro_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3])
//
//! @section DESCRIPTION
//! Set or read the filter's gyroscope process noise values.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] float noise_1sigma[3]       - The gyroscope 1-sigma process noise values in rad/sec. (used to set or get depending on \c function_selector)
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
//! The gyroscope 1-sigma process noise values are ordered [x, y, z] in the\n
//! sensor frame in rad/sec.  These values are used as process noise and allow the user to tune\n
//! the behavior of the Kalman filter for their particular application.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_gyro_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3])
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*3] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  float_ptr       = (float*)&command_data[1];
 
  //Copy the angles to a local buffer
  memcpy(float_ptr, noise_1sigma, sizeof(float)*3);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<3; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }    
 }
                                  
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_GYRO_NOISE, command_data, 
                                                        sizeof(u8) + sizeof(float)*3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_GYRO_NOISE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*3))
  {
   memcpy(noise_1sigma, response_data + sizeof(mip_field_header), sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
     byteswap_inplace(&noise_1sigma[i], sizeof(float));
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
//! u16 mip_filter_gyro_bias_model(mip_interface *device_interface, u8 function_selector, float bias_beta[3], float bias_noise_1sigma[3])
//
//! @section DESCRIPTION
//! Set or read the filter's gyroscope Guass-Markov bias model values.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] float bias_beta[3]          - The gyroscope bias model time constants in 1/sec. (used to set or get depending on \c function_selector)
//! @param [in,out] float bias_noise_1sigma[3]  - The gyroscope bias model 1-sigma white noise values in rad/sec. (used to set or get depending on \c function_selector)
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
//! The gyroscope Gauss-Markov bias model time constant values are ordered [x, y, z] in the\n
//! sensor frame in units of 1/sec.  The gyro Gauss-Markov bias model 1-sigma white noise values are ordered [x, y, z] in the\n
//! sensor frame in units of rad/sec.  These values are used as process noise to the gyro bias model and allow the user to tune\n
//! the behavior of the Kalman filter for their particular application.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_gyro_bias_model(mip_interface *device_interface, u8 function_selector, float bias_beta[3], float bias_noise_1sigma[3])
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*6] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  float_ptr       = (float*)&command_data[1];
  
  //Copy the parameters to a local buffer
  memcpy(float_ptr,   bias_beta,         sizeof(float)*3);
  memcpy(float_ptr+3, bias_noise_1sigma, sizeof(float)*3);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<6; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }   
 }
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_GYRO_BIAS_MODEL, command_data, 
                                                        sizeof(u8) + sizeof(float)*6, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_GYRO_BIAS_MODEL) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*6))
  {
   float_ptr = (float*)(response_data + sizeof(mip_field_header));
  
   memcpy(bias_beta,         float_ptr,   sizeof(float)*3);
   memcpy(bias_noise_1sigma, float_ptr+3, sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
    {
     byteswap_inplace(&bias_beta[i],         sizeof(float));
     byteswap_inplace(&bias_noise_1sigma[i], sizeof(float));
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
//! u16 mip_filter_accel_bias_model(mip_interface *device_interface, u8 function_selector, float bias_beta[3], float bias_noise_1sigma[3])
//
//! @section DESCRIPTION
//! Set or read the filter's accelerometer Guass-Markov bias model values.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] float bias_beta[3]          - The accelerometer bias model time constants in 1/sec. (used to set or get depending on \c function_selector)
//! @param [in,out] float bias_noise_1sigma[3]  - The accelerometer bias model 1-sigma white noise values in rad/sec. (used to set or get depending on \c function_selector)
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
//! The accelerometer Gauss-Markov bias model time constant values are ordered [x, y, z] in the\n
//! sensor frame in units of 1/sec.  The accel Gauss-Markov bias model 1-sigma white noise values are ordered [x, y, z] in the\n
//! sensor frame in units of rad/sec.  These values are used as process noise to the accel bias model and allow the user to tune\n
//! the behavior of the Kalman filter for their particular application.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_accel_bias_model(mip_interface *device_interface, u8 function_selector, float bias_beta[3], float bias_noise_1sigma[3])
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*6] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  float_ptr       = (float*)&command_data[1];
  
  //Copy the parameters to a local buffer
  memcpy(float_ptr,   bias_beta,         sizeof(float)*3);
  memcpy(float_ptr+3, bias_noise_1sigma, sizeof(float)*3);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<6; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }   
 }
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_ACCEL_BIAS_MODEL, command_data, 
                                                        sizeof(u8) + sizeof(float)*6, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_ACCEL_BIAS_MODEL) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*6))
  {
   float_ptr = (float*)(response_data + sizeof(mip_field_header));
  
   memcpy(bias_beta,         float_ptr,   sizeof(float)*3);
   memcpy(bias_noise_1sigma, float_ptr+3, sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
    {
     byteswap_inplace(&bias_beta[i],         sizeof(float));
     byteswap_inplace(&bias_noise_1sigma[i], sizeof(float));
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
//! u16 mip_filter_zero_velocity_update_control(mip_interface *device_interface, u8 function_selector, mip_filter_zero_update_command_velocity *zero_velocity_control)
//
//! @section DESCRIPTION
//! Set or read the filter's zero-velocity update threshold.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 enable                   - Writes/Reads enable bit state.
//! @param [in,out] float threshold             - The GPS velocity magnitude below which the device is considered to be stationary.
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
//! Set or read the scalar magnitude of the GPS velocity below which the device
//! is considered to be stationary.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_zero_velocity_update_control(mip_interface *device_interface, u8 function_selector,  mip_filter_zero_update_command *zero_velocity_control)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(mip_filter_zero_update_command)] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[1] = zero_velocity_control->enable; 
  float_ptr       = (float*)&command_data[2];
  
  //Copy the parameters to a local buffer
  memcpy(float_ptr, &zero_velocity_control->threshold, sizeof(float));
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   byteswap_inplace(float_ptr, sizeof(float));
  }   
 }
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_ZERO_VELOCITY_UPDATE, command_data, 
                                                        sizeof(u8) + sizeof(mip_filter_zero_update_command), &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_ZERO_VELOCITY_UPDATE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(mip_filter_zero_update_command)))
  {
   zero_velocity_control->enable = *(response_data + sizeof(mip_field_header));
  
   float_ptr = (float*)(response_data + sizeof(mip_field_header) + sizeof(u8));
  
   memcpy(&zero_velocity_control->threshold, float_ptr, sizeof(float));
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(&zero_velocity_control->threshold, sizeof(float));
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
//! u16 mip_filter_zero_angular_rate_update_control(mip_interface *device_interface, u8 function_selector, mip_filter_zero_update_command *zero_angular_rate_control)
//
//! @section DESCRIPTION
//! Set or read the filter's gyro zero-angular_rate update threshold.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 enable                   - Writes/Reads enable bit state.
//! @param [in,out] float threshold             - The gyro magnitude below which the device is considered to be stationary.
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
//! Set or read the scalar magnitude of the gyro below which the device
//! is considered to be stationary.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_zero_angular_rate_update_control(mip_interface *device_interface, u8 function_selector, mip_filter_zero_update_command *zero_angular_rate_control)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(mip_filter_zero_update_command)] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[1] = zero_angular_rate_control->enable; 
  float_ptr       = (float*)&command_data[2];
  
  //Copy the parameters to a local buffer
  memcpy(float_ptr, &zero_angular_rate_control->threshold, sizeof(float));
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   byteswap_inplace(float_ptr, sizeof(float));
  }   
 }
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_ANGULAR_RATE_ZERO_UPDATE, command_data, 
                                                        sizeof(u8) + sizeof(mip_filter_zero_update_command), &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_ZERO_ANGULAR_RATE_UPDATE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(mip_filter_zero_update_command)))
  {
   zero_angular_rate_control->enable = *(response_data + sizeof(mip_field_header));
  
   float_ptr = (float*)(response_data + sizeof(mip_field_header) + sizeof(u8));
  
   memcpy(&zero_angular_rate_control->threshold, float_ptr, sizeof(float));
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(&zero_angular_rate_control->threshold, sizeof(float));
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
//! u16 mip_filter_tare_orientation(mip_interface *device_interface, u8 function_selector, u8 tare_bitfield)
//
//! @section DESCRIPTION
//! Set orientation relative to NED frame as the current sensor to vehicle frame.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in] u8 tare_bitfield                - Tare bitfield (where 0x01 bit represents the roll axis, 0x02 represents the pitch axis, 0x04 represents the yaw axis).
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c The tare_bitfield value is ignored for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \n Possible \c tare_bitfield values:\n
//!    \li 0x01 - Tare the roll axis
//!    \li 0x02 - Tare the pitch axis
//!    \li 0x03 - Tare the roll and pitch axes
//!    \li 0x04 - Tare the yaw axis
//!    \li 0x05 - Tare the roll and yaw axes
//!    \li 0x06 - Tare the pitch and yaw axes
//!    \li 0x07 - Tare the roll, pitch and yaw axes
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_tare_orientation(mip_interface *device_interface, u8 function_selector, u8 tare_bitfield)
{
 u16 return_code;
 u8  command_data[2] = {0};
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[1] = tare_bitfield;
 }
 
 return_code = mip_interface_send_command(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_TARE_ORIENTATION, command_data, 
                                                        2, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_commanded_zero_velocity_update(mip_interface *device_interface)
//
//! @section DESCRIPTION
//!   
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
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_commanded_zero_velocity_update(mip_interface *device_interface)
{
 u16 return_code;
 
 return_code = mip_interface_send_command(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_COMMANDED_ZUPT, NULL, 0, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 return return_code;
}

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_commanded_zero_angular_rate_update(mip_interface *device_interface)
//
//! @section DESCRIPTION
//!   
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
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_commanded_zero_angular_rate_update(mip_interface *device_interface)
{
 u16 return_code;
 
 return_code = mip_interface_send_command(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_COMMANDED_ZERO_ANGULAR_RATE_UPDATE, NULL, 0, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_external_heading_update_with_time(mip_interface *device_interface, mip_filter_external_heading_with_time_command *heading_with_time_command)
//
//! @section DESCRIPTION
//! Set or read the AHRS signal conditioning settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface        - The device interface.
//! @param [in] u8 function_selector                   - Selects which function to perform.
//! @param [in,out] mip_filter_external_heading_with_time_command *heading_with_time_command - The external heading update with timestamp command structure. (used to set or get depending on \c function_selector)
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
//! If the FILTER datastream is available, the values remain unaffected.
//! Please reference the device DCP for further information.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_external_heading_update_with_time(mip_interface *device_interface, mip_filter_external_heading_with_time_command *heading_with_time_command)
{
 u16 return_code;
 u8  command_data[sizeof(mip_filter_external_heading_with_time_command)] = {0};
 
 memcpy(command_data, heading_with_time_command, sizeof(mip_filter_external_heading_with_time_command));
  
 //Byteswap the baudrate if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  mip_filter_external_heading_update_with_time_byteswap((mip_filter_external_heading_with_time_command*)command_data);
 }   
 
 return_code = mip_interface_send_command(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_EXTERNAL_HEADING_TIMESTAMP, command_data, 
                                          sizeof(mip_filter_external_heading_with_time_command), 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_mag_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3])
//
//! @section DESCRIPTION
//! Set or read the filter's magnetometer measurement noise values.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] float noise_1sigma[3]       - The magnetometer 1-sigma measurement noise values in Gauss. (used to set or get depending on \c function_selector)
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
//! The magnetometer 1-sigma measurement noise values are ordered [x, y, z] in the\n
//! sensor frame in Gauss.  These values are used as measurement noise and allow the user to tune\n
//! the behavior of the Kalman filter for their particular application.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_mag_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3])
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*3] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  float_ptr = (float*)&command_data[1];
 
  //Copy the angles to a local buffer
  memcpy(float_ptr, noise_1sigma, sizeof(float)*3);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<3; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }   
 }
  
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_MAG_NOISE, command_data, 
                                                        sizeof(u8) + sizeof(float)*3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_MAG_NOISE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*3))
  {
   memcpy(noise_1sigma, response_data + sizeof(mip_field_header), sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
     byteswap_inplace(&noise_1sigma[i], sizeof(float));
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
//! u16 mip_filter_reference_position(mip_interface *device_interface,  u8 function_selector, u8 *reference_enable, double reference_position[3])
//
//! @section DESCRIPTION
//! Set or read the filter's reference position [Latitude (deg), Longitude (deg), and Height (m)]  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface  - The device interface.
//! @param [in] u8 function_selector             - Selects which function to perform.
//! @param [in,out] u8 *reference_enable         - Pointer to the enable reference position flag. (used to set or get depending on \c function_selector)
//! @param [in,out] double reference_position[3] - The reference position [Latitude (deg), Longitude (deg), and Height (m)]. (used to set or get depending on \c function_selector)
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
//! The reference position is used by AHRS devices to determine the WMM solution (declination angle)\n
//! and WGS84 gravity magnitude.  GPS/INS devices will NACK this command.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_reference_position(mip_interface *device_interface,  u8 function_selector, u8 *reference_enable, double reference_position[3])
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(u8) + sizeof(double)*3] = {0};
 mip_field_header *field_header_ptr;
 double *double_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  //Copy the enable flag to the local buffer
  command_data[1] = *reference_enable;
	 
  double_ptr = (double*)&command_data[2];
 
  //Copy the reference position to the local buffer
  memcpy(double_ptr, reference_position, sizeof(double)*3);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<3; i++)
    byteswap_inplace(&double_ptr[i], sizeof(double));
  }   
 }
  
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_REFERENCE_POSITION, command_data, 
                                                        sizeof(u8) + sizeof(u8) + sizeof(double)*3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_REFERENCE_POSITION) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8) + sizeof(double)*3))
  {
   memcpy(reference_enable,   response_data + sizeof(mip_field_header), sizeof(u8));
   memcpy(reference_position, response_data + sizeof(mip_field_header) + sizeof(u8), sizeof(double)*3);
   
   //Byteswap the position if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
     byteswap_inplace(&reference_position[i], sizeof(double));
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
//! u16 mip_filter_estimation_control(mip_interface *device_interface, u8 function_selector, u16 *estimation_control)
//
//! @section DESCRIPTION
//! Set or read the filter's estimation control flags.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u16 *estimation_control     - The estimation control value. (used to set or get depending on \c function_selector)
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
//! Please consult the device DCP for valid estimation control values
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_estimation_control(mip_interface *device_interface, u8 function_selector, u16 *estimation_control)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[3] = {0};
 mip_field_header *field_header_ptr;
 u16 *short_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;

 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  short_ptr       = (u16*)&command_data[1];
  *short_ptr      = *estimation_control;

  //Byteswap the bias control value if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   byteswap_inplace(short_ptr, sizeof(u16));
  }   
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_ESTIMATION_CONTROL, command_data, 
                                                        3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_ESTIMATION_CONTROL) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   memcpy(estimation_control, response_data + sizeof(mip_field_header), sizeof(u16));
   
   //Byteswap the bias control value if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(estimation_control, sizeof(u16));
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
//! u16 mip_filter_enable_measurement(mip_interface *device_interface, u8 function_selector, u16 *measurement_enable)
//
//! @section DESCRIPTION
//! Set or read the filter's measurement enable flags.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u16 *measurement_enable     - The measurement enable value. (used to set or get depending on \c function_selector)
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
//! Please consult the device DCP for valid measurement enable values
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_enable_measurement(mip_interface *device_interface, u8 function_selector, u16 *measurement_enable)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[3] = {0};
 mip_field_header *field_header_ptr;
 u16 *short_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;

 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  short_ptr       = (u16*)&command_data[1];
  *short_ptr      = *measurement_enable;

  //Byteswap the bias control value if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   byteswap_inplace(short_ptr, sizeof(u16));
  }   
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_ENABLE_MEASUREMENT, command_data, 
                                                        3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_ENABLE_MEASUREMENT) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   memcpy(measurement_enable, response_data + sizeof(mip_field_header), sizeof(u16));
   
   //Byteswap the bias control value if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(measurement_enable, sizeof(u16));
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
//! u16 mip_filter_declination_source(mip_interface *device_interface, u8 function_selector, u8 *declination_source)
//
//! @section DESCRIPTION
//! Set or read the filter's declination source settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *declination_source      - The source of declination provided to the filter. (used to set or get depending on \c function_selector)
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
//! Please consult the device DCP for valid declination source values.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_declination_source(mip_interface *device_interface, u8 function_selector, u8 *declination_source)
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
  command_data[1] = *declination_source;
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_DECLINATION_SOURCE, command_data, 
                                                        2, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_DECLINATION_SOURCE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   memcpy(declination_source, response_data + sizeof(mip_field_header), sizeof(u8));
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_filter_accel_magnitude_error_adaptive_measurement(mip_interface *device_interface, u8 function_selector, mip_filter_accel_magnitude_error_adaptive_measurement_command *params)
//
//! @section DESCRIPTION
//! Set or read the filter's accelerometer magnitude error adaptive measurement settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] mip_filter_accel_magnitude_error_adaptive_measurement_command *params - The accelerometer magnitude error adaptive measurement settings.
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
//! Set or read the accelerometer magnitude error adaptive measurement settings.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_accel_magnitude_error_adaptive_measurement(mip_interface *device_interface, u8 function_selector, mip_filter_accel_magnitude_error_adaptive_measurement_command *params)
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(mip_filter_accel_magnitude_error_adaptive_measurement_command)] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[1] = params->enable; 
  float_ptr       = (float*)&command_data[2];
 
  //Copy the parameters to a local buffer
  memcpy(float_ptr, &params->low_pass_cutoff, sizeof(float)*6);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<6; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }   
 }
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, command_data, 
                                                        sizeof(u8) + sizeof(mip_filter_accel_magnitude_error_adaptive_measurement_command), &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(mip_filter_accel_magnitude_error_adaptive_measurement_command)))
  {
   params->enable = *(response_data + sizeof(mip_field_header));
  
   float_ptr = (float*)(response_data + sizeof(mip_field_header) + sizeof(u8));

   memcpy(&params->low_pass_cutoff, float_ptr, sizeof(float)*6);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(&params->low_pass_cutoff,   sizeof(float));
    byteswap_inplace(&params->low_limit,         sizeof(float));
    byteswap_inplace(&params->high_limit,        sizeof(float));
    byteswap_inplace(&params->low_limit_1sigma,  sizeof(float));
    byteswap_inplace(&params->high_limit_1sigma, sizeof(float));
    byteswap_inplace(&params->min_1sigma,        sizeof(float));
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
//! u16 mip_filter_mag_magnitude_error_adaptive_measurement(mip_interface *device_interface, u8 function_selector, mip_filter_magnetometer_magnitude_error_adaptive_measurement_command *params)
//
//! @section DESCRIPTION
//! Set or read the filter's magnetometer magnitude error adaptive measurement settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] mip_filter_magnetometer_magnitude_error_adaptive_measurement_command *params - The magnetometer magnitude error adaptive measurement settings.
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
//! Set or read the magnetometer magnitude error adaptive measurement settings.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_mag_magnitude_error_adaptive_measurement(mip_interface *device_interface, u8 function_selector, mip_filter_magnetometer_magnitude_error_adaptive_measurement_command *params)
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(mip_filter_magnetometer_magnitude_error_adaptive_measurement_command)] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[1] = params->enable; 
  float_ptr       = (float*)&command_data[2];
 
  //Copy the parameters to a local buffer
  memcpy(float_ptr, &params->low_pass_cutoff, sizeof(float)*6);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<6; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }   
 }
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, command_data, 
                                                        sizeof(u8) + sizeof(mip_filter_magnetometer_magnitude_error_adaptive_measurement_command), &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(mip_filter_magnetometer_magnitude_error_adaptive_measurement_command)))
  {
   params->enable = *(response_data + sizeof(mip_field_header));
  
   float_ptr = (float*)(response_data + sizeof(mip_field_header) + sizeof(u8));

   memcpy(&params->low_pass_cutoff, float_ptr, sizeof(float)*6);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(&params->low_pass_cutoff,   sizeof(float));
    byteswap_inplace(&params->low_limit,         sizeof(float));
    byteswap_inplace(&params->high_limit,        sizeof(float));
    byteswap_inplace(&params->low_limit_1sigma,  sizeof(float));
    byteswap_inplace(&params->high_limit_1sigma, sizeof(float));
    byteswap_inplace(&params->min_1sigma,        sizeof(float));
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
//! u16 mip_filter_mag_dip_angle_error_adaptive_measurement(mip_interface *device_interface, u8 function_selector, mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command *params)
//
//! @section DESCRIPTION
//! Set or read the filter's magnetometer dip angle error adaptive measurement settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command *params - The magnetometer dip angle error adaptive measurement settings.
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
//! Set or read the magnetometer dip angle error adaptive measurement settings.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_filter_mag_dip_angle_error_adaptive_measurement(mip_interface *device_interface, u8 function_selector, mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command *params)
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command)] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[1] = params->enable; 
  float_ptr       = (float*)&command_data[2];
 
  //Copy the parameters to a local buffer
  memcpy(float_ptr, &params->low_pass_cutoff, sizeof(float)*4);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<4; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }   
 }
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_FILTER_COMMAND_SET, MIP_FILTER_CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, command_data, 
                                                        sizeof(u8) + sizeof(mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command), &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_FILTER_REPLY_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command)))
  {
   params->enable = *(response_data + sizeof(mip_field_header));
  
   float_ptr = (float*)(response_data + sizeof(mip_field_header) + sizeof(u8));

   memcpy(&params->low_pass_cutoff, float_ptr, sizeof(float)*4);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(&params->low_pass_cutoff,   sizeof(float));
    byteswap_inplace(&params->high_limit,        sizeof(float));
    byteswap_inplace(&params->high_limit_1sigma, sizeof(float));
    byteswap_inplace(&params->min_1sigma,        sizeof(float));
   }
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code; 
}

