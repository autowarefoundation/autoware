/* 

Copyright (c) 2017, Brian Bingham
Copyright (c) 2017, Yuki Kitsukawa

All rights reserved

This file is part of the microstrain_3dm_gx5_45 package.

microstrain_3dm_gx5_45 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

microstrain_3dm_gx5_45 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

2017.06.07 Modified Microstrain::run() for support of 3DM-GX5-15.

*/

#include "microstrain.h"
#include <tf2/LinearMath/Transform.h>
#include <string>
#include <algorithm>  

namespace Microstrain
{
  Microstrain::Microstrain():
    // Initialization list
    filter_valid_packet_count_(0),
    ahrs_valid_packet_count_(0),
    gps_valid_packet_count_(0),
    filter_timeout_packet_count_(0),
    ahrs_timeout_packet_count_(0),
    gps_timeout_packet_count_(0),
    filter_checksum_error_packet_count_(0),
    ahrs_checksum_error_packet_count_(0),
    gps_checksum_error_packet_count_(0),
    gps_frame_id_("gps_frame"),
    imu_frame_id_("imu_frame"),
    odom_frame_id_("odom_frame"),
    odom_child_frame_id_("odom_frame"),
    publish_gps_(true),
    publish_imu_(true),
    publish_odom_(true)
  {
    // pass
  }
  Microstrain::~Microstrain()
  {
    // pass
  }
  void Microstrain::run()
  {
    // Variables for device configuration, ROS parameters, etc.
    u32 com_port, baudrate;
    bool device_setup = false;
    bool readback_settings = true;
    bool save_settings = true;
    bool auto_init = true;
    u8 auto_init_u8 = 1;
    u8 readback_headingsource = 0;
    u8 readback_auto_init = 0;
    u8 dynamics_mode           = 0;
    u8 readback_dynamics_mode  = 0;
    int declination_source;
    u8 declination_source_u8;
    u8 readback_declination_source;
    double declination;
    
    // Variables
    tf2::Quaternion quat;
    base_device_info_field device_info;
    u8  temp_string[20] = {0};
    u32 bit_result;
    u8  enable = 1;
    u8  data_stream_format_descriptors[10];
    u16 data_stream_format_decimation[10];
    u8  data_stream_format_num_entries = 0;
    u8  readback_data_stream_format_descriptors[10] = {0};
    u16 readback_data_stream_format_decimation[10]  = {0};
    u8  readback_data_stream_format_num_entries     =  0;
    u16 base_rate = 0;
    u16 device_descriptors[128]  = {0};
    u16 device_descriptors_size  = 128*2;
    s16 i;
    u16 j;
    u8  com_mode = 0;
    u8  readback_com_mode = 0;
    float angles[3]             = {0};
    float readback_angles[3]    = {0};
    float offset[3]             = {0};
    float readback_offset[3]    = {0};
    float hard_iron[3]          = {0};
    float hard_iron_readback[3] = {0};
    float soft_iron[9]          = {0};
    float soft_iron_readback[9] = {0};
    u16 estimation_control   = 0, estimation_control_readback = 0;
    u8  gps_source     = 0;
    u8  heading_source = 0x1;
    float noise[3]          = {0};
    float readback_noise[3] = {0};
    float beta[3]                 = {0};
    float readback_beta[3]        = {0};
    mip_low_pass_filter_settings filter_settings;
    float bias_vector[3]		   = {0};
    u16 duration = 0;
    gx4_imu_diagnostic_device_status_field imu_diagnostic_field;
    gx4_imu_basic_status_field imu_basic_field;
    gx4_45_diagnostic_device_status_field diagnostic_field;
    gx4_45_basic_status_field basic_field;
    mip_filter_external_gps_update_command external_gps_update;
    mip_filter_external_heading_update_command external_heading_update;
    mip_filter_zero_update_command zero_update_control, zero_update_readback;
    mip_filter_external_heading_with_time_command external_heading_with_time;
    mip_complementary_filter_settings comp_filter_command, comp_filter_readback;
    
    mip_filter_accel_magnitude_error_adaptive_measurement_command        accel_magnitude_error_command, accel_magnitude_error_readback;
    mip_filter_magnetometer_magnitude_error_adaptive_measurement_command mag_magnitude_error_command, mag_magnitude_error_readback;
    mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command mag_dip_angle_error_command, mag_dip_angle_error_readback;
    
    // ROS setup
    ros::Time::init();
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    // ROS Parameters
    // Comms Parameters
    std::string port;
    int baud, pdyn_mode;
    private_nh.param("port", port, std::string("/dev/ttyACM0"));
    private_nh.param("baudrate",baud,115200);
    baudrate = (u32)baud; 
    // Configuration Parameters
    private_nh.param("device_setup",device_setup,false);
    private_nh.param("readback_settings",readback_settings,true);
    private_nh.param("save_settings",save_settings,true);

    private_nh.param("auto_init",auto_init,true);
    private_nh.param("gps_rate",gps_rate_, 1);
    private_nh.param("imu_rate",imu_rate_, 10);
    private_nh.param("nav_rate",nav_rate_, 10);
    private_nh.param("dynamics_mode",pdyn_mode,1);
    dynamics_mode = (u8)pdyn_mode;
    if (dynamics_mode < 1 || dynamics_mode > 3){
      ROS_WARN("dynamics_mode can't be %#04X, must be 1, 2 or 3.  Setting to 1.",dynamics_mode);
      dynamics_mode = 1;
    }
    private_nh.param("declination_source",declination_source,2);
    if (declination_source < 1 || declination_source > 3){
      ROS_WARN("declination_source can't be %#04X, must be 1, 2 or 3.  Setting to 2.",declination_source);
      declination_source = 2;
    }
    declination_source_u8 = (u8)declination_source;
    //declination_source_command=(u8)declination_source;
    private_nh.param("declination",declination,0.23);
    private_nh.param("gps_frame_id",gps_frame_id_, std::string("wgs84"));
    private_nh.param("imu_frame_id",imu_frame_id_, std::string("base_link"));
    private_nh.param("odom_frame_id",odom_frame_id_, std::string("wgs84"));
    private_nh.param("odom_child_frame_id",odom_child_frame_id_,
		     std::string("base_link"));
    private_nh.param("publish_gps",publish_gps_, true);
    private_nh.param("publish_imu",publish_imu_, true);
    private_nh.param("publish_odom",publish_odom_, true);

    // ROS publishers and subscribers
    if (publish_gps_)
      gps_pub_ = node.advertise<sensor_msgs::NavSatFix>("gps/fix",100);
    if (publish_imu_)
//        imu_pub_ = node.advertise<sensor_msgs::Imu>("imu/data",100);
      imu_pub_ = node.advertise<sensor_msgs::Imu>("/imu_raw",100);
    if (publish_odom_)
    {
      nav_pub_ = node.advertise<nav_msgs::Odometry>("nav/odom",100);
      nav_status_pub_ = node.advertise<std_msgs::Int16MultiArray>("nav/status",100);
    }
    ros::ServiceServer service = node.advertiseService("reset_kf", &Microstrain::reset_callback, this);


    //Initialize the serial interface to the device
    ROS_INFO("Attempting to open serial port <%s> at <%d> \n",
	     port.c_str(),baudrate);
    if(mip_interface_init(port.c_str(), baudrate, &device_interface_, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK){
      ROS_FATAL("Couldn't open serial port!  Is it plugged in?");
    }


    // Setup device callbacks
    if(mip_interface_add_descriptor_set_callback(&device_interface_, MIP_FILTER_DATA_SET, this, &filter_packet_callback_wrapper) != MIP_INTERFACE_OK)
      {
	ROS_FATAL("Can't setup filter callback!");
	return;
      }
    if(mip_interface_add_descriptor_set_callback(&device_interface_, MIP_AHRS_DATA_SET, this, &ahrs_packet_callback_wrapper) != MIP_INTERFACE_OK)
      {
	ROS_FATAL("Can't setup ahrs callbacks!");
	return;
      }
    if(mip_interface_add_descriptor_set_callback(&device_interface_, MIP_GPS_DATA_SET, this, &gps_packet_callback_wrapper) != MIP_INTERFACE_OK)
      {
	ROS_FATAL("Can't setup gpscallbacks!");
	return;
      }

    ////////////////////////////////////////
    // Device setup
    float dT=0.5;  // common sleep time after setup communications
    if (device_setup)
    {
      // Put device into standard mode - we never really use "direct mode"
      ROS_INFO("Putting device communications into 'standard mode'");
      device_descriptors_size  = 128*2;
      com_mode = MIP_SDK_GX4_45_IMU_STANDARD_MODE;
      while(mip_system_com_mode(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK){}
      //Verify device mode setting
      ROS_INFO("Verify comm's mode");
      while(mip_system_com_mode(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &com_mode) != MIP_INTERFACE_OK){}
      ROS_INFO("Sleep for a second...");
      ros::Duration(dT).sleep();
      ROS_INFO("Right mode?");
      if(com_mode != MIP_SDK_GX4_45_IMU_STANDARD_MODE)
      {
	ROS_ERROR("Appears we didn't get into standard mode!");
      }

      // Put into idle mode
      ROS_INFO("Idling Device: Stopping data streams and/or waking from sleep");
      while(mip_base_cmd_idle(&device_interface_) != MIP_INTERFACE_OK){}
      ros::Duration(dT).sleep();

      // AHRS Setup
      // Get base rate
      if (publish_imu_){
	while(mip_3dm_cmd_get_ahrs_base_rate(&device_interface_, &base_rate) != MIP_INTERFACE_OK){}
	ROS_INFO("AHRS Base Rate => %d Hz", base_rate);
	ros::Duration(dT).sleep();
	// Deterimine decimation to get close to goal rate
	u8 imu_decimation = (u8)((float)base_rate/ (float)imu_rate_);
	ROS_INFO("AHRS decimation set to %#04X",imu_decimation);

	// AHRS Message Format
	// Set message format
	ROS_INFO("Setting the AHRS message format");
	data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
	data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;
	data_stream_format_descriptors[2] = MIP_AHRS_DATA_QUATERNION;
	data_stream_format_decimation[0]  = imu_decimation;//0x32;
	data_stream_format_decimation[1]  = imu_decimation;//0x32;
	data_stream_format_decimation[2]  = imu_decimation;//0x32;
	data_stream_format_num_entries = 3;
	while(mip_3dm_cmd_ahrs_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries, data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK){}
	ros::Duration(dT).sleep();
	// Poll to verify
	ROS_INFO("Poll AHRS data to verify");
	while(mip_3dm_cmd_poll_ahrs(&device_interface_, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries, data_stream_format_descriptors) != MIP_INTERFACE_OK){}
	ros::Duration(dT).sleep();
	// Save
	if (save_settings)
	{
	  ROS_INFO("Saving AHRS data settings");
	  while(mip_3dm_cmd_ahrs_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_STORE_EEPROM, 0, NULL,NULL) != MIP_INTERFACE_OK){}
	  ros::Duration(dT).sleep();
	}

	// Declination Source
	// Set declination

	/*
	ROS_INFO("Setting declination source to %#04X",declination_source_u8);
	while(mip_filter_declination_source(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &declination_source_u8) != MIP_INTERFACE_OK){}
	ros::Duration(dT).sleep();

	//Read back the declination source
	ROS_INFO("Reading back declination source");
	while(mip_filter_declination_source(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &readback_declination_source) != MIP_INTERFACE_OK){}
	if(declination_source_u8 == readback_declination_source)
	{
	  ROS_INFO("Success: Declination source set to %#04X", declination_source_u8);
	}
	else
	{
	  ROS_WARN("Failed to set the declination source to %#04X!", declination_source_u8);
	}
	ros::Duration(dT).sleep();
	if (save_settings)
	{
	  ROS_INFO("Saving declination source settings to EEPROM");
	  while(mip_filter_declination_source(&device_interface_, 
					      MIP_FUNCTION_SELECTOR_STORE_EEPROM,
					      NULL) != MIP_INTERFACE_OK)
	  {}
	  ros::Duration(dT).sleep();
	}
	*/
	
      } // end of AHRS setup

      // GPS Setup
      if (publish_gps_){
      
	while(mip_3dm_cmd_get_gps_base_rate(&device_interface_, &base_rate) != MIP_INTERFACE_OK){}
	ROS_INFO("GPS Base Rate => %d Hz", base_rate);
	u8 gps_decimation = (u8)((float)base_rate/ (float)gps_rate_);
	ros::Duration(dT).sleep();
      
	////////// GPS Message Format
	// Set
	ROS_INFO("Setting GPS stream format");
	data_stream_format_descriptors[0] = MIP_GPS_DATA_LLH_POS;
	data_stream_format_descriptors[1] = MIP_GPS_DATA_NED_VELOCITY;
	data_stream_format_descriptors[2] = MIP_GPS_DATA_GPS_TIME;
	data_stream_format_decimation[0]  = gps_decimation; //0x01; //0x04;
	data_stream_format_decimation[1]  = gps_decimation; //0x01; //0x04;
	data_stream_format_decimation[2]  = gps_decimation; //0x01; //0x04;
	data_stream_format_num_entries = 3;
	while(mip_3dm_cmd_gps_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries,data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK){}
	ros::Duration(dT).sleep();
	// Save
	if (save_settings)
	{
	  ROS_INFO("Saving GPS data settings");
	  while(mip_3dm_cmd_gps_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_STORE_EEPROM, 0, NULL,NULL) != MIP_INTERFACE_OK){}
	  ros::Duration(dT).sleep();
	}
      } // end of GPS setup

      if (publish_odom_){
	while(mip_3dm_cmd_get_filter_base_rate(&device_interface_, &base_rate) != MIP_INTERFACE_OK){}
	ROS_INFO("FILTER Base Rate => %d Hz", base_rate);
	u8 nav_decimation = (u8)((float)base_rate/ (float)nav_rate_);
	ros::Duration(dT).sleep();
	
	////////// Filter Message Format
	// Set
	ROS_INFO("Setting Filter stream format");
	data_stream_format_descriptors[0] = MIP_FILTER_DATA_LLH_POS;
	data_stream_format_descriptors[1] = MIP_FILTER_DATA_NED_VEL;
	//data_stream_format_descriptors[2] = MIP_FILTER_DATA_ATT_EULER_ANGLES;
	data_stream_format_descriptors[2] = MIP_FILTER_DATA_ATT_QUATERNION;
	data_stream_format_descriptors[3] = MIP_FILTER_DATA_POS_UNCERTAINTY;
	data_stream_format_descriptors[4] = MIP_FILTER_DATA_VEL_UNCERTAINTY;
	data_stream_format_descriptors[5] = MIP_FILTER_DATA_ATT_UNCERTAINTY_EULER;
	data_stream_format_descriptors[6] = MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE;
	data_stream_format_descriptors[7] = MIP_FILTER_DATA_FILTER_STATUS;
	data_stream_format_decimation[0]  = nav_decimation; //0x32;
	data_stream_format_decimation[1]  = nav_decimation; //0x32;
	data_stream_format_decimation[2]  = nav_decimation; //0x32;
	data_stream_format_decimation[3]  = nav_decimation; //0x32;
	data_stream_format_decimation[4]  = nav_decimation; //0x32;
	data_stream_format_decimation[5]  = nav_decimation; //0x32;
	data_stream_format_decimation[6]  = nav_decimation; //0x32;
	data_stream_format_decimation[7]  = nav_decimation; //0x32;
	data_stream_format_num_entries = 8;
	while(mip_3dm_cmd_filter_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries,data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK){}
	ros::Duration(dT).sleep();
	// Poll to verify
	ROS_INFO("Poll filter data to test stream");
	while(mip_3dm_cmd_poll_filter(&device_interface_, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries, data_stream_format_descriptors) != MIP_INTERFACE_OK){}
	ros::Duration(dT).sleep();
	// Save
	if (save_settings)
	{
	  ROS_INFO("Saving Filter data settings");
	  while(mip_3dm_cmd_filter_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_STORE_EEPROM, 0, NULL,NULL) != MIP_INTERFACE_OK){}
	  ros::Duration(dT).sleep();
	}
	// Dynamics Mode
	// Set dynamics mode
	ROS_INFO("Setting dynamics mode to %#04X",dynamics_mode);
	while(mip_filter_vehicle_dynamics_mode(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &dynamics_mode) != MIP_INTERFACE_OK){}
	ros::Duration(dT).sleep();
	// Readback dynamics mode
	if (readback_settings)
	{
	  // Read the settings back
	  ROS_INFO("Reading back dynamics mode setting");
	  while(mip_filter_vehicle_dynamics_mode(&device_interface_, 
						 MIP_FUNCTION_SELECTOR_READ, 
						 &readback_dynamics_mode)
		!= MIP_INTERFACE_OK)
	  {}
	  ros::Duration(dT).sleep();
	  if (dynamics_mode == readback_dynamics_mode)
	    ROS_INFO("Success: Dynamics mode setting is: %#04X",readback_dynamics_mode);
	  else
	    ROS_ERROR("Failure: Dynamics mode set to be %#04X, but reads as %#04X",
		      dynamics_mode,readback_dynamics_mode);
	}
	if (save_settings)
	{
	  ROS_INFO("Saving dynamics mode settings to EEPROM");
	  while(mip_filter_vehicle_dynamics_mode(&device_interface_, 
						 MIP_FUNCTION_SELECTOR_STORE_EEPROM,
						 NULL) != MIP_INTERFACE_OK)
	  {}
	  ros::Duration(dT).sleep();
	}
	
	// Heading Source
	ROS_INFO("Set heading source to internal mag.");
	heading_source = 0x1;
	ROS_INFO("Setting heading source to %#04X",heading_source);
	while(mip_filter_heading_source(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &heading_source) != MIP_INTERFACE_OK)
	{} 
	ros::Duration(dT).sleep();
	
	ROS_INFO("Read back heading source...");
	while(mip_filter_heading_source(&device_interface_, 
					MIP_FUNCTION_SELECTOR_READ, 
					&readback_headingsource)!= MIP_INTERFACE_OK){}
	ROS_INFO("Heading source = %#04X",readback_headingsource);
	ros::Duration(dT).sleep();
	
	if (save_settings)
	{
	  ROS_INFO("Saving heading source to EEPROM");
	  while(mip_filter_heading_source(&device_interface_, 
					  MIP_FUNCTION_SELECTOR_STORE_EEPROM, 
					  NULL)!= MIP_INTERFACE_OK){}
	  ros::Duration(dT).sleep();
	}
      }  // end of Filter setup

      // I believe the auto-init pertains to the kalman filter for the -45
      // OR for the complementary filter for the -25  - need to test
      // Auto Initialization
      // Set auto-initialization based on ROS parameter
      ROS_INFO("Setting auto-initinitalization to: %#04X",auto_init);
      auto_init_u8 = auto_init;  // convert bool to u8
      while(mip_filter_auto_initialization(&device_interface_, 
					   MIP_FUNCTION_SELECTOR_WRITE, 
					   &auto_init_u8) != MIP_INTERFACE_OK)
      {}
      ros::Duration(dT).sleep();
      
      if (readback_settings)
      {
	// Read the settings back
	ROS_INFO("Reading back auto-initialization value");
	while(mip_filter_auto_initialization(&device_interface_, 
					     MIP_FUNCTION_SELECTOR_READ, 
					     &readback_auto_init)!= MIP_INTERFACE_OK)
	{}
	ros::Duration(dT).sleep();
	if (auto_init == readback_auto_init)
	  ROS_INFO("Success: Auto init. setting is: %#04X",readback_auto_init);
	else
	  ROS_ERROR("Failure: Auto init. setting set to be %#04X, but reads as %#04X",
		    auto_init,readback_auto_init);
      }
      if (save_settings)
      {
	ROS_INFO("Saving auto init. settings to EEPROM");
	while(mip_filter_auto_initialization(&device_interface_, 
					     MIP_FUNCTION_SELECTOR_STORE_EEPROM,
					     NULL) != MIP_INTERFACE_OK)
	{}
	ros::Duration(dT).sleep();
      }

      // Enable Data streams
      dT = 0.25;
      if (publish_imu_){
	ROS_INFO("Enabling AHRS stream");
	enable = 0x01;
	while(mip_3dm_cmd_continuous_data_stream(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_AHRS_DATASTREAM, &enable) != MIP_INTERFACE_OK){}
	ros::Duration(dT).sleep();
      }
      if (publish_odom_){
	ROS_INFO("Enabling Filter stream");
	enable = 0x01;
	while(mip_3dm_cmd_continuous_data_stream(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_INS_DATASTREAM, &enable) != MIP_INTERFACE_OK){}
	ros::Duration(dT).sleep();
      }
      if (publish_gps_){
	ROS_INFO("Enabling GPS stream");
	enable = 0x01;
	while(mip_3dm_cmd_continuous_data_stream(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_GPS_DATASTREAM, &enable) != MIP_INTERFACE_OK){}
	ros::Duration(dT).sleep();
      }

      ROS_INFO("End of device setup - starting streaming");
    } 
    else
    {
      ROS_INFO("Skipping device setup and listing for existing streams");
    } // end of device_setup
    
    // Reset filter - should be for either the KF or CF
    ROS_INFO("Reset filter");
    while(mip_filter_reset_filter(&device_interface_) != MIP_INTERFACE_OK){}
    ros::Duration(dT).sleep();

    // Loop
    // Determine loop rate as 2*(max update rate), but abs. max of 1kHz
    int max_rate = 1;
    if (publish_imu_){
      max_rate = std::max(max_rate,imu_rate_);
    }
    if (publish_gps_){
      max_rate = std::max(max_rate,gps_rate_);
    }
    if (publish_odom_){
      max_rate = std::max(max_rate,nav_rate_);
    }
    int spin_rate = std::min(3*max_rate,1000);
    ROS_INFO("Setting spin rate to <%d>",spin_rate);
    ros::Rate r(spin_rate);  // Rate in Hz
    while (ros::ok()){
      //Update the parser (this function reads the port and parses the bytes
      mip_interface_update(&device_interface_);
      ros::spinOnce();  // take care of service requests.
      r.sleep();
      
      //ROS_INFO("Spinning");
    } // end loop
    
    // close serial port
    mip_sdk_port_close(device_interface_.port_handle);
    
  } // End of ::run()
  
  bool Microstrain::reset_callback(std_srvs::Empty::Request &req,
				   std_srvs::Empty::Response &resp)
  {
    ROS_INFO("Reseting the filter");
    while(mip_filter_reset_filter(&device_interface_) != MIP_INTERFACE_OK){}
    
    return true;
  }

  void Microstrain::filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;

    // If we aren't publishing, then return
    if (!publish_odom_)
      return;
    //ROS_INFO("Filter callback");
    //The packet callback can have several types, process them all
    switch(callback_type)
      {
	///
	//Handle valid packets
	///

      case MIP_INTERFACE_CALLBACK_VALID_PACKET:
	{
	  filter_valid_packet_count_++;

	  ///
	  //Loop through all of the data fields
	  ///

	  while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
	    {

	      ///
	      // Decode the field
	      ///

	      switch(field_header->descriptor)
		{
		  ///
		  // Estimated LLH Position
		  ///

		case MIP_FILTER_DATA_LLH_POS:
		  {
		    memcpy(&curr_filter_pos_, field_data, sizeof(mip_filter_llh_pos));

		    //For little-endian targets, byteswap the data field
		    mip_filter_llh_pos_byteswap(&curr_filter_pos_);

		    nav_msg_.header.seq = filter_valid_packet_count_;
		    nav_msg_.header.stamp = ros::Time::now();
		    nav_msg_.header.frame_id = odom_frame_id_;
		    nav_msg_.child_frame_id = odom_child_frame_id_;
		    nav_msg_.pose.pose.position.y = curr_filter_pos_.latitude;
		    nav_msg_.pose.pose.position.x = curr_filter_pos_.longitude;
		    nav_msg_.pose.pose.position.z = curr_filter_pos_.ellipsoid_height;

		  }break;

		  ///
		  // Estimated NED Velocity
		  ///

		case MIP_FILTER_DATA_NED_VEL:
		  {
		    memcpy(&curr_filter_vel_, field_data, sizeof(mip_filter_ned_velocity));

		    //For little-endian targets, byteswap the data field
		    mip_filter_ned_velocity_byteswap(&curr_filter_vel_);
      
		    // rotate velocities from NED to sensor coordinates
		    // Constructor takes x, y, z , w
		    tf2::Quaternion nav_quat(curr_filter_quaternion_.q[2],
					     curr_filter_quaternion_.q[1],
					     -1.0*curr_filter_quaternion_.q[3],
					     curr_filter_quaternion_.q[0]);
					     
		    tf2::Vector3 vel_enu(curr_filter_vel_.east,
					 curr_filter_vel_.north,
					 -1.0*curr_filter_vel_.down);
		    tf2::Vector3 vel_in_sensor_frame = tf2::quatRotate(nav_quat.inverse(),vel_enu);
		      
		    nav_msg_.twist.twist.linear.x = vel_in_sensor_frame[0]; //curr_filter_vel_.east;
		    nav_msg_.twist.twist.linear.y =  vel_in_sensor_frame[1]; //curr_filter_vel_.north;
		    nav_msg_.twist.twist.linear.z =  vel_in_sensor_frame[2]; //-1*curr_filter_vel_.down;
		  }break;

		  ///
		  // Estimated Attitude, Euler Angles
		  ///

		case MIP_FILTER_DATA_ATT_EULER_ANGLES:
		  {
		    memcpy(&curr_filter_angles_, field_data, sizeof(mip_filter_attitude_euler_angles));

		    //For little-endian targets, byteswap the data field
		    mip_filter_attitude_euler_angles_byteswap(&curr_filter_angles_);

		  }break;

		  // Quaternion
		case MIP_FILTER_DATA_ATT_QUATERNION:
		  {
		    memcpy(&curr_filter_quaternion_, field_data, sizeof(mip_filter_attitude_quaternion));

		    //For little-endian targets, byteswap the data field
		    mip_filter_attitude_quaternion_byteswap(&curr_filter_quaternion_);

		    // put into ENU - swap X/Y, invert Z
		    nav_msg_.pose.pose.orientation.x = curr_filter_quaternion_.q[2];
		    nav_msg_.pose.pose.orientation.y = curr_filter_quaternion_.q[1];
		    nav_msg_.pose.pose.orientation.z = -1.0*curr_filter_quaternion_.q[3];
		    nav_msg_.pose.pose.orientation.w = curr_filter_quaternion_.q[0];

		  }break;

		  // Angular Rates
		case MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE:
		  {
		    memcpy(&curr_filter_angular_rate_, field_data, sizeof(mip_filter_compensated_angular_rate));

		    //For little-endian targets, byteswap the data field
		    mip_filter_compensated_angular_rate_byteswap(&curr_filter_angular_rate_);

		    nav_msg_.twist.twist.angular.x = curr_filter_angular_rate_.x;
		    nav_msg_.twist.twist.angular.y = curr_filter_angular_rate_.y;
		    nav_msg_.twist.twist.angular.z = curr_filter_angular_rate_.z;


		  }break;

		  // Position Uncertainty
		case MIP_FILTER_DATA_POS_UNCERTAINTY:
		  {
		    memcpy(&curr_filter_pos_uncertainty_, field_data, sizeof(mip_filter_llh_pos_uncertainty));

		    //For little-endian targets, byteswap the data field
		    mip_filter_llh_pos_uncertainty_byteswap(&curr_filter_pos_uncertainty_);

		    //x-axis
		    nav_msg_.pose.covariance[0] = curr_filter_pos_uncertainty_.east*curr_filter_pos_uncertainty_.east;
		    nav_msg_.pose.covariance[7] = curr_filter_pos_uncertainty_.north*curr_filter_pos_uncertainty_.north;
		    nav_msg_.pose.covariance[14] = curr_filter_pos_uncertainty_.down*curr_filter_pos_uncertainty_.down;
		  }break;

		  // Velocity Uncertainty
		case MIP_FILTER_DATA_VEL_UNCERTAINTY:
		  {
		    memcpy(&curr_filter_vel_uncertainty_, field_data, sizeof(mip_filter_ned_vel_uncertainty));

		    //For little-endian targets, byteswap the data field
		    mip_filter_ned_vel_uncertainty_byteswap(&curr_filter_vel_uncertainty_);
      
		    nav_msg_.twist.covariance[0] = curr_filter_vel_uncertainty_.east*curr_filter_vel_uncertainty_.east;
		    nav_msg_.twist.covariance[7] = curr_filter_vel_uncertainty_.north*curr_filter_vel_uncertainty_.north;
		    nav_msg_.twist.covariance[14] = curr_filter_vel_uncertainty_.down*curr_filter_vel_uncertainty_.down;

		  }break;

		  // Attitude Uncertainty
		case MIP_FILTER_DATA_ATT_UNCERTAINTY_EULER:
		  {
		    memcpy(&curr_filter_att_uncertainty_, field_data, sizeof(mip_filter_euler_attitude_uncertainty));

		    //For little-endian targets, byteswap the data field
		    mip_filter_euler_attitude_uncertainty_byteswap(&curr_filter_att_uncertainty_);
		    nav_msg_.pose.covariance[21] = curr_filter_att_uncertainty_.roll*curr_filter_att_uncertainty_.roll;
		    nav_msg_.pose.covariance[28] = curr_filter_att_uncertainty_.pitch*curr_filter_att_uncertainty_.pitch;
		    nav_msg_.pose.covariance[35] = curr_filter_att_uncertainty_.yaw*curr_filter_att_uncertainty_.yaw;

		  }break;

		  // Filter Status
		case MIP_FILTER_DATA_FILTER_STATUS:
		  {
		    memcpy(&curr_filter_status_, field_data, sizeof(mip_filter_status));

		    //For little-endian targets, byteswap the data field
		    mip_filter_status_byteswap(&curr_filter_status_);
      
		    nav_status_msg_.data.clear();
		    ROS_DEBUG_THROTTLE(1.0,"Filter Status: %#06X, Dyn. Mode: %#06X, Filter State: %#06X",
				       curr_filter_status_.filter_state,
				       curr_filter_status_.dynamics_mode,
				       curr_filter_status_.status_flags);
		    nav_status_msg_.data.push_back(curr_filter_status_.filter_state);
		    nav_status_msg_.data.push_back(curr_filter_status_.dynamics_mode);
		    nav_status_msg_.data.push_back(curr_filter_status_.status_flags);
		    nav_status_pub_.publish(nav_status_msg_);


		  }break;

		default: break;
		}
	    }

	  // Publish
	  nav_pub_.publish(nav_msg_);
	}break;


	///
	//Handle checksum error packets
	///

      case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
	{
	  filter_checksum_error_packet_count_++;
	}break;

	///
	//Handle timeout packets
	///

      case MIP_INTERFACE_CALLBACK_TIMEOUT:
	{
	  filter_timeout_packet_count_++;
	}break;
      default: break;
      }

    print_packet_stats();
  } // filter_packet_callback


  ////////////////////////////////////////////////////////////////////////////////
  //
  // AHRS Packet Callback
  //
  ////////////////////////////////////////////////////////////////////////////////

  void Microstrain::ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;
    // If we aren't publishing, then return
    if (!publish_imu_)
      return;
    //The packet callback can have several types, process them all
    switch(callback_type)
      {
	///
	//Handle valid packets
	///

      case MIP_INTERFACE_CALLBACK_VALID_PACKET:
	{
	  ahrs_valid_packet_count_++;

	  ///
	  //Loop through all of the data fields
	  ///

	  while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
	    {

	      ///
	      // Decode the field
	      ///

	      switch(field_header->descriptor)
		{
		  ///
		  // Scaled Accelerometer
		  ///

		case MIP_AHRS_DATA_ACCEL_SCALED:
		  {
		    memcpy(&curr_ahrs_accel_, field_data, sizeof(mip_ahrs_scaled_accel));

		    //For little-endian targets, byteswap the data field
		    mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel_);
      
		    // Stuff into ROS message - acceleration in m/s^2
		    // Header
		    imu_msg_.header.seq = ahrs_valid_packet_count_;
		    imu_msg_.header.stamp = ros::Time::now();
		    imu_msg_.header.frame_id = imu_frame_id_;
		    imu_msg_.linear_acceleration.x = 9.81*curr_ahrs_accel_.scaled_accel[0];
		    imu_msg_.linear_acceleration.y = 9.81*curr_ahrs_accel_.scaled_accel[1];
		    imu_msg_.linear_acceleration.z = 9.81*curr_ahrs_accel_.scaled_accel[2];
      
		  }break;

		  ///
		  // Scaled Gyro
		  ///

		case MIP_AHRS_DATA_GYRO_SCALED:
		  {
		    memcpy(&curr_ahrs_gyro_, field_data, sizeof(mip_ahrs_scaled_gyro));

		    //For little-endian targets, byteswap the data field
		    mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro_);
      
		    imu_msg_.angular_velocity.x = curr_ahrs_gyro_.scaled_gyro[0];
		    imu_msg_.angular_velocity.y = curr_ahrs_gyro_.scaled_gyro[1];
		    imu_msg_.angular_velocity.z = curr_ahrs_gyro_.scaled_gyro[2];

		  }break;

		  ///
		  // Scaled Magnetometer
		  ///

		case MIP_AHRS_DATA_MAG_SCALED:
		  {
		    memcpy(&curr_ahrs_mag_, field_data, sizeof(mip_ahrs_scaled_mag));

		    //For little-endian targets, byteswap the data field
		    mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag_);

		  }break;

		  // Quaternion
		case MIP_AHRS_DATA_QUATERNION:
		  {
		    memcpy(&curr_ahrs_quaternion_, field_data, sizeof(mip_ahrs_quaternion));

		    //For little-endian targets, byteswap the data field
		    mip_ahrs_quaternion_byteswap(&curr_ahrs_quaternion_);
		    // put into ENU - swap X/Y, invert Z
		    imu_msg_.orientation.x = curr_ahrs_quaternion_.q[2];
		    imu_msg_.orientation.y = curr_ahrs_quaternion_.q[1];
		    imu_msg_.orientation.z = -1.0*curr_ahrs_quaternion_.q[3];
		    imu_msg_.orientation.w = curr_ahrs_quaternion_.q[0];

		  }break;

		default: break;
		}
	    }
   
	  // Publish
	  imu_pub_.publish(imu_msg_);

	}break;

	///
	//Handle checksum error packets
	///

      case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
	{
	  ahrs_checksum_error_packet_count_++;
	}break;

	///
	//Handle timeout packets
	///

      case MIP_INTERFACE_CALLBACK_TIMEOUT:
	{
	  ahrs_timeout_packet_count_++;
	}break;
      default: break;
      }

    print_packet_stats();
  } // ahrs_packet_callback


  ////////////////////////////////////////////////////////////////////////////////
  //
  // GPS Packet Callback
  //
  ////////////////////////////////////////////////////////////////////////////////

  void Microstrain::gps_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;
    u8 msgvalid = 1;  // keep track of message validity

    // If we aren't publishing, then return
    if (!publish_gps_)
      return;
    //The packet callback can have several types, process them all
    switch(callback_type)
      {
	///
	//Handle valid packets
	///

      case MIP_INTERFACE_CALLBACK_VALID_PACKET:
	{
	  gps_valid_packet_count_++;

	  ///
	  //Loop through all of the data fields
	  ///

	  while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
	    {

	      ///
	      // Decode the field
	      ///

	      switch(field_header->descriptor)
		{
		  ///
		  // LLH Position
		  ///

		case MIP_GPS_DATA_LLH_POS:
		  {
		    memcpy(&curr_llh_pos_, field_data, sizeof(mip_gps_llh_pos));

		    //For little-endian targets, byteswap the data field
		    mip_gps_llh_pos_byteswap(&curr_llh_pos_);

		    // push into ROS message
		    gps_msg_.latitude = curr_llh_pos_.latitude;
		    gps_msg_.longitude = curr_llh_pos_.longitude;
		    gps_msg_.altitude = curr_llh_pos_.ellipsoid_height;
		    gps_msg_.position_covariance_type = 2;  // diagnals known
		    gps_msg_.position_covariance[0] = curr_llh_pos_.horizontal_accuracy*curr_llh_pos_.horizontal_accuracy;
		    gps_msg_.position_covariance[4] = curr_llh_pos_.horizontal_accuracy*curr_llh_pos_.horizontal_accuracy;
		    gps_msg_.position_covariance[8] = curr_llh_pos_.vertical_accuracy*curr_llh_pos_.vertical_accuracy;
		    gps_msg_.status.status = curr_llh_pos_.valid_flags - 1;
		    gps_msg_.status.service = 1;  // assumed
		    // Header
		    gps_msg_.header.seq = gps_valid_packet_count_;
		    gps_msg_.header.stamp = ros::Time::now();
		    gps_msg_.header.frame_id = gps_frame_id_;

		  }break;

		  ///
		  // NED Velocity
		  ///

		case MIP_GPS_DATA_NED_VELOCITY:
		  {
		    memcpy(&curr_ned_vel_, field_data, sizeof(mip_gps_ned_vel));

		    //For little-endian targets, byteswap the data field
		    mip_gps_ned_vel_byteswap(&curr_ned_vel_);

		  }break;

		  ///
		  // GPS Time
		  ///

		case MIP_GPS_DATA_GPS_TIME:
		  {
		    memcpy(&curr_gps_time_, field_data, sizeof(mip_gps_time));

		    //For little-endian targets, byteswap the data field
		    mip_gps_time_byteswap(&curr_gps_time_);

		  }break;

		default: break;
		}
	    }
	}break;


	///
	//Handle checksum error packets
	///

      case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
	{
	  msgvalid = 0;
	  gps_checksum_error_packet_count_++;
	}break;

	///
	//Handle timeout packets
	///

      case MIP_INTERFACE_CALLBACK_TIMEOUT:
	{
	  msgvalid = 0;
	  gps_timeout_packet_count_++;
	}break;
      default: break;
      }

    if (msgvalid){
      // Publish the message
      gps_pub_.publish(gps_msg_);
    }

    print_packet_stats();
  } // gps_packet_callback

  void Microstrain::print_packet_stats()
  {
    ROS_DEBUG_THROTTLE(1.0,"%u FILTER (%u errors)    %u AHRS (%u errors)    %u GPS (%u errors) Packets", filter_valid_packet_count_,  filter_timeout_packet_count_ + filter_checksum_error_packet_count_,
		       ahrs_valid_packet_count_, ahrs_timeout_packet_count_ + ahrs_checksum_error_packet_count_,
		       gps_valid_packet_count_,  gps_timeout_packet_count_ + gps_checksum_error_packet_count_);
  } // print_packet_stats


  // Wrapper callbacks
  void filter_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    Microstrain* ustrain = (Microstrain*) user_ptr;
    ustrain->filter_packet_callback(user_ptr,packet,packet_size,callback_type);
  }

  void ahrs_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    Microstrain* ustrain = (Microstrain*) user_ptr;
    ustrain->ahrs_packet_callback(user_ptr,packet,packet_size,callback_type);
  }
  // Wrapper callbacks
  void gps_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    Microstrain* ustrain = (Microstrain*) user_ptr;
    ustrain->gps_packet_callback(user_ptr,packet,packet_size,callback_type);
  }

} // Microstrain namespace

