/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <fstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_rawdata
{
  ////////////////////////////////////////////////////////////////////////
  //
  // RawData base class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  RawData::RawData() {}
  
  /** Update parameters: conversions and update */
  void RawData::setParameters(double min_range,
                              double max_range,
                              double view_direction,
                              double view_width)
  {
    config_.min_range = min_range;
    config_.max_range = max_range;

    //converting angle parameters into the velodyne reference (rad)
    config_.tmp_min_angle = view_direction + view_width/2;
    config_.tmp_max_angle = view_direction - view_width/2;
    
    //computing positive modulo to keep theses angles into [0;2*M_PI]
    config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle,2*M_PI) + 2*M_PI,2*M_PI);
    config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle,2*M_PI) + 2*M_PI,2*M_PI);
    
    //converting into the hardware velodyne ref (negative yaml and degrees)
    //adding 0.5 perfomrs a centered double to int conversion 
    config_.min_angle = 100 * (2*M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
    config_.max_angle = 100 * (2*M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
    if (config_.min_angle == config_.max_angle)
    {
      //avoid returning empty cloud if min_angle = max_angle
      config_.min_angle = 0;
      config_.max_angle = 36000;
    }
  }

  /** Set up for on-line operation. */
  int RawData::setup(ros::NodeHandle private_nh)
  {
    // get path to angles.config file for this device
    if (!private_nh.getParam("calibration", config_.calibrationFile))
      {
        ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

        // have to use something: grab unit test version as a default
        std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
        config_.calibrationFile = pkgPath + "/params/64e_utexas.yaml";
      }

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized) {
      ROS_ERROR_STREAM("Unable to open calibration file: " << 
          config_.calibrationFile);
      return -1;
    }
    
    ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");
    
    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }
   return 0;
  }


  /** Set up for offline operation */
  int RawData::setupOffline(std::string calibration_file, double max_range_, double min_range_)
  {

      config_.max_range = max_range_;
      config_.min_range = min_range_;
      ROS_INFO_STREAM("data ranges to publish: ["
	      << config_.min_range << ", "
	      << config_.max_range << "]");

      config_.calibrationFile = calibration_file;

      ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

      calibration_.read(config_.calibrationFile);
      if (!calibration_.initialized) {
	  ROS_ERROR_STREAM("Unable to open calibration file: " <<
		  config_.calibrationFile);
	  return -1;
      }

      // Set up cached values for sin and cos of all the possible headings
      for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
	  float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
	  cos_rot_table_[rot_index] = cosf(rotation);
	  sin_rot_table_[rot_index] = sinf(rotation);
      }
      return 0;
  }


  /** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack(const velodyne_msgs::VelodynePacket &pkt,
                       VPointCloud &pc, int packets_num)
  {
    ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);
    
    /** special parsing for the VLP16 **/
    if (calibration_.num_lasers == 16)
    {
      unpack_vlp16(pkt, pc);
      return;
    }
    
    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    for (int i = 0; i < BLOCKS_PER_PACKET; i++) {

      // upper bank lasers are numbered [0..31]
      // NOTE: this is a change from the old velodyne_common implementation
      int bank_origin = 0;
      if (raw->blocks[i].header == LOWER_BANK) {
        // lower bank lasers are [32..63]
        bank_origin = 32;
      }

      for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
        
        float x, y, z;
        float intensity;
        uint8_t laser_number;       ///< hardware laser number

        laser_number = j + bank_origin;
        velodyne_pointcloud::LaserCorrection &corrections = 
          calibration_.laser_corrections[laser_number];

        /** Position Calculation */

        union two_bytes tmp;
        tmp.bytes[0] = raw->blocks[i].data[k];
        tmp.bytes[1] = raw->blocks[i].data[k+1];
        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
        if ((raw->blocks[i].rotation >= config_.min_angle 
             && raw->blocks[i].rotation <= config_.max_angle 
             && config_.min_angle < config_.max_angle)
             ||(config_.min_angle > config_.max_angle 
             && (raw->blocks[i].rotation <= config_.max_angle 
             || raw->blocks[i].rotation >= config_.min_angle))){

          float distance;
          if (packets_num==(int) ceil(1507.0 / 10))
            distance = tmp.uint * DISTANCE_RESOLUTION * 2;
          else
            distance = tmp.uint * DISTANCE_RESOLUTION;

          distance += corrections.dist_correction;
  
          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;
  
          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle = 
            cos_rot_table_[raw->blocks[i].rotation] * cos_rot_correction + 
            sin_rot_table_[raw->blocks[i].rotation] * sin_rot_correction;
          float sin_rot_angle = 
            sin_rot_table_[raw->blocks[i].rotation] * cos_rot_correction - 
            cos_rot_table_[raw->blocks[i].rotation] * sin_rot_correction;
  
          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;
  
          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
  
          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0) xx=-xx;
          if (yy < 0) yy=-yy;
    
          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x = 
              (corrections.dist_correction - corrections.dist_correction_x)
                * (xx - 2.4) / (25.04 - 2.4) 
              + corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y = 
              (corrections.dist_correction - corrections.dist_correction_y)
                * (yy - 1.93) / (25.04 - 1.93)
              + corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }
  
          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
          ///the expression wiht '-' is proved to be better than the one with '+'
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
  
          float distance_y = distance + distance_corr_y;
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
  
          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;
  
          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;
  
          /** Intensity Calculation */
  
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;
  
          intensity = raw->blocks[i].data[k+2];
  
          float focal_offset = 256 
                             * (1 - corrections.focal_distance / 13100) 
                             * (1 - corrections.focal_distance / 13100);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope * (std::abs(focal_offset - 256 * 
            (1 - static_cast<float>(tmp.uint)/65535)*(1 - static_cast<float>(tmp.uint)/65535)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;
  
          if (pointInRange(distance)) {
  
            // convert polar coordinates to Euclidean XYZ
            VPoint point;
            point.ring = corrections.laser_ring;
            point.x = x_coord;
            point.y = y_coord;
            point.z = z_coord;
            point.intensity = intensity;
  
            // append this point to the cloud
            pc.points.push_back(point);
            ++pc.width;
          }
        }
      }
    }
  }
  
  /** @brief convert raw VLP16 packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack_vlp16(const velodyne_msgs::VelodynePacket &pkt,
                             VPointCloud &pc)
  {
    float azimuth;
    float azimuth_diff;
    float last_azimuth_diff=0;
    float azimuth_corrected_f;
    int azimuth_corrected;
    float x, y, z;
    float intensity;

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    for (int block = 0; block < BLOCKS_PER_PACKET; block++) {

      // ignore packets with mangled or otherwise different contents
      if (UPPER_BANK != raw->blocks[block].header) {
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                                 << block << " header value is "
                                 << raw->blocks[block].header);
        return;                         // bad packet: skip the rest
      }

      // Calculate difference between current and next block's azimuth angle.
      azimuth = (float)(raw->blocks[block].rotation);
      if (block < (BLOCKS_PER_PACKET-1)){
        azimuth_diff = (float)((36000 + raw->blocks[block+1].rotation - raw->blocks[block].rotation)%36000);
        last_azimuth_diff = azimuth_diff;
      }else{
        azimuth_diff = last_azimuth_diff;
      }

      for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++){
        for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE){
          velodyne_pointcloud::LaserCorrection &corrections = 
            calibration_.laser_corrections[dsr];

          /** Position Calculation */
          union two_bytes tmp;
          tmp.bytes[0] = raw->blocks[block].data[k];
          tmp.bytes[1] = raw->blocks[block].data[k+1];
          
          /** correct for the laser rotation as a function of timing during the firings **/
          azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
          azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;
          
          /*condition added to avoid calculating points which are not
            in the interesting defined area (min_angle < area < max_angle)*/
          if ((azimuth_corrected >= config_.min_angle 
               && azimuth_corrected <= config_.max_angle 
               && config_.min_angle < config_.max_angle)
               ||(config_.min_angle > config_.max_angle 
               && (azimuth_corrected <= config_.max_angle 
               || azimuth_corrected >= config_.min_angle))){

            // convert polar coordinates to Euclidean XYZ
            float distance = tmp.uint * DISTANCE_RESOLUTION;
            distance += corrections.dist_correction;
            
            float cos_vert_angle = corrections.cos_vert_correction;
            float sin_vert_angle = corrections.sin_vert_correction;
            float cos_rot_correction = corrections.cos_rot_correction;
            float sin_rot_correction = corrections.sin_rot_correction;
    
            // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
            // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
            float cos_rot_angle = 
              cos_rot_table_[azimuth_corrected] * cos_rot_correction + 
              sin_rot_table_[azimuth_corrected] * sin_rot_correction;
            float sin_rot_angle = 
              sin_rot_table_[azimuth_corrected] * cos_rot_correction - 
              cos_rot_table_[azimuth_corrected] * sin_rot_correction;
    
            float horiz_offset = corrections.horiz_offset_correction;
            float vert_offset = corrections.vert_offset_correction;
    
            // Compute the distance in the xy plane (w/o accounting for rotation)
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
    
            // Calculate temporal X, use absolute value.
            float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
            // Calculate temporal Y, use absolute value
            float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
            if (xx < 0) xx=-xx;
            if (yy < 0) yy=-yy;
      
            // Get 2points calibration values,Linear interpolation to get distance
            // correction for X and Y, that means distance correction use
            // different value at different distance
            float distance_corr_x = 0;
            float distance_corr_y = 0;
            if (corrections.two_pt_correction_available) {
              distance_corr_x = 
                (corrections.dist_correction - corrections.dist_correction_x)
                  * (xx - 2.4) / (25.04 - 2.4) 
                + corrections.dist_correction_x;
              distance_corr_x -= corrections.dist_correction;
              distance_corr_y = 
                (corrections.dist_correction - corrections.dist_correction_y)
                  * (yy - 1.93) / (25.04 - 1.93)
                + corrections.dist_correction_y;
              distance_corr_y -= corrections.dist_correction;
            }
    
            float distance_x = distance + distance_corr_x;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
            x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
    
            float distance_y = distance + distance_corr_y;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
            y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
    
            // Using distance_y is not symmetric, but the velodyne manual
            // does this.
            /**the new term of 'vert_offset * cos_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;
  
    
            /** Use standard ROS coordinate system (right-hand rule) */
            float x_coord = y;
            float y_coord = -x;
            float z_coord = z;
    
            /** Intensity Calculation */
            float min_intensity = corrections.min_intensity;
            float max_intensity = corrections.max_intensity;
    
            intensity = raw->blocks[block].data[k+2];
    
            float focal_offset = 256 
                               * (1 - corrections.focal_distance / 13100) 
                               * (1 - corrections.focal_distance / 13100);
            float focal_slope = corrections.focal_slope;
            intensity += focal_slope * (std::abs(focal_offset - 256 * 
              (1 - tmp.uint/65535)*(1 - tmp.uint/65535)));
            intensity = (intensity < min_intensity) ? min_intensity : intensity;
            intensity = (intensity > max_intensity) ? max_intensity : intensity;
    
            if (pointInRange(distance)) {
    
              // append this point to the cloud
              VPoint point;
              point.ring = corrections.laser_ring;
              point.x = x_coord;
              point.y = y_coord;
              point.z = z_coord;
              point.intensity = intensity;

              pc.points.push_back(point);
              ++pc.width;
            }
          }
        }
      }
    }
  }  

} // namespace velodyne_rawdata
