/**
 * \file  calibration.h 
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, Austin Robot Technology, University of Texas at Austin
 *
 * License: Modified BSD License
 *
 * $ Id: 02/14/2012 11:25:34 AM piyushk $
 */

#ifndef __VELODYNE_CALIBRATION_H
#define __VELODYNE_CALIBRATION_H

#include <map>
#include <string>

namespace velodyne_pointcloud {

  /** \brief correction values for a single laser
   *
   * Correction values for a single laser (as provided by db.xml from veleodyne)
   * Includes parameters for Velodyne HDL-64E S2.1 calibration.
   * http://velodynelidar.com/lidar/products/manual/63-HDL64E%20S2%20Manual_Rev%20D_2011_web.pdf
   **/
  struct LaserCorrection {

    /** parameters in db.xml */
    float rot_correction;
    float vert_correction;
    float dist_correction;
    bool two_pt_correction_available;
    float dist_correction_x;
    float dist_correction_y;
    float vert_offset_correction;
    float horiz_offset_correction;
    int max_intensity;
    int min_intensity;
    float focal_distance;
    float focal_slope;

    /** cached values calculated when the calibration file is read */
    float cos_rot_correction;              ///< cached cosine of rot_correction
    float sin_rot_correction;              ///< cached sine of rot_correction
    float cos_vert_correction;             ///< cached cosine of vert_correction
    float sin_vert_correction;             ///< cached sine of vert_correction

    int laser_ring;                        ///< ring number for this laser
  };

  /** \brief Calibration class storing entire configuration for the Velodyne */
  class Calibration {

  public:

    std::map<int, LaserCorrection> laser_corrections;
    int num_lasers;
    bool initialized;

  public:

    Calibration() : initialized(false) {}
    Calibration(const std::string& calibration_file) {
      read(calibration_file);
    }

    void read(const std::string& calibration_file);
    void write(const std::string& calibration_file);
  };
  
} /* velodyne_pointcloud */


#endif /* end of include guard: __VELODYNE_CALIBRATION_H */


