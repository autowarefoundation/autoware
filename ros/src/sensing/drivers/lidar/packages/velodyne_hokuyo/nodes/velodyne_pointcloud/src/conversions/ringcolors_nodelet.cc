/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This ROS nodelet converts a Velodyne 3D LIDAR PointXYZIR cloud to
    PointXYZRGB, assigning colors for visualization of the laser
    rings.

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "colors.h"

namespace velodyne_pointcloud
{
  class vhRingColorsNodelet: public nodelet::Nodelet
  {
  public:

    vhRingColorsNodelet() {}
    ~vhRingColorsNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<RingColors> colors_;
  };

  /** @brief Nodelet initialization. */
  void vhRingColorsNodelet::onInit()
  {
    colors_.reset(new RingColors(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace velodyne_pointcloud


// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_pointcloud, vhRingColorsNodelet,
                        velodyne_pointcloud::vhRingColorsNodelet, nodelet::Nodelet);
