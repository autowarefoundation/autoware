/*
 *  Copyright (c) 2019, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *  THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * PCDMap.h
 *
 *  Created on: Sep 30, 2018
 *      Author: sujiwo
 */

#ifndef _RVIZ_PLUGINS_PCDMAP_H_
#define _RVIZ_PLUGINS_PCDMAP_H_

#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/display.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>

class PCDFileDisplay : public rviz::Display {
  Q_OBJECT
public:
  PCDFileDisplay();
  virtual ~PCDFileDisplay();

  enum { FLAT_COLOR, Z_COLOR };

protected:
  virtual void onInitialize();

public Q_SLOTS:
  void causeRetransform();

private Q_SLOTS:
  void changeFile();
  void updateStyle();
  void updateBillboardSize();
  void updateColorTransformer();

private:
  rviz::StringProperty *pcdfile_;
  rviz::EnumProperty *style_property_;
  rviz::FloatProperty *point_world_size_property_;
  rviz::FloatProperty *point_pixel_size_property_;

  sensor_msgs::PointCloud2::Ptr cloudMsg_;
  boost::shared_ptr<rviz::PointCloud> cloud_render_;
  std::vector<rviz::PointCloud::Point> pointList;

  rviz::EnumProperty *colorChooser_;

  rviz::AxisColorPCTransformer *axesColorTransform_;
  QList<rviz::Property *> axesColorTransformProps;

  rviz::FlatColorPCTransformer *flatColorTransform_;
  QList<rviz::Property *> flatColorTransformProps;

  rviz::PointCloudTransformer *activeTransform_ = NULL;

private:
  void updatePointCloud(const std::string &loadThisFile);

  void updateDisplay();
};

#endif /* _RVIZ_PLUGINS_PCDMAP_H_ */
