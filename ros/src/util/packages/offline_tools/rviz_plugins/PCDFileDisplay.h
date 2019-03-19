/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
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
