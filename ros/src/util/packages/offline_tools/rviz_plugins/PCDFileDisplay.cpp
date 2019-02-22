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
 * PCDMap.cpp
 *
 *  Created on: Sep 30, 2018
 *      Author: sujiwo
 */

#include "PCDFileDisplay.h"
#include <OGRE/OgreSceneManager.h>
#include <pcl_ros/point_cloud.h>
#include <rviz/properties/status_property.h>

using namespace std;
// using pcl::PointCloud;
// using pcl::PointXYZ;

PCDFileDisplay::PCDFileDisplay() : rviz::Display() {
  cloudMsg_ = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);

  pcdfile_ = new rviz::StringProperty(
      "PCDFilePath", QString(), "PCD File to load", this, SLOT(changeFile()));

  cloud_render_ = boost::shared_ptr<rviz::PointCloud>(new rviz::PointCloud());

  style_property_ = new rviz::EnumProperty(
      "Style", "Flat Squares",
      "Rendering mode to use, in order of computational complexity.", this,
      SLOT(updateStyle()), this);
  style_property_->addOption("Points", rviz::PointCloud::RM_POINTS);
  style_property_->addOption("Squares", rviz::PointCloud::RM_SQUARES);
  style_property_->addOption("Flat Squares", rviz::PointCloud::RM_FLAT_SQUARES);
  style_property_->addOption("Spheres", rviz::PointCloud::RM_SPHERES);
  style_property_->addOption("Boxes", rviz::PointCloud::RM_BOXES);

  point_world_size_property_ =
      new rviz::FloatProperty("Size (m)", 0.01, "Point size in meters.", this,
                              SLOT(updateBillboardSize()), this);
  point_world_size_property_->setMin(0.0001);

  point_pixel_size_property_ =
      new rviz::FloatProperty("Size (Pixels)", 3, "Point size in pixels.", this,
                              SLOT(updateBillboardSize()), this);
  point_pixel_size_property_->setMin(1);

  colorChooser_ = new rviz::EnumProperty(
      "Color Transformer", "",
      "Set the transformer to use to set the color of the points.", this,
      SLOT(updateColorTransformer()), this);

  axesColorTransform_ = new rviz::AxisColorPCTransformer;
  axesColorTransform_->createProperties(
      this, rviz::PointCloudTransformer::Support_Color,
      axesColorTransformProps);
  connect(axesColorTransform_, SIGNAL(needRetransform()), this,
          SLOT(causeRetransform()));

  flatColorTransform_ = new rviz::FlatColorPCTransformer;
  flatColorTransform_->createProperties(
      this, rviz::PointCloudTransformer::Support_Color,
      flatColorTransformProps);
  connect(flatColorTransform_, SIGNAL(needRetransform()), this,
          SLOT(causeRetransform()));

  colorChooser_->addOption("Flat", FLAT_COLOR);
  colorChooser_->addOption("Z Color", Z_COLOR);

  activeTransform_ = flatColorTransform_;
}

PCDFileDisplay::~PCDFileDisplay() {}

void PCDFileDisplay::onInitialize() {}

void PCDFileDisplay::changeFile() {
  const string filename = pcdfile_->getString().toStdString();
  return updatePointCloud(filename);
}

void PCDFileDisplay::updateStyle() { updateDisplay(); }

void PCDFileDisplay::updateColorTransformer() {
  if (colorChooser_->getOptionInt() == FLAT_COLOR)
    activeTransform_ = flatColorTransform_;
  else if (colorChooser_->getOptionInt() == Z_COLOR)
    activeTransform_ = axesColorTransform_;

  updateDisplay();
}

void PCDFileDisplay::updateDisplay() {
  cloud_render_->clear();
  activeTransform_->transform(cloudMsg_,
                              rviz::PointCloudTransformer::Support_Color,
                              Ogre::Matrix4::IDENTITY, pointList);
  cloud_render_->addPoints(pointList.data(), pointList.size());

  rviz::PointCloud::RenderMode renderMode =
      (rviz::PointCloud::RenderMode)style_property_->getOptionInt();
  cloud_render_->setRenderMode(renderMode);

  queueRender();
}

void PCDFileDisplay::causeRetransform() { updateDisplay(); }

void PCDFileDisplay::updatePointCloud(const std::string &loadThisFile) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_open_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_render_->clear();

  pcl::PCDReader fileReader;
  try {
    fileReader.read(loadThisFile, *cloud_open_);
    pcl::toROSMsg(*cloud_open_, *cloudMsg_);

    // Do something with this pointcloud
    pointList.clear();
    pointList.resize(cloud_open_->width * cloud_open_->height);
    int i = 0;
    for (auto it = cloud_open_->begin(); it != cloud_open_->end(); ++it) {
      pcl::PointXYZ &p = *it;
      rviz::PointCloud::Point pn;
      pn.position = Ogre::Vector3(p.x, p.y, p.z);
      pointList.at(i) = pn;
      ++i;
    }

    scene_node_->attachObject(cloud_render_.get());
    updateDisplay();

  } catch (exception &e) {
    // put error in rviz status
  }
}

void PCDFileDisplay::updateBillboardSize() {
  rviz::PointCloud::RenderMode mode =
      (rviz::PointCloud::RenderMode)style_property_->getOptionInt();
  float size;
  if (mode == rviz::PointCloud::RM_POINTS) {
    size = point_pixel_size_property_->getFloat();
  } else {
    size = point_world_size_property_->getFloat();
  }
  cloud_render_->setDimensions(size, size, size);

  //	queueRender();
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PCDFileDisplay, rviz::Display)
