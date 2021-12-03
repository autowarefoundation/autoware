// Copyright 2018 Autoware Foundation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "shape_estimation/shape_estimator.hpp"

#include "shape_estimation/corrector/corrector.hpp"
#include "shape_estimation/filter/filter.hpp"
#include "shape_estimation/model/model.hpp"

#include <iostream>
#include <memory>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

ShapeEstimator::ShapeEstimator(bool use_corrector, bool use_filter)
: use_corrector_(use_corrector), use_filter_(use_filter)
{
}

bool ShapeEstimator::estimateShapeAndPose(
  const uint8_t label, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  const boost::optional<float> & yaw, autoware_auto_perception_msgs::msg::Shape & shape_output,
  geometry_msgs::msg::Pose & pose_output)
{
  autoware_auto_perception_msgs::msg::Shape shape;
  geometry_msgs::msg::Pose pose;
  // estimate shape
  if (!estimateShape(label, cluster, yaw, shape, pose)) {
    return false;
  }

  // rule based filter
  if (use_filter_) {
    if (!applyFilter(label, shape, pose)) {
      return false;
    }
  }

  // rule based corrector
  if (use_corrector_) {
    bool use_reference_yaw = yaw ? true : false;
    if (!applyCorrector(label, use_reference_yaw, shape, pose)) {
      return false;
    }
  }

  shape_output = shape;
  pose_output = pose;
  return true;
}

bool ShapeEstimator::estimateShape(
  const uint8_t label, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  const boost::optional<float> & yaw, autoware_auto_perception_msgs::msg::Shape & shape_output,
  geometry_msgs::msg::Pose & pose_output)
{
  // estimate shape
  std::unique_ptr<ShapeEstimationModelInterface> model_ptr;
  if (
    label == Label::CAR || label == Label::TRUCK || label == Label::BUS ||
    label == Label::TRAILER) {
    model_ptr.reset(new BoundingBoxShapeModel(yaw));
  } else if (label == Label::PEDESTRIAN) {
    model_ptr.reset(new CylinderShapeModel());
  } else if (label == Label::MOTORCYCLE) {
    model_ptr.reset(new BoundingBoxShapeModel(yaw));
  } else if (label == Label::BICYCLE) {
    model_ptr.reset(new BoundingBoxShapeModel(yaw));
  } else {
    model_ptr.reset(new ConvexHullShapeModel());
  }

  return model_ptr->estimate(cluster, shape_output, pose_output);
}

bool ShapeEstimator::applyFilter(
  const uint8_t label, const autoware_auto_perception_msgs::msg::Shape & shape_output,
  const geometry_msgs::msg::Pose & pose_output)
{
  std::unique_ptr<ShapeEstimationFilterInterface> filter_ptr;
  if (label == Label::CAR) {
    filter_ptr.reset(new CarFilter);
  } else if (label == Label::BUS) {
    filter_ptr.reset(new BusFilter);
  } else if (label == Label::TRUCK || label == Label::TRAILER) {
    filter_ptr.reset(new TruckFilter);
  } else {
    filter_ptr.reset(new NoFilter);
  }

  return filter_ptr->filter(shape_output, pose_output);
}

bool ShapeEstimator::applyCorrector(
  const uint8_t label, const bool use_reference_yaw,
  autoware_auto_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
{
  std::unique_ptr<ShapeEstimationCorrectorInterface> corrector_ptr;
  if (label == Label::CAR) {
    corrector_ptr.reset(new CarCorrector(use_reference_yaw));
  } else if (label == Label::BUS) {
    corrector_ptr.reset(new BusCorrector(use_reference_yaw));
  } else if (label == Label::TRUCK || label == Label::TRAILER) {
    corrector_ptr.reset(new TruckCorrector(use_reference_yaw));
  } else {
    corrector_ptr.reset(new NoCorrector);
  }

  return corrector_ptr->correct(shape_output, pose_output);
}
