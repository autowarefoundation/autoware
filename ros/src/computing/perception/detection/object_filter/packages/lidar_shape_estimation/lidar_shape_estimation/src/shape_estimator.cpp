/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 *
 *
 * v1.0 Yukihiro Saito
 */

#include "lidar_shape_estimation/shape_estimator.hpp"
#include "lidar_shape_estimation/model_interface.hpp"
#include "model/bounding_box.hpp"
#include "model/convex_hull.hpp"
#include "model/cylinder.hpp"
#include <memory>
#include <iostream>

ShapeEstimator::ShapeEstimator()
{
}

bool ShapeEstimator::getShapeAndPose(const std::string& label, const pcl::PointCloud<pcl::PointXYZ>& cluster,
                                     autoware_msgs::DetectedObject& output)
{
  if (cluster.empty())
    return false;
  std::unique_ptr<ShapeEstimationModelInterface> model_ptr;
  if (label == "car" || label == "vehicle" || label == "truck" || label == "bus")
  {
    model_ptr.reset(new BoundingBoxModel);
  }
  else if (label == "person")
  {
    model_ptr.reset(new CylinderModel);
  }
  else if (label == "motorbike")
  {
    model_ptr.reset(new BoundingBoxModel);
  }
  else if (label == "bicycle")
  {
    model_ptr.reset(new BoundingBoxModel);
  }
  else
  {
    //        model_ptr.reset(new CylinderModel);
    model_ptr.reset(new BoundingBoxModel);
  };

  return model_ptr->estimate(cluster, output);
}
