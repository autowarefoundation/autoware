// Copyright 2021-2022 Arm Ltd.
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

#ifndef LIDAR_APOLLO_SEGMENTATION_TVM_NODES__VISIBILITY_CONTROL_HPP_
#define LIDAR_APOLLO_SEGMENTATION_TVM_NODES__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
#if defined(LIDAR_APOLLO_SEGMENTATION_TVM_NODES_BUILDING_DLL) || \
  defined(LIDAR_APOLLO_SEGMENTATION_TVM_NODES_EXPORTS)
#define LIDAR_APOLLO_SEGMENTATION_TVM_NODES_PUBLIC __declspec(dllexport)
#define LIDAR_APOLLO_SEGMENTATION_TVM_NODES_LOCAL
#else
#define LIDAR_APOLLO_SEGMENTATION_TVM_NODES_PUBLIC __declspec(dllimport)
#define LIDAR_APOLLO_SEGMENTATION_TVM_NODES_LOCAL
#endif
#elif defined(__linux__)
#define LIDAR_APOLLO_SEGMENTATION_TVM_NODES_PUBLIC __attribute__((visibility("default")))
#define LIDAR_APOLLO_SEGMENTATION_TVM_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define LIDAR_APOLLO_SEGMENTATION_TVM_NODES_PUBLIC __attribute__((visibility("default")))
#define LIDAR_APOLLO_SEGMENTATION_TVM_NODES_LOCAL __attribute__((visibility("hidden")))
#else
#error "Unsupported Build Configuration"
#endif

#endif  // LIDAR_APOLLO_SEGMENTATION_TVM_NODES__VISIBILITY_CONTROL_HPP_
