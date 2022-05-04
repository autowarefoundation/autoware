// Copyright 2017-2019 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef GEOMETRY__VISIBILITY_CONTROL_HPP_
#define GEOMETRY__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
#if defined(GEOMETRY_BUILDING_DLL) || defined(GEOMETRY_EXPORTS)
#define GEOMETRY_PUBLIC __declspec(dllexport)
#define GEOMETRY_LOCAL
#else  // defined(GEOMETRY_BUILDING_DLL) || defined(GEOMETRY_EXPORTS)
#define GEOMETRY_PUBLIC __declspec(dllimport)
#define GEOMETRY_LOCAL
#endif  // defined(GEOMETRY_BUILDING_DLL) || defined(GEOMETRY_EXPORTS)
#elif defined(__linux__)
#define GEOMETRY_PUBLIC __attribute__((visibility("default")))
#define GEOMETRY_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define GEOMETRY_PUBLIC __attribute__((visibility("default")))
#define GEOMETRY_LOCAL __attribute__((visibility("hidden")))
#elif defined(QNX)
#define GEOMETRY_PUBLIC __attribute__((visibility("default")))
#define GEOMETRY_LOCAL __attribute__((visibility("hidden")))
#else  // defined(__linux__)
#error "Unsupported Build Configuration"
#endif  // defined(__WIN32)
#endif  // GEOMETRY__VISIBILITY_CONTROL_HPP_
