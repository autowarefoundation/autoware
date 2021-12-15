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

#ifndef HAD_MAP_UTILS__VISIBILITY_CONTROL_HPP_
#define HAD_MAP_UTILS__VISIBILITY_CONTROL_HPP_

#if defined(_MSC_VER) && defined(_WIN64)
  #if defined(HAD_MAP_UTILS_BUILDING_DLL) || defined(HAD_MAP_UTILS_EXPORTS)
    #define HAD_MAP_UTILS_PUBLIC __declspec(dllexport)
    #define HAD_MAP_UTILS_LOCAL
  #else  // defined(HAD_MAP_UTILS_BUILDING_DLL) || defined(HAD_MAP_UTILS_EXPORTS)
    #define HAD_MAP_UTILS_PUBLIC __declspec(dllimport)
    #define HAD_MAP_UTILS_LOCAL
  #endif  // defined(HAD_MAP_UTILS_BUILDING_DLL) || defined(HAD_MAP_UTILS_EXPORTS)
#elif defined(__GNUC__) && defined(__linux__)
  #define HAD_MAP_UTILS_PUBLIC __attribute__((visibility("default")))
  #define HAD_MAP_UTILS_LOCAL __attribute__((visibility("hidden")))
#elif defined(__GNUC__) && defined(__APPLE__)
  #define HAD_MAP_UTILS_PUBLIC __attribute__((visibility("default")))
  #define HAD_MAP_UTILS_LOCAL __attribute__((visibility("hidden")))
#else  // !(defined(__GNUC__) && defined(__APPLE__))
  #error "Unsupported Build Configuration"
#endif  // _MSC_VER

#endif  // HAD_MAP_UTILS__VISIBILITY_CONTROL_HPP_
