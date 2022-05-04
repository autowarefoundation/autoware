// Copyright 2021 The Autoware Foundation
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

#ifndef SIMPLE_PLANNING_SIMULATOR__VISIBILITY_CONTROL_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
#if defined(PLANNING_SIMULATOR_BUILDING_DLL) || defined(PLANNING_SIMULATOR_EXPORTS)
#define PLANNING_SIMULATOR_PUBLIC __declspec(dllexport)
#define PLANNING_SIMULATOR_LOCAL
#else  // defined(PLANNING_SIMULATOR_BUILDING_DLL) ||
       // defined(PLANNING_SIMULATOR_EXPORTS)
#define PLANNING_SIMULATOR_PUBLIC __declspec(dllimport)
#define PLANNING_SIMULATOR_LOCAL
#endif  // defined(PLANNING_SIMULATOR_BUILDING_DLL) ||
        // defined(PLANNING_SIMULATOR_EXPORTS)
#elif defined(__linux__)
#define PLANNING_SIMULATOR_PUBLIC __attribute__((visibility("default")))
#define PLANNING_SIMULATOR_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define PLANNING_SIMULATOR_PUBLIC __attribute__((visibility("default")))
#define PLANNING_SIMULATOR_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
#error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // SIMPLE_PLANNING_SIMULATOR__VISIBILITY_CONTROL_HPP_
