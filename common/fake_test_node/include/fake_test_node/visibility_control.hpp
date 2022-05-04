// Copyright 2021 Apex.AI, Inc.
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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#ifndef FAKE_TEST_NODE__VISIBILITY_CONTROL_HPP_
#define FAKE_TEST_NODE__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
#if defined(FAKE_TEST_NODE_BUILDING_DLL) || defined(FAKE_TEST_NODE_EXPORTS)
#define FAKE_TEST_NODE_PUBLIC __declspec(dllexport)
#define FAKE_TEST_NODE_LOCAL
#else  // defined(FAKE_TEST_NODE_BUILDING_DLL) || defined(FAKE_TEST_NODE_EXPORTS)
#define FAKE_TEST_NODE_PUBLIC __declspec(dllimport)
#define FAKE_TEST_NODE_LOCAL
#endif  // defined(FAKE_TEST_NODE_BUILDING_DLL) || defined(FAKE_TEST_NODE_EXPORTS)
#elif defined(__linux__)
#define FAKE_TEST_NODE_PUBLIC __attribute__((visibility("default")))
#define FAKE_TEST_NODE_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define FAKE_TEST_NODE_PUBLIC __attribute__((visibility("default")))
#define FAKE_TEST_NODE_LOCAL __attribute__((visibility("hidden")))
#else  // defined(__linux__)
#error "Unsupported Build Configuration"
#endif  // defined(__WIN32)

#endif  // FAKE_TEST_NODE__VISIBILITY_CONTROL_HPP_
