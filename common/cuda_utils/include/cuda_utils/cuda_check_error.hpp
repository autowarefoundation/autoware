// Copyright 2022 Tier IV, Inc.
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

// This code is licensed under CC0 1.0 Universal (Public Domain).
// You can use this without any limitation.
// https://creativecommons.org/publicdomain/zero/1.0/deed.en
// borrowed from https://proc-cpuinfo.fixstars.com/2019/02/cuda_smart_pointer/

#ifndef CUDA_UTILS__CUDA_CHECK_ERROR_HPP_
#define CUDA_UTILS__CUDA_CHECK_ERROR_HPP_

#include <cuda_runtime_api.h>

#include <sstream>
#include <stdexcept>

namespace cuda_utils
{
template <typename F, typename N>
void cuda_check_error(const ::cudaError_t e, F && f, N && n)
{
  if (e != ::cudaSuccess) {
    std::stringstream s;
    s << ::cudaGetErrorName(e) << " (" << e << ")@" << f << "#L" << n << ": "
      << ::cudaGetErrorString(e);
    throw std::runtime_error{s.str()};
  }
}
}  // namespace cuda_utils

#define CHECK_CUDA_ERROR(e) (cuda_utils::cuda_check_error(e, __FILE__, __LINE__))

#endif  // CUDA_UTILS__CUDA_CHECK_ERROR_HPP_
