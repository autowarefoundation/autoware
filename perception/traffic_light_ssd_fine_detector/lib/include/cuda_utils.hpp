// Copyright 2020 Tier IV, Inc.
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

/*
 * This code is licensed under CC0 1.0 Universal (Public Domain).
 * You can use this without any limitation.
 * https://creativecommons.org/publicdomain/zero/1.0/deed.en
 */

#ifndef CUDA_UTILS_HPP_
#define CUDA_UTILS_HPP_

#include <./cuda_runtime_api.h>

#include <memory>
#include <sstream>
#include <stdexcept>
#include <type_traits>

#define CHECK_CUDA_ERROR(e) (cuda::check_error(e, __FILE__, __LINE__))

namespace cuda
{
void check_error(const ::cudaError_t e, decltype(__FILE__) f, decltype(__LINE__) n)
{
  if (e != ::cudaSuccess) {
    std::stringstream s;
    s << ::cudaGetErrorName(e) << " (" << e << ")@" << f << "#L" << n << ": "
      << ::cudaGetErrorString(e);
    throw std::runtime_error{s.str()};
  }
}

struct deleter
{
  void operator()(void * p) const { CHECK_CUDA_ERROR(::cudaFree(p)); }
};
template <typename T>
using unique_ptr = std::unique_ptr<T, deleter>;

// auto array = cuda::make_unique<float[]>(n);
// ::cudaMemcpy(array.get(), src_array, sizeof(float)*n, ::cudaMemcpyHostToDevice);
template <typename T>
typename std::enable_if<std::is_array<T>::value, cuda::unique_ptr<T>>::type make_unique(
  const std::size_t n)
{
  using U = typename std::remove_extent<T>::type;
  U * p;
  CHECK_CUDA_ERROR(::cudaMalloc(reinterpret_cast<void **>(&p), sizeof(U) * n));
  return cuda::unique_ptr<T>{p};
}

// auto value = cuda::make_unique<my_class>();
// ::cudaMemcpy(value.get(), src_value, sizeof(my_class), ::cudaMemcpyHostToDevice);
template <typename T>
cuda::unique_ptr<T> make_unique()
{
  T * p;
  CHECK_CUDA_ERROR(::cudaMalloc(reinterpret_cast<void **>(&p), sizeof(T)));
  return cuda::unique_ptr<T>{p};
}
}  // namespace cuda

#endif  // CUDA_UTILS_HPP_
