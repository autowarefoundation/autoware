// Copyright 2023 TIER IV, Inc.
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

#ifndef TENSORRT_YOLOX__PREPROCESS_HPP_
#define TENSORRT_YOLOX__PREPROCESS_HPP_

#include <cublas_v2.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <curand.h>

namespace tensorrt_yolox
{
/**
 * @brief Resize a image using bilinear interpolation on gpus
 * @param[out] dst Resized image
 * @param[in] src image
 * @param[in] d_w width for resized image
 * @param[in] d_h height for resized image
 * @param[in] d_c channel for resized image
 * @param[in] s_w width for input image
 * @param[in] s_h height for input image
 * @param[in] s_c channel for input image
 * @param[in] stream cuda stream
 */
extern void resize_bilinear_gpu(
  unsigned char * dst, unsigned char * src, int d_w, int d_h, int d_c, int s_w, int s_h, int s_c,
  cudaStream_t stream);

/**
 * @brief Letterbox a image on gpus
 * @param[out] dst letterboxed image
 * @param[in] src image
 * @param[in] d_w width for letterboxing
 * @param[in] d_h height foletterboxing
 * @param[in] d_c channel foletterboxing
 * @param[in] s_w width for input image
 * @param[in] s_h height for input image
 * @param[in] s_c channel for input image
 * @param[in] stream cuda stream
 */
extern void letterbox_gpu(
  unsigned char * dst, unsigned char * src, int d_w, int d_h, int d_c, int s_w, int s_h, int s_c,
  cudaStream_t stream);

/**
 * @brief NHWC to NHWC conversion
 * @param[out] dst converted image
 * @param[in] src image
 * @param[in] d_w width for a image
 * @param[in] d_h height for a image
 * @param[in] d_c channel for a image
 * @param[in] stream cuda stream
 */
extern void nchw_to_nhwc_gpu(
  unsigned char * dst, unsigned char * src, int d_w, int d_h, int d_c, cudaStream_t stream);

/**
 * @brief Unsigned char to float32 for inference
 * @param[out] dst32 converted image
 * @param[in] src image
 * @param[in] d_w width for a image
 * @param[in] d_h height for a image
 * @param[in] d_c channel for a image
 * @param[in] stream cuda stream
 */
extern void to_float_gpu(
  float * dst32, unsigned char * src, int d_w, int d_h, int d_c, cudaStream_t stream);

/**
 * @brief Resize and letterbox a image using bilinear interpolation on gpus
 * @param[out] dst processsed image
 * @param[in] src image
 * @param[in] d_w width for output
 * @param[in] d_h height for output
 * @param[in] d_c channel for output
 * @param[in] s_w width for input
 * @param[in] s_h height for input
 * @param[in] s_c channel for input
 * @param[in] stream cuda stream
 */
extern void resize_bilinear_letterbox_gpu(
  unsigned char * dst, unsigned char * src, int d_w, int d_h, int d_c, int s_w, int s_h, int s_c,
  cudaStream_t stream);

/**
 * @brief Optimized preprocessing including resize, letterbox, nhwc2nchw, toFloat and normalization
 * for YOLOX on gpus
 * @param[out] dst processsed image
 * @param[in] src image
 * @param[in] d_w width for output
 * @param[in] d_h height for output
 * @param[in] d_c channel for output
 * @param[in] s_w width for input
 * @param[in] s_h height for input
 * @param[in] s_c channel for input
 * @param[in] norm normalization
 * @param[in] stream cuda stream
 */
extern void resize_bilinear_letterbox_nhwc_to_nchw32_gpu(
  float * dst, unsigned char * src, int d_w, int d_h, int d_c, int s_w, int s_h, int s_c,
  float norm, cudaStream_t stream);
}  // namespace tensorrt_yolox

#endif  // TENSORRT_YOLOX__PREPROCESS_HPP_
