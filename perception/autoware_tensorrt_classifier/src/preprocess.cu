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
#include <autoware/tensorrt_classifier/preprocess.h>
#include <stdio.h>
#include <stdlib.h>

#include <algorithm>

#define BLOCK 512

#define MIN(x, y) x < y ? x : y

dim3 cuda_gridsize(size_t n)
{
  size_t k = (n - 1) / BLOCK + 1;
  size_t x = k;
  size_t y = 1;
  if (x > 65535) {
    x = ceil(sqrt(k));
    y = (n - 1) / (x * BLOCK) + 1;
  }
  dim3 d;
  d.x = x;
  d.y = y;
  d.z = 1;
  return d;
}

__device__ double lerp1d(int a, int b, float w)
{
  return fma(w, (float)b, fma(-w, (float)a, (float)a));
}

__device__ float lerp2d(int f00, int f01, int f10, int f11, float centroid_h, float centroid_w)
{
  centroid_w = (1 + lroundf(centroid_w) - centroid_w) / 2;
  centroid_h = (1 + lroundf(centroid_h) - centroid_h) / 2;

  float r0, r1, r;
  r0 = lerp1d(f00, f01, centroid_w);
  r1 = lerp1d(f10, f11, centroid_w);

  r = lerp1d(r0, r1, centroid_h);  //+ 0.00001
  return r;
}

__global__ void resize_bilinear_kernel(
  int N, unsigned char * dst_img, unsigned char * src_img, int dst_h, int dst_w, int src_h,
  int src_w, float stride_h, float stride_w)
{
  // NHWC
  int index = (blockIdx.x + blockIdx.y * gridDim.x) * blockDim.x + threadIdx.x;

  if (index >= N) return;
  int C = 3;
  int W = dst_w;

  int c = 0;
  int n = 0;

  int w = index % W;
  int h = index / W;

  float centroid_h, centroid_w;
  centroid_h = stride_h * (float)(h + 0.5);
  centroid_w = stride_w * (float)(w + 0.5);

  int f00, f01, f10, f11;

  int src_h_idx = lroundf(centroid_h) - 1;
  int src_w_idx = lroundf(centroid_w) - 1;
  if (src_h_idx < 0) {
    src_h_idx = 0;
  }
  if (src_w_idx < 0) {
    src_w_idx = 0;
  }

  index = C * w + C * W * h;
  // Unroll
  for (c = 0; c < C; c++) {
    f00 = n * src_h * src_w * C + src_h_idx * src_w * C + src_w_idx * C + c;
    f01 = n * src_h * src_w * C + src_h_idx * src_w * C + (src_w_idx + 1) * C + c;
    f10 = n * src_h * src_w * C + (src_h_idx + 1) * src_w * C + src_w_idx * C + c;
    f11 = n * src_h * src_w * C + (src_h_idx + 1) * src_w * C + (src_w_idx + 1) * C + c;

    float rs = lroundf(lerp2d(
      (int)src_img[f00], (int)src_img[f01], (int)src_img[f10], (int)src_img[f11], centroid_h,
      centroid_w));
    dst_img[index + c] = (unsigned char)rs;
  }
}

void resize_bilinear_gpu(
  unsigned char * dst, unsigned char * src, int d_w, int d_h, int d_c, int s_w, int s_h, int s_c,
  cudaStream_t stream)
{
  int N = d_w * d_h;
  float stride_h = (float)s_h / (float)d_h;
  float stride_w = (float)s_w / (float)d_w;

  resize_bilinear_kernel<<<cuda_gridsize(N), BLOCK, 0, stream>>>(
    N, dst, src, d_h, d_w, s_h, s_w, stride_h, stride_w);
}

__global__ void letterbox_kernel(
  int N, unsigned char * dst_img, unsigned char * src_img, int dst_h, int dst_w, int src_h,
  int src_w, float scale, int letter_bot, int letter_right)
{
  // NHWC
  int index = (blockIdx.x + blockIdx.y * gridDim.x) * blockDim.x + threadIdx.x;

  if (index >= N) return;
  int C = 3;
  int W = dst_w;
  int c = 0;

  int w = index % W;
  int h = index / W;

  index = (C * w) + (C * W * h);
  // Unroll
  int index2 = (C * w) + (C * src_w * h);
  for (c = 0; c < C; c++) {
    dst_img[index + c] =
      (w >= letter_right || h >= letter_bot) ? (unsigned int)114 : src_img[index2 + c];
  }
}

void letterbox_gpu(
  unsigned char * dst, unsigned char * src, int d_w, int d_h, int d_c, int s_w, int s_h, int s_c,
  cudaStream_t stream)
{
  int N = d_w * d_h;
  const float scale = std::min(d_w / (float)s_w, d_h / (float)s_h);
  int r_h = (int)(scale * s_h);
  int r_w = (int)(scale * s_w);

  float stride_h = (float)s_h / (float)r_h;
  float stride_w = (float)s_w / (float)r_w;
  letterbox_kernel<<<cuda_gridsize(N), BLOCK, 0, stream>>>(
    N, dst, src, d_h, d_w, r_h, r_w, 1.0 / scale, r_h, r_w);
}

__global__ void NHWC2NCHW_kernel(
  int N, unsigned char * dst_img, unsigned char * src_img, int height, int width)
{
  // NHWC
  int index = (blockIdx.x + blockIdx.y * gridDim.x) * blockDim.x + threadIdx.x;

  if (index >= N) return;
  int chan = 3;
  int c = 0;
  int x = index % width;
  int y = index / width;
  int src_index = 0;
  int dst_index = 0;
  for (c = 0; c < chan; c++) {
    src_index = c + (chan * x) + (chan * width * y);
    dst_index = x + (width * y) + (width * height * c);
    dst_img[dst_index] = src_img[src_index];
  }
}

void NHWC2NCHW_gpu(
  unsigned char * dst, unsigned char * src, int d_w, int d_h, int d_c, cudaStream_t stream)
{
  int N = d_w * d_h;
  NHWC2NCHW_kernel<<<cuda_gridsize(N), BLOCK, 0, stream>>>(N, dst, src, d_h, d_w);
}

__global__ void NCHW2NHWC_kernel(
  int N, unsigned char * dst, unsigned char * src, int height, int width)
{
  // NHWC
  int index = (blockIdx.x + blockIdx.y * gridDim.x) * blockDim.x + threadIdx.x;

  if (index >= N) return;
  int chan = 3;
  int c = 0;
  int x = index % width;
  int y = index / width;
  int src_index = 0;
  int dst_index = 0;
  for (c = 0; c < chan; c++) {
    // NHWC
    dst_index = c + (chan * x) + (chan * width * y);
    // NCHW
    src_index = x + (width * y) + (width * height * c);
    dst[dst_index] = src[src_index];
  }
}

void NCHW2NHWC_gpu(
  unsigned char * dst, unsigned char * src, int d_w, int d_h, int d_c, cudaStream_t stream)
{
  int N = d_w * d_h;
  NCHW2NHWC_kernel<<<cuda_gridsize(N), BLOCK, 0, stream>>>(N, dst, src, d_h, d_w);
}

__global__ void toFloat_kernel(int N, float * dst32, unsigned char * src8, int height, int width)
{
  // NHWC
  int index = (blockIdx.x + blockIdx.y * gridDim.x) * blockDim.x + threadIdx.x;

  if (index >= N) return;
  int chan = 3;
  int c = 0;
  int x = index % width;
  int y = index / width;
  int dst_index = 0;
  for (c = 0; c < chan; c++) {
    // NCHW
    dst_index = x + (width * y) + (width * height * c);
    dst32[dst_index] = (float)(src8[dst_index]);
  }
}

void toFloat_gpu(float * dst32, unsigned char * src, int d_w, int d_h, int d_c, cudaStream_t stream)
{
  int N = d_w * d_h;
  toFloat_kernel<<<cuda_gridsize(N), BLOCK, 0, stream>>>(N, dst32, src, d_h, d_w);
}

__global__ void resize_bilinear_letterbox_kernel(
  int N, unsigned char * dst_img, unsigned char * src_img, int dst_h, int dst_w, int src_h,
  int src_w, float scale, int letter_bot, int letter_right)
{
  // NHWC
  int index = (blockIdx.x + blockIdx.y * gridDim.x) * blockDim.x + threadIdx.x;

  if (index >= N) return;
  int C = 3;  // # ChannelDim
  int W = dst_w;
  int c = 0;
  int n = 0;  // index / (C*W*H);

  int w = index % W;
  int h = index / W;

  float centroid_h, centroid_w;
  centroid_h = scale * (float)(h + 0.5);
  centroid_w = scale * (float)(w + 0.5);

  int f00, f01, f10, f11;

  int src_h_idx = (int)lroundf(centroid_h) - 1;
  int src_w_idx = (int)lroundf(centroid_w) - 1;
  if (src_h_idx < 0) {
    src_h_idx = 0;
  }
  if (src_w_idx < 0) {
    src_w_idx = 0;
  }
  if (src_h_idx >= src_h) {
    src_h_idx = src_h - 1;
  }
  if (src_w_idx >= src_w) {
    src_w_idx = src_w - 1;
  }

  index = (C * w) + (C * W * h);
  // Unroll
  for (c = 0; c < C; c++) {
    f00 = n * src_h * src_w * C + src_h_idx * src_w * C + src_w_idx * C + c;
    f01 = n * src_h * src_w * C + src_h_idx * src_w * C + (src_w_idx + 1) * C + c;
    f10 = n * src_h * src_w * C + (src_h_idx + 1) * src_w * C + src_w_idx * C + c;
    f11 = n * src_h * src_w * C + (src_h_idx + 1) * src_w * C + (src_w_idx + 1) * C + c;

    float rs = lroundf(lerp2d(
      (int)src_img[f00], (int)src_img[f01], (int)src_img[f10], (int)src_img[f11], centroid_h,
      centroid_w));
    dst_img[index + c] = (unsigned char)rs;
    dst_img[index + c] = (h >= letter_bot) ? (unsigned int)114 : dst_img[index + c];
    dst_img[index + c] = (w >= letter_right) ? (unsigned int)114 : dst_img[index + c];
  }
}

void resize_bilinear_letterbox_gpu(
  unsigned char * dst, unsigned char * src, int d_w, int d_h, int d_c, int s_w, int s_h, int s_c,
  cudaStream_t stream)
{
  int N = d_w * d_h;
  const float scale = std::min(d_w / (float)s_w, d_h / (float)s_h);
  int r_h = (int)(scale * s_h);
  int r_w = (int)(scale * s_w);
  float stride_h = (float)s_h / (float)r_h;
  float stride_w = (float)s_w / (float)r_w;
  resize_bilinear_letterbox_kernel<<<cuda_gridsize(N), BLOCK, 0, stream>>>(
    N, dst, src, d_h, d_w, s_h, s_w, 1.0 / scale, r_h, r_w);
}

__global__ void resize_bilinear_letterbox_nhwc_to_nchw32_kernel(
  int N, float * dst_img, unsigned char * src_img, int dst_h, int dst_w, int src_h, int src_w,
  float scale, int letter_bot, int letter_right, float norm)
{
  // NHWC
  int index = (blockIdx.x + blockIdx.y * gridDim.x) * blockDim.x + threadIdx.x;

  if (index >= N) return;
  int C = 3;
  int H = dst_h;
  int W = dst_w;
  int c = 0;

  int w = index % W;
  int h = index / W;

  float centroid_h, centroid_w;
  centroid_h = scale * (float)(h + 0.5);
  centroid_w = scale * (float)(w + 0.5);

  int f00, f01, f10, f11;

  int src_h_idx = lroundf(centroid_h) - 1;
  int src_w_idx = lroundf(centroid_w) - 1;
  src_h_idx = (src_h_idx < 0) ? 0 : src_h_idx;
  src_h_idx = (src_h_idx >= (src_h - 1)) ? src_h - 2 : src_h_idx;
  src_w_idx = (src_w_idx < 0) ? 0 : src_w_idx;
  src_w_idx = (src_w_idx >= (src_w - 1)) ? src_w - 2 : src_w_idx;
  // Unroll
  int stride = src_w * C;
  for (c = 0; c < C; c++) {
    f00 = src_h_idx * stride + src_w_idx * C + c;
    f01 = src_h_idx * stride + (src_w_idx + 1) * C + c;
    f10 = (src_h_idx + 1) * stride + src_w_idx * C + c;
    f11 = (src_h_idx + 1) * stride + (src_w_idx + 1) * C + c;

    float rs = lroundf(lerp2d(
      (int)src_img[f00], (int)src_img[f01], (int)src_img[f10], (int)src_img[f11], centroid_h,
      centroid_w));

    // NHCW
    int dst_index = w + (W * h) + (W * H * c);

    dst_img[dst_index] = (float)rs;
    dst_img[dst_index] = (h >= letter_bot) ? 114.0 : dst_img[dst_index];
    dst_img[dst_index] = (w >= letter_right) ? 114.0 : dst_img[dst_index];
    dst_img[dst_index] *= norm;
  }
}

void resize_bilinear_letterbox_nhwc_to_nchw32_gpu(
  float * dst, unsigned char * src, int d_w, int d_h, int d_c, int s_w, int s_h, int s_c,
  float norm, cudaStream_t stream)
{
  int N = d_w * d_h;
  const float scale = std::min(d_w / (float)s_w, d_h / (float)s_h);
  int r_h = scale * s_h;
  int r_w = scale * s_w;
  float stride_h = (float)s_h / (float)r_h;
  float stride_w = (float)s_w / (float)r_w;

  resize_bilinear_letterbox_nhwc_to_nchw32_kernel<<<cuda_gridsize(N), BLOCK, 0, stream>>>(
    N, dst, src, d_h, d_w, s_h, s_w, 1.0 / scale, r_h, r_w, norm);
}

__global__ void resize_bilinear_letterbox_nhwc_to_nchw32_batch_kernel(
  int N, float * dst_img, unsigned char * src_img, int dst_h, int dst_w, int src_h, int src_w,
  float scale, int letter_bot, int letter_right, float norm, int batch)
{
  // NHWC
  int index = (blockIdx.x + blockIdx.y * gridDim.x) * blockDim.x + threadIdx.x;

  if (index >= N) return;
  int C = 3;
  int H = dst_h;
  int W = dst_w;
  int c = 0;
  // w * h * b

  int w = index % W;
  int h = index / (W);
  float centroid_h, centroid_w;
  centroid_h = scale * (float)(h + 0.5);
  centroid_w = scale * (float)(w + 0.5);

  int f00, f01, f10, f11;

  int src_h_idx = lroundf(centroid_h) - 1;
  int src_w_idx = lroundf(centroid_w) - 1;
  src_h_idx = (src_h_idx < 0) ? 0 : src_h_idx;
  src_h_idx = (src_h_idx >= (src_h - 1)) ? src_h - 2 : src_h_idx;
  src_w_idx = (src_w_idx < 0) ? 0 : src_w_idx;
  src_w_idx = (src_w_idx >= (src_w - 1)) ? src_w - 2 : src_w_idx;
  // Unroll
  int stride = src_w * C;
  int b_stride = src_h * src_w * C;
  int b;
  const float mean[3] = {123.675, 116.28, 103.53};
  const float std[3] = {58.395, 57.12, 57.375};
  for (b = 0; b < batch; b++) {
    for (c = 0; c < C; c++) {
      // NHWC
      // bgr2rgb
      f00 = src_h_idx * stride + src_w_idx * C + (C - c - 1) + b * b_stride;

      f01 = src_h_idx * stride + (src_w_idx + 1) * C + (C - c - 1) + b * b_stride;

      f10 = (src_h_idx + 1) * stride + src_w_idx * C + (C - c - 1) + b * b_stride;
      f11 = (src_h_idx + 1) * stride + (src_w_idx + 1) * C + (C - c - 1) + b * b_stride;

      float rs = lroundf(lerp2d(
        (int)src_img[f00], (int)src_img[f01], (int)src_img[f10], (int)src_img[f11], centroid_h,
        centroid_w));

      // NCHW
      int dst_index = b * (W * H * C) + (W * H * c) + (W * h) + w;

      dst_img[dst_index] = (float)rs;
      dst_img[dst_index] = (h >= letter_bot) ? 0.0 : dst_img[dst_index];
      dst_img[dst_index] = (w >= letter_right) ? 0.0 : dst_img[dst_index];
      dst_img[dst_index] -= mean[c];
      dst_img[dst_index] /= std[c];
    }
  }
}

void resize_bilinear_letterbox_nhwc_to_nchw32_batch_gpu(
  float * dst, unsigned char * src, int d_w, int d_h, int d_c, int s_w, int s_h, int s_c, int batch,
  float norm, cudaStream_t stream)
{
  int N = d_w * d_h;
  const float scale = std::min(d_w / (float)s_w, d_h / (float)s_h);
  int r_h = scale * s_h;
  int r_w = scale * s_w;
  float stride_h = (float)s_h / (float)r_h;
  float stride_w = (float)s_w / (float)r_w;

  resize_bilinear_letterbox_nhwc_to_nchw32_batch_kernel<<<cuda_gridsize(N), BLOCK, 0, stream>>>(
    N, dst, src, d_h, d_w, s_h, s_w, 1.0 / scale, r_h, r_w, norm, batch);
  /*
  int b
  for (b = 0; b < batch; b++) {
    int index_dst = b * d_w * d_h * d_c;
    int index_src = b * s_w * s_h * s_c;
    resize_bilinear_letterbox_nhwc_to_nchw32_kernel<<<cuda_gridsize(N), BLOCK, 0, stream>>>(N,
  &dst[index_dst], &src[index_src], d_h, d_w, s_h, s_w, 1.0/scale, r_h, r_w, norm
                                                                                       );
  }
  */
}

__global__ void crop_resize_bilinear_letterbox_nhwc_to_nchw32_batch_kernel(
  int N, float * dst_img, unsigned char * src_img, int dst_h, int dst_w, int src_h, int src_w,
  Roi * d_roi, float norm, int batch)
{
  // NHWC
  int index = (blockIdx.x + blockIdx.y * gridDim.x) * blockDim.x + threadIdx.x;

  if (index >= N) return;
  int C = 3;
  int H = dst_h;
  int W = dst_w;
  int c = 0;
  // w * h * b

  int w = index % W;
  int h = index / (W);

  int f00, f01, f10, f11;

  int b;
  for (b = 0; b < batch; b++) {
    float centroid_h, centroid_w;
    int crop_h = d_roi[b].h;
    int crop_w = d_roi[b].w;
    int crop_y = d_roi[b].y;
    int crop_x = d_roi[b].x;
    float scale = (MIN(W / (float)crop_w, H / (float)crop_h));
    int letter_bot = (int)(scale * crop_h);
    int letter_right = (int)(scale * crop_w);
    scale = 1.0 / scale;
    centroid_h = scale * (float)(h + 0.5);
    centroid_w = scale * (float)(w + 0.5);
    int src_h_idx = lroundf(centroid_h) - 1;
    int src_w_idx = lroundf(centroid_w) - 1;
    src_h_idx = (src_h_idx < 0) ? 0 : src_h_idx;
    src_h_idx = (src_h_idx >= (crop_h - 1)) ? crop_h - 2 : src_h_idx;
    src_w_idx = (src_w_idx < 0) ? 0 : src_w_idx;
    src_w_idx = (src_w_idx >= (crop_w - 1)) ? crop_w - 2 : src_w_idx;
    // Unroll
    int stride = src_w * C;
    int b_stride = src_h * src_w * C;
    src_w_idx += crop_x;
    src_h_idx += crop_y;

    for (c = 0; c < C; c++) {
      // NHWC
      f00 = src_h_idx * stride + src_w_idx * C + c + b * b_stride;
      f01 = src_h_idx * stride + (src_w_idx + 1) * C + c + b * b_stride;
      f10 = (src_h_idx + 1) * stride + src_w_idx * C + c + b * b_stride;
      f11 = (src_h_idx + 1) * stride + (src_w_idx + 1) * C + c + b * b_stride;

      float rs = lroundf(lerp2d(
        (int)src_img[f00], (int)src_img[f01], (int)src_img[f10], (int)src_img[f11], centroid_h,
        centroid_w));

      // NCHW
      int dst_index = w + (W * h) + (W * H * c) + b * (W * H * C);

      dst_img[dst_index] = (float)rs;
      dst_img[dst_index] = (h >= letter_bot) ? 114.0 : dst_img[dst_index];
      dst_img[dst_index] = (w >= letter_right) ? 114.0 : dst_img[dst_index];
      dst_img[dst_index] *= norm;
    }
  }
}

void crop_resize_bilinear_letterbox_nhwc_to_nchw32_batch_gpu(
  float * dst, unsigned char * src, int d_w, int d_h, int d_c, Roi * d_roi, int s_w, int s_h,
  int s_c, int batch, float norm, cudaStream_t stream)
{
  int N = d_w * d_h;
  crop_resize_bilinear_letterbox_nhwc_to_nchw32_batch_kernel<<<
    cuda_gridsize(N), BLOCK, 0, stream>>>(N, dst, src, d_h, d_w, s_h, s_w, d_roi, norm, batch);
}

__global__ void multi_scale_resize_bilinear_letterbox_nhwc_to_nchw32_batch_kernel(
  int N, float * dst_img, unsigned char * src_img, int dst_h, int dst_w, int src_h, int src_w,
  Roi * d_roi, float norm, int batch)
{
  int index = (blockIdx.x + blockIdx.y * gridDim.x) * blockDim.x + threadIdx.x;

  if (index >= N) return;
  int C = 3;
  int H = dst_h;
  int W = dst_w;
  int c = 0;
  // w * h * b
  int w = index % W;
  int h = index / (W);
  int f00, f01, f10, f11, b;
  for (b = 0; b < batch; b++) {
    float centroid_h, centroid_w;
    int crop_h = d_roi[b].h;
    int crop_w = d_roi[b].w;
    int crop_y = d_roi[b].y;
    int crop_x = d_roi[b].x;
    float scale = (MIN(W / (float)crop_w, H / (float)crop_h));
    int letter_bot = (int)(scale * crop_h);
    int letter_right = (int)(scale * crop_w);
    int src_h_idx, src_w_idx, stride;
    scale = 1.0 / scale;
    centroid_h = scale * (float)(h + 0.5);
    centroid_w = scale * (float)(w + 0.5);
    src_h_idx = lroundf(centroid_h) - 1;
    src_w_idx = lroundf(centroid_w) - 1;
    src_h_idx = (src_h_idx < 0) ? 0 : src_h_idx;
    src_h_idx = (src_h_idx >= (crop_h - 1)) ? crop_h - 2 : src_h_idx;
    src_w_idx = (src_w_idx < 0) ? 0 : src_w_idx;
    src_w_idx = (src_w_idx >= (crop_w - 1)) ? crop_w - 2 : src_w_idx;
    // Unroll
    stride = src_w * C;
    src_w_idx += crop_x;
    src_h_idx += crop_y;
    for (c = 0; c < C; c++) {
      float rs;
      int dst_index;
      // NHWC
      f00 = src_h_idx * stride + src_w_idx * C + c;
      f01 = src_h_idx * stride + (src_w_idx + 1) * C + c;
      f10 = (src_h_idx + 1) * stride + src_w_idx * C + c;
      f11 = (src_h_idx + 1) * stride + (src_w_idx + 1) * C + c;
      rs = lroundf(lerp2d(
        (int)src_img[f00], (int)src_img[f01], (int)src_img[f10], (int)src_img[f11], centroid_h,
        centroid_w));
      // NCHW
      dst_index = w + (W * h) + (W * H * c) + b * (W * H * C);
      dst_img[dst_index] = (float)rs;
      dst_img[dst_index] = (h >= letter_bot) ? 114.0 : dst_img[dst_index];
      dst_img[dst_index] = (w >= letter_right) ? 114.0 : dst_img[dst_index];
      dst_img[dst_index] *= norm;
    }
  }
}

void multi_scale_resize_bilinear_letterbox_nhwc_to_nchw32_batch_gpu(
  float * dst, unsigned char * src, int d_w, int d_h, int d_c, Roi * d_roi, int s_w, int s_h,
  int s_c, int batch, float norm, cudaStream_t stream)
{
  int N = d_w * d_h;
  multi_scale_resize_bilinear_letterbox_nhwc_to_nchw32_batch_kernel<<<
    cuda_gridsize(N), BLOCK, 0, stream>>>(N, dst, src, d_h, d_w, s_h, s_w, d_roi, norm, batch);
}
