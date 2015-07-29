/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <vector>
#include <algorithm>

#include <cuda.h>

static void getDrvErrorCode(int error_id, std::string* str);

int main(void)
{
	int deviceCount;
	CUresult error_id = cuInit(0);

	error_id = cuDeviceGetCount(&deviceCount);

	if (error_id != CUDA_SUCCESS) {
#if CUDA_VERSION < 6000         // if CUDA version is under 6.0
        std::string error_string;
        getDrvErrorCode(error_id, &error_string);
#else
		const char *error_string;
		cuGetErrorString(error_id, &error_string);
#endif
		std::cerr << "Failed: cuDeviceGetCount()"
			  << " = "
			  << static_cast<int>(error_id)
			  << " -> "
			  << error_string
			  << std::endl;
		return -1;
	}

	if (deviceCount == 0) {
		std::cerr << "No CUDA GPU" << std::endl;
		return -1;
	}

	std::vector<int> capability_versions;
	for (int device = 0; device < deviceCount; ++device) {
		CUdevice devHandle;

		cuDeviceGet(&devHandle, device);

		int major = 0, minor = 0;
		cuDeviceComputeCapability(&major, &minor, devHandle);

		int capability_version = (10 * major) + minor;
		capability_versions.push_back(capability_version);
	}

	int min_version = *std::min_element(capability_versions.begin(),
					    capability_versions.end());

	std::cout << min_version << std::endl;
	return 0;
}

static void getDrvErrorCode(int error_id, std::string* str)
{
  switch (error_id)
    {
    case 0 :
      *str = "CUDA_SUCCESS";
      break;
    case 1 :
      *str = "CUDA_ERROR_INVALID_VALUE";
      break;
    case 2 :
      *str = "CUDA_ERROR_OUT_OF_MEMORY";
      break;
    case 3 :
      *str = "CUDA_ERROR_NOT_INITIALIZED";
      break;
    case 4 :
      *str = "CUDA_ERROR_DEINITIALIZED";
      break;
    case 5 :
      *str = "CUDA_ERROR_PROFILER_DISABLED";
      break;
    case 6 :
      *str = "CUDA_ERROR_PROFILER_NOT_INITIALIZED";
      break;
    case 7 :
      *str = "CUDA_ERROR_PROFILER_ALREADY_STARTED";
      break;
    case 8 :
      *str = "CUDA_ERROR_PROFILER_ALREADY_STOPPED";
      break;
    case 100 :
      *str = "CUDA_ERROR_NO_DEVICE (no CUDA-capable devices were detected)";
      break;
    case 101 :
      *str = "CUDA_ERROR_INVALID_DEVICE (device specified is not a valid CUDA device)";
      break;
    case 200 :
      *str = "CUDA_ERROR_INVALID_IMAGE";
      break;
    case 201 :
      *str = "CUDA_ERROR_INVALID_CONTEXT";
      break;
    case 202 :
      *str = "CUDA_ERROR_CONTEXT_ALREADY_CURRENT";
      break;
    case 205 :
      *str = "CUDA_ERROR_MAP_FAILED";
      break;
    case 206 :
      *str = "CUDA_ERROR_UNMAP_FAILED";
      break;
    case 207 :
      *str = "CUDA_ERROR_ARRAY_IS_MAPPED";
      break;
    case 208 :
      *str = "CUDA_ERROR_ALREADY_MAPPED";
      break;
    case 209 :
      *str = "CUDA_ERROR_NO_BINARY_FOR_GPU";
      break;
    case 210 :
      *str = "CUDA_ERROR_ALREADY_ACQUIRED";
      break;
    case 211 :
      *str = "CUDA_ERROR_NOT_MAPPED";
      break;
    case 212 :
      *str = "CUDA_ERROR_NOT_MAPPED_AS_ARRAY";
      break;
    case 213 :
      *str = "CUDA_ERROR_NOT_MAPPED_AS_POINTER";
      break;
    case 214 :
      *str = "CUDA_ERROR_ECC_UNCORRECTABLE";
      break;
    case 215 :
      *str = "CUDA_ERROR_UNSUPPORTED_LIMIT";
      break;
    case 216 :
      *str = "CUDA_ERROR_CONTEXT_ALREADY_IN_USE";
      break;
    case 300 :
      *str = "CUDA_ERROR_INVALID_SOURCE";
      break;
    case 301 :
      *str = "CUDA_ERROR_FILE_NOT_FOUND";
      break;
    case 302 :
      *str = "CUDA_ERROR_SHARED_OBJECT_SYMBOL_NOT_FOUND";
      break;
    case 303 :
      *str = "CUDA_ERROR_SHARED_OBJECT_INIT_FAILED";
      break;
    case 304 :
      *str = "CUDA_ERROR_OPERATING_SYSTEM";
      break;
    case 400 :
      *str = "CUDA_ERROR_INVALID_HANDLE";
      break;
    case 500 :
      *str = "CUDA_ERROR_NOT_FOUND";
      break;
    case 600 :
      *str = "CUDA_ERROR_NOT_READY";
      break;
    case 700 :
      *str = "CUDA_ERROR_LAUNCH_FAILED";
      break;
    case 701 :
      *str = "CUDA_ERROR_LAUNCH_OUT_OF_RESOURCES";
      break;
    case 702 :
      *str = "CUDA_ERROR_LAUNCH_TIMEOUT";
      break;
    case 703 :
      *str = "CUDA_ERROR_LAUNCH_INCOMPATIBLE_TEXTURING";
      break;
    case 704 :
      *str = "CUDA_ERROR_PEER_ACCESS_ALREADY_ENABLED";
      break;
    case 705 :
      *str = "CUDA_ERROR_PEER_ACCESS_NOT_ENABLED";
      break;
    case 708 :
      *str = "CUDA_ERROR_PRIMARY_CONTEXT_ACTIVE";
      break;
    case 709 :
      *str = "CUDA_ERROR_CONTEXT_IS_DESTROYED";
      break;
    case 710 :
      *str = "CUDA_ERROR_ASSERT";
      break;
    case 999 :
      *str = "CUDA_ERROR_UNKNOWN";
      break;
    default :
      *str = "Not defined CUDA error in this program";
      break;
    }
}
