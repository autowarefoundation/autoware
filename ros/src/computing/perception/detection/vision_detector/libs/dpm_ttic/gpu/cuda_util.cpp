/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

const char *cuda_response_to_string(unsigned int res)
{
	switch(res){
	case 0:
		return "CUDA_SUCCESS";
	case 1:
		return "CUDA_ERROR_INVALID_VALUE";
	case 2:
		return "CUDA_ERROR_OUT_OF_MEMORY";
	case 3:
		return "CUDA_ERROR_NOT_INITIALIZED";
	case 4:
		return "CUDA_ERROR_DEINITIALIZED";
	case 5:
		return "CUDA_ERROR_PROFILER_DESABLED";
	case 6:
		return "CUDA_ERROR_PROFILER_NOT_INITIALIZED";
	case 7:
		return "CUDA_ERROR_PROFILER_ALREADY_STARTED";
	case 8:
		return "CUDA_ERROR_PROFILER_ALREADY_STOPPED";
	case 100:
		return "CUDA_ERROR_NO_DEVICE";
	case 101:
		return "CUDA_ERROR_INVALID_DEVICE";
	case 200:
		return "CUDA_ERROR_INVALID_IMAGE";
	case 201:
		return "CUDA_ERROR_INVALID_CONTEXT";
	case 202:
		return "CUDA_ERROR_CONTEXT_ALREADY_CURRENT";
	case 205:
		return "CUDA_ERROR_MAP_FAILD";
	case 206:
		return "CUDA_ERROR_UNMAP_FAILED";
	case 207:
		return "CUDA_ERROR_ARRAY_IS_MAPPED";
	case 208:
		return "CUDA_ERROR_ALREADY_MAPPED";
	case 209:
		return "CUDA_ERROR_NO_BINARY_FOR_GPU";
	case 210:
		return "CUDA_ERROR_ALREADY_ACQUIRED";
	case 211:
		return "CUDA_ERROR_NOT_MAPPED";
	case 212:
		return "CUDA_ERROR_NOT_MAPPED_AS_ARRAY";
	case 213:
		return "CUDA_ERROR_NOT_MAPPED_AS_POINTER";
	case 214:
		return "CUDA_ERROR_ECC_UNCORRECTABLE";
	case 215:
		return "CUDA_ERROR_UNSUPPORTED_LIMIT";
	case 216:
		return "CUDA_ERROR_CONTEXT_ALREADY_IN_USE";
	case 300:
		return "CUDA_ERROR_INVALID_SOURCE";
	case 301:
		return "CUDA_ERROR_FILE_NOT_FOUND";
	case 302:
		return "CUDA_ERROR_SHARED_OBJECT_SYMBOL_NOT_FOUND";
	case 303:
		return "CUDA_ERROR_SHARED_OBJECT_INIT_FAILED";
	case 304:
		return "CUDA_ERROR_OPERATING_SYSTEM";
	case 400:
		return "CUDA_ERROR_INVALID_HANDLE";
	case 500:
		return "CUDA_ERROR_NOT_FOUND";
	case 600:
		return "CUDA_ERROR_NOT_READY";
	case 700:
		return "CUDA_ERROR_LAUNCH_FAILED";
	case 701:
		return "CUDA_ERROR_LAUNCH_OUT_OF_RESOURCES";
	case 702:
		return "CUDA_ERROR_LAUNCH_TIMEOUT";
	case 703:
		return "CUDA_ERROR_LAUNCH_INCOMPATIBLE_TEXTURING";
	case 704:
		return "CUDA_ERROR_PEER_ACCESS_ALREADY_ENABLED";
	case 705:
		return "CUDA_ERROR_PEER_ACCESS_NOT_ENABLED";
	case 708:
		return "CUDA_ERROR_PRIMARY_CONTEXT_ACTIVE";
	case 709:
		return "CUDA_ERROR_CONTEXT_IS_DESTROYED";
	case 999:
		return "CUDA_ERROR_UNKNOWN";
	default:
		return "not defined as value of CUresult";
	}
}
