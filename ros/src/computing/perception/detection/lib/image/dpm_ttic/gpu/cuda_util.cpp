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
