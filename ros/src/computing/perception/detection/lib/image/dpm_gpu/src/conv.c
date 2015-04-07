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

#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>   /* to use "bool" in C-language.(bool is used in for_use_GPU.h) */
#include"for_use_GPU.h"



char *conv(unsigned int res)
{
    static char return_str[256];
    
    switch(res){
    case 0:
        sprintf(return_str, "CUDA_SUCCESS");
        break;
    case 1:
        sprintf(return_str, "CUDA_ERROR_INVALID_VALUE");
        break;
    case 2:
        sprintf(return_str, "CUDA_ERROR_OUT_OF_MEMORY");
        break;
    case 3:
        sprintf(return_str, "CUDA_ERROR_NOT_INITIALIZED");
        break;
    case 4:
        sprintf(return_str, "CUDA_ERROR_DEINITIALIZED");
        break;
    case 5:
        sprintf(return_str, "CUDA_ERROR_PROFILER_DESABLED");
        break;
    case 6:
        sprintf(return_str, "CUDA_ERROR_PROFILER_NOT_INITIALIZED");
        break;
    case 7:
        sprintf(return_str, "CUDA_ERROR_PROFILER_ALREADY_STARTED");
        break;
    case 8:
        sprintf(return_str, "CUDA_ERROR_PROFILER_ALREADY_STOPPED");
        break;
    case 100:
        sprintf(return_str, "CUDA_ERROR_NO_DEVICE");
        break;
    case 101:
        sprintf(return_str, "CUDA_ERROR_INVALID_DEVICE");
        break;
    case 200:
        sprintf(return_str, "CUDA_ERROR_INVALID_IMAGE");
        break;
    case 201:
        sprintf(return_str, "CUDA_ERROR_INVALID_CONTEXT");
        break;
    case 202:
        sprintf(return_str, "CUDA_ERROR_CONTEXT_ALREADY_CURRENT");
        break;
    case 205:
        sprintf(return_str, "CUDA_ERROR_MAP_FAILD");
        break;
    case 206:
        sprintf(return_str, "CUDA_ERROR_UNMAP_FAILED");
        break;
    case 207:
        sprintf(return_str, "CUDA_ERROR_ARRAY_IS_MAPPED");
        break;
    case 208:
        sprintf(return_str, "CUDA_ERROR_ALREADY_MAPPED");
        break;
    case 209:
        sprintf(return_str, "CUDA_ERROR_NO_BINARY_FOR_GPU");
        break;
    case 210:
        sprintf(return_str, "CUDA_ERROR_ALREADY_ACQUIRED");
        break;
    case 211:
        sprintf(return_str, "CUDA_ERROR_NOT_MAPPED");
        break;
    case 212:
        sprintf(return_str, "CUDA_ERROR_NOT_MAPPED_AS_ARRAY");
        break;
    case 213:
        sprintf(return_str, "CUDA_ERROR_NOT_MAPPED_AS_POINTER");
        break;
    case 214:
        sprintf(return_str, "CUDA_ERROR_ECC_UNCORRECTABLE");
        break;
    case 215:
        sprintf(return_str, "CUDA_ERROR_UNSUPPORTED_LIMIT");
        break;
    case 216:
        sprintf(return_str, "CUDA_ERROR_CONTEXT_ALREADY_IN_USE");
        break;
    case 300:
        sprintf(return_str, "CUDA_ERROR_INVALID_SOURCE");
        break;
    case 301:
        sprintf(return_str, "CUDA_ERROR_FILE_NOT_FOUND");
        break;
    case 302:
        sprintf(return_str, "CUDA_ERROR_SHARED_OBJECT_SYMBOL_NOT_FOUND");
        break;
    case 303:
        sprintf(return_str, "CUDA_ERROR_SHARED_OBJECT_INIT_FAILED");
        break;
    case 304:
        sprintf(return_str, "CUDA_ERROR_OPERATING_SYSTEM");
        break;
    case 400:
        sprintf(return_str, "CUDA_ERROR_INVALID_HANDLE");
        break;
    case 500:
        sprintf(return_str, "CUDA_ERROR_NOT_FOUND");
        break;
    case 600:
        sprintf(return_str, "CUDA_ERROR_NOT_READY");
        break;
    case 700:
        sprintf(return_str, "CUDA_ERROR_LAUNCH_FAILED");
        break;
    case 701:
        sprintf(return_str, "CUDA_ERROR_LAUNCH_OUT_OF_RESOURCES");
        break;
    case 702:
        sprintf(return_str, "CUDA_ERROR_LAUNCH_TIMEOUT");
        break;
    case 703:
        sprintf(return_str, "CUDA_ERROR_LAUNCH_INCOMPATIBLE_TEXTURING");
        break;
    case 704:
        sprintf(return_str, "CUDA_ERROR_PEER_ACCESS_ALREADY_ENABLED");
        break;
    case 705:
        sprintf(return_str, "CUDA_ERROR_PEER_ACCESS_NOT_ENABLED");
        break;
    case 708:
        sprintf(return_str, "CUDA_ERROR_PRIMARY_CONTEXT_ACTIVE");
        break;
    case 709:
        sprintf(return_str, "CUDA_ERROR_CONTEXT_IS_DESTROYED");
        break;
    case 999:
        sprintf(return_str, "CUDA_ERROR_UNKNOWN");
        break;

    default:
        sprintf(return_str, "not defined as value of CUresult");
        break;
    }
    return return_str;

}/* conv() */ 
