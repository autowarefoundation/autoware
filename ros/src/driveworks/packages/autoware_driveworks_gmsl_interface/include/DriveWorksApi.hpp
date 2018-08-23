/*
 * This code has been modified from Nvidia SDK
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
 *  All rights reserved.
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

#ifndef _DRIVE_WORKS_API_H_
#define _DRIVE_WORKS_API_H_

#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <memory>

#include <signal.h>
#include <iostream>
#include <cstring>
#include <thread>
#include <queue>

#ifdef LINUX
#include <execinfo.h>
#include <unistd.h>
#endif

#include <functional>
#include <list>
#include <iomanip>

#include <chrono>
#include <mutex>
#include <condition_variable>

// CORE
#include <dw/core/Context.h>
#include <dw/core/Logger.h>

// HAL (need  nvmedia_image.h)
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>

// IMAGE
#include <dw/image/FormatConverter.h>
#include <dw/image/ImageStreamer.h>

#include "DeviceArguments.hpp"


namespace DriveWorks
{

// Combine by camera sensor
struct Camera {
  dwSensorHandle_t sensor;
  uint32_t numSiblings;
  uint32_t width;
  uint32_t height;
  dwImageStreamerHandle_t streamer; // different streamers to support different resolutions
  dwImageFormatConverterHandle_t yuv2rgba;
  std::queue<dwImageNvMedia *> rgbaPool;
  std::vector<dwImageNvMedia> frameRGBA;
};


// Driveworks SDK interface for GMSL Camera
class DriveWorksApi
{
public:
  DriveWorksApi(DeviceArguments arguments);
  ~DriveWorksApi(); 
 
  void stopCameras(); 
  void grabImages(int port,std::vector<unsigned char*>& data_out, uint32_t& width, uint32_t& height);
  bool isCamReady();
  bool isShutdownCompleted();
  uint32_t getNumPort();
  std::vector<uint32_t> getCameraPort();
 
private:
  void startCameras();
  //Driveworks sdk interface 
  void initSdk(dwContextHandle_t *context);
  void initSAL(dwSALHandle_t *sal, dwContextHandle_t context);
  void initSensors(std::vector<Camera> *cameras,
                   uint32_t *numCameras,
                   dwSALHandle_t sal,
                   DeviceArguments &arguments);
                   
  void initFramesStart();
  void initFrameRGBA(Camera* cameraSensor);

  
  dwStatus captureCamera(dwImageNvMedia *frameNVMrgba,
                         dwSensorHandle_t cameraSensor,
                         uint32_t sibling,
                         dwImageFormatConverterHandle_t yuv2rgba);
                         
  void releaseCameras(Camera* cameraSensor);
  void releaseSDK();

  
private:
  //Variables 
  bool gTakeScreenshot = false;
  int gScreenshotCount = 0;
  bool g_run = false;
  bool g_exitCompleted = false;
  bool g_initState = false;
  uint32_t g_imageWidth;
  uint32_t g_imageHeight;
  uint32_t g_numCameras;
  uint32_t g_numPort;
  std::vector<uint32_t> g_numCameraPort;
  const uint32_t px_numPort = 3;           // fixed for px2, don't change
  const uint32_t px_numCameraPort = 4;     // fixed for px2, don't change
  const int32_t  px_imagePoolSize = 2;     // fixed for px2, don't change
	
	
  //DriveWorks sdk 
  DeviceArguments g_arguments;
  std::vector<std::vector<dwImageNvMedia*>> g_frameRGBAPtr; 
  std::vector<Camera> cameras;
  bool eof_capture;
  dwContextHandle_t sdk = DW_NULL_HANDLE;
  dwSALHandle_t sal     = DW_NULL_HANDLE;
};
  	
};

	


#endif
