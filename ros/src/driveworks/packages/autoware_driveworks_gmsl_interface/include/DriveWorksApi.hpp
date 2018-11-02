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
/*
  This program requires Nvidia SDK installed
  Modified from Nvidia SDK - Camera gmsl and others (see Readme)
  Author: Punnu Phairatt
  Initial Date: 10/05/18
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
#include "nvmedia_image.h"
#include "nvmedia_ijpe.h"

// OPENCV-ROS Bridge
#include "cv_connection.hpp"

#include "DeviceArguments.hpp"


namespace DriveWorks
{

// Combine by camera sensor
struct Camera {
  dwSensorHandle_t sensor;
  uint32_t numSiblings;
  uint32_t width;
  uint32_t height;
  dwImageStreamerHandle_t streamer;         // different streamers to support different resolutions
  dwImageFormatConverterHandle_t yuv2rgba;
  std::queue<dwImageNvMedia *> rgbaPool;
  std::vector<dwImageNvMedia> frameRGBA;
  std::queue<uint8_t*> jpegPool;
	std::vector<NvMediaIJPE *> jpegEncoders;
};


struct ImageConfig {
  uint32_t pub_width;                     //publish image width
  uint32_t pub_height;                    //publish image height
  uint32_t pub_buffer;                    //publish buffer
  bool pub_compressed;                    //publish raw or compressed image
  uint32_t pub_compressed_quality;        //image compressed quality
  std::string pub_caminfo_folder;         //camera calibration folder
};


// Driveworks SDK interface for GMSL Camera
class DriveWorksApi
{
public:
  DriveWorksApi(DeviceArguments arguments, ImageConfig g_imageConfig);
  ~DriveWorksApi();

  void stopCameras();
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
  void initFrameImage(Camera* cameraSensor);
  void startCameraPipline();
  void threadCameraPipeline(Camera* cameraSensor, uint32_t port, dwContextHandle_t sdk);

  dwStatus captureCamera(dwImageNvMedia *frameNVMrgba,
                         dwSensorHandle_t cameraSensor, uint32_t port,
                         uint32_t sibling, dwImageFormatConverterHandle_t yuv2rgba,
                         uint8_t* jpeg_image, NvMediaIJPE *jpegEncoder);

  void releaseCameras(Camera* cameraSensor);
  void releaseSDK();


private:
  //Variables
  bool gTakeScreenshot = true;
  bool gImageCompressed = true;
  int gScreenshotCount = 0;
  bool g_run = false;
  bool g_exitCompleted = false;
  bool g_initState = false;
  uint32_t g_imageWidth;
  uint32_t g_imageHeight;
  uint32_t g_numCameras;
  uint32_t g_numPort;
  std::vector<uint32_t> g_numCameraPort;
  const uint32_t max_jpeg_bytes = 3 * 1290 * 1208;
	uint32_t JPEG_quality = 70;
  std::string g_calibFolder = "";


  //DriveWorks sdk
  DeviceArguments g_arguments;
  ImageConfig g_imageConfig;
  std::vector<Camera> cameras;
  bool eof;
  dwContextHandle_t sdk = DW_NULL_HANDLE;
  dwSALHandle_t sal     = DW_NULL_HANDLE;
  std::vector<std::vector<dwImageNvMedia*>> g_frameRGBAPtr;
  std::vector<std::vector<uint8_t*>>	g_frameJPGPtr;
	std::vector<std::vector<uint32_t>>  g_compressedSize;


};

};




#endif
