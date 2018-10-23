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


#include "DriveWorksApi.hpp"

namespace DriveWorks
{

/*
 * Constructor
 *  This will initialise csi camera configuration and setting up sdk and sensors
 */
DriveWorksApi::DriveWorksApi(DeviceArguments arguments, ImageConfig imageConfig):
g_arguments(arguments), g_imageConfig(imageConfig)
{
	// init image publishing configuration
	g_imageWidth = imageConfig.pub_width;
  g_imageHeight = imageConfig.pub_height;
  gImageCompressed = imageConfig.pub_compressed;
  JPEG_quality = imageConfig.pub_compressed_quality;
  g_calibFolder = imageConfig.pub_caminfo_folder;
  // init sdk, and start cameras
  this->startCameras();
}

/*
 * Destructor
 */
DriveWorksApi::~DriveWorksApi()
{
}

/*
 * Init SDK
 */
void DriveWorksApi::initSdk(dwContextHandle_t *context)
{
  std::cout << "Init SDK .. " << std::endl;
  // Instantiate Driveworks SDK context
  dwContextParameters sdkParams;
  memset(&sdkParams, 0, sizeof(dwContextParameters));
  dwInitialize(context, DW_VERSION, &sdkParams);
}

/*
 * Init SAL abstract layer
 */
void DriveWorksApi::initSAL(dwSALHandle_t *sal, dwContextHandle_t context)
{
  std::cout << "Init SAL .. " << std::endl;
  dwStatus result;
  result = dwSAL_initialize(sal, context);
  if (result != DW_SUCCESS)
  {
	  std::cerr << "Cannot initialize SAL: " << dwGetStatusName(result) << std::endl;
    exit(1);
  }
}


/*
 * Init sensors: csi ports,connected cameras, image width/height
 */
void DriveWorksApi::initSensors(std::vector<Camera> *cameras, uint32_t *numCameras,
                                dwSALHandle_t sal, DeviceArguments &arguments)
{
  std::cout << "Init Sensors .. " << std::endl;
  std::string selector = arguments.get("selector_mask");
  dwStatus result;
  // identify active ports
  int idx             = 0;
  int cnt[3]          = {0, 0, 0};
  std::string port[3] = {"ab", "cd", "ef"};
  for (size_t i = 0; i < selector.length() && i < 12; i++, idx++)
  {
		const char s = selector[i];
		if (s == '1')
		{
			cnt[idx / 4]++;
    }
  }

  // parsing arguments
  (*numCameras) = 0;
  for (size_t p = 0; p < 3; p++)
  {
    if (cnt[p] > 0)
    {
      std::string params;
      params += std::string("csi-port=") + port[p];
      params += ",camera-type=" + arguments.get((std::string("type_") + port[p]).c_str());
      params += ",camera-count=4"; // when using the mask, just ask for all cameras, mask will select properly

      if (selector.size() >= p*4)
      {
        params += ",camera-mask="+ selector.substr(p*4, std::min(selector.size() - p*4, size_t{4}));
      }

      params += ",slave="  + arguments.get("slave");
      params += ",cross-csi-sync="  + arguments.get("cross_csi_sync");
      params += ",fifo-size="  + arguments.get("fifo_size");


      //Debug arguments

      std::cout << "DEBUG ARGS PORT:  " << p << std::endl;
      std::cout << params.c_str() << std::endl;

      //End Debug

      dwSensorHandle_t salSensor = DW_NULL_HANDLE;
      dwSensorParams salParams;
      salParams.parameters = params.c_str();
      salParams.protocol = "camera.gmsl";
      result = dwSAL_createSensor(&salSensor, salParams, sal);
      if (result == DW_SUCCESS)
      {
        Camera cam;
        cam.sensor = salSensor;

        dwImageProperties cameraImageProperties;
        dwSensorCamera_getImageProperties(&cameraImageProperties,
                                          DW_CAMERA_PROCESSED_IMAGE,
                                          salSensor);

        dwCameraProperties cameraProperties;
        dwSensorCamera_getSensorProperties(&cameraProperties, salSensor);

        cam.width = cameraImageProperties.width;
        cam.height = cameraImageProperties.height;
        cam.numSiblings = cameraProperties.siblings;
		    cameras->push_back(cam);

        (*numCameras) += cam.numSiblings;
      }
      else
      {
        std::cerr << "Cannot create driver: " << salParams.protocol
                  << " with params: " << salParams.parameters << std::endl
                  << "Error: " << dwGetStatusName(result) << std::endl;

        if (result == DW_INVALID_ARGUMENT)
        {
          std::cerr << "It is possible the given camera is not supported. "
                    << "Please refer to the documentation for this sample."
                    << std::endl;
        }
      }
    }
  }
}

/*
 * Init camera frames
 */
void DriveWorksApi::initFramesStart()
{
  std::cout << "Init Camera Frames .. " << std::endl;
  // check cameras connected to csi ports
  if (cameras.size() == 0)
  {
    std::cerr << "Need to specify at least 1 at most 12 cameras to be used" << std::endl;
	  exit(-1);
  }

  // allocate frameRGBA pointer
  for (size_t csiPort = 0; csiPort < cameras.size(); csiPort++)
  {
    std::vector<dwImageNvMedia*> pool;
    std::vector<uint8_t*> pool_jpeg;
		std::vector< uint32_t> poolsize;
    for (size_t cameraIdx = 0; cameraIdx < cameras[csiPort].numSiblings; ++cameraIdx)
    {
			pool.push_back(nullptr);
			pool_jpeg.push_back(nullptr);
			poolsize.push_back(0);
    }
    // assign to class variables for later use
    g_frameRGBAPtr.push_back(pool);
    g_frameJPGPtr.push_back(pool_jpeg);
		g_compressedSize.push_back(poolsize);

  }

  // init RGBA frames for each camera in the port(e.g. 4 cameras/port)
  for (size_t csiPort = 0; csiPort < cameras.size() && g_run; csiPort++)
  {
    initFrameImage(&cameras[csiPort]);
	  // record a number of connected camera
	  g_numCameraPort.push_back(cameras[csiPort].numSiblings);
  }
}



/*
 * Init camera frames convertion to RGBA and image pools per port
 */
void DriveWorksApi::initFrameImage(Camera* camera)
{
  std::cout << "Init Camera Frame Pools .. " << std::endl;
  // RGBA image pool for conversion from YUV camera output
  // two RGBA frames per camera per sibling for a pool
  // since image streamer might hold up-to one frame when using egl stream
  dwStatus result;
  int32_t pool_size = 2;
  uint32_t numFramesRGBA = pool_size*camera->numSiblings;

  // temp variable for easy access and de-reference back to camera->frameRGBA in releasing nvidia image frame read
  std::vector<dwImageNvMedia>& g_frameRGBA = camera->frameRGBA;

  g_frameRGBA.reserve(numFramesRGBA);
  {
    dwImageProperties cameraImageProperties;
    dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE,camera->sensor);
    dwImageProperties displayImageProperties = cameraImageProperties;
    displayImageProperties.pxlFormat         = DW_IMAGE_RGBA;
    displayImageProperties.planeCount        = 1;

    // format converter
    result = dwImageFormatConverter_initialize(&camera->yuv2rgba, cameraImageProperties.type, sdk);
    if (result != DW_SUCCESS)
    {
	    std::cerr << "Cannot create pixel format converter : yuv->rgba" << dwGetStatusName(result) <<  std::endl;
      g_run = false;
    }

    // allocate image pool
    for(uint32_t cameraIdx = 0; cameraIdx < camera->numSiblings; cameraIdx++)
    {
	    for(int32_t k = 0; k < pool_size; k++)
	    {
		    dwImageNvMedia rgba{};
        result = dwImageNvMedia_create(&rgba, &displayImageProperties, sdk);
        if (result != DW_SUCCESS)
        {
		      std::cerr << "Cannot create nvmedia image for pool:" << dwGetStatusName(result) << std::endl;
          g_run = false;
          break;
        }
        // don't forget to delete dwImage via g_frameRGBA when exit
		    g_frameRGBA.push_back(rgba);
        camera->rgbaPool.push(&g_frameRGBA.back());
      }
    }


    // NVMedia image compression definition.
		for(uint32_t cameraIdx = 0; cameraIdx < camera->numSiblings; cameraIdx++)
		{
			NvMediaDevice *device;
			device = NvMediaDeviceCreate();
			if(!device)
			{
				std::cerr << "main: NvMediaDeviceCreate failed\n" <<  std::endl;
				g_run = false;
			}
			NvMediaIJPE* jpegEncoder = NULL;
			jpegEncoder = NvMediaIJPECreate(device, NvMediaSurfaceType_Image_YUV_420,(uint8_t) 1, max_jpeg_bytes );
			if(!jpegEncoder)
			{
				std::cerr << "main: NvMediaIJPECreate failed\n" <<  std::endl;
				g_run = false;
			}
			else
			{
				camera->jpegEncoders.push_back(jpegEncoder);
			}
		}
		// allocate compressed image pool
		for(uint32_t cameraIdx = 0; cameraIdx < camera->numSiblings; cameraIdx++)
		{
			for (int32_t k = 0; k < pool_size; k++)
			{
				uint8_t* jpeg_img = (uint8_t*) malloc( max_jpeg_bytes );
        camera->jpegPool.push(jpeg_img);
      }
    }
    // start camera capturing
    g_run = g_run && dwSensor_start(camera->sensor) == DW_SUCCESS;
    eof = false;
  }
}



void DriveWorksApi::startCameraPipline()
{
	std::cout << "Start camera pipline  " << std::endl;
	std::vector<std::thread> camThreads;
	for (uint32_t i = 0; i < cameras.size(); ++i)
  {
		camThreads.push_back(std::thread(&DriveWorksApi::threadCameraPipeline, this, &cameras[i], i, sdk));
  }

  // loop through all cameras check if they have provided the first frame
  /*
  for (size_t csiPort = 0; csiPort < cameras.size() && g_run; csiPort++)
  {
		for (uint32_t cameraIdx = 0;cameraIdx < cameras[csiPort].numSiblings && g_run;cameraIdx++)
		{
			while (!g_frameRGBAPtr[csiPort][cameraIdx] && g_run)
			{
				std::this_thread::yield();
      }
    }
  }*/

  // start camera threads and release
	for (uint32_t i = 0; i < cameras.size(); ++i)
	{
		camThreads.at(i).detach();
  }
}


void DriveWorksApi::threadCameraPipeline(Camera* cameraSensor, uint32_t port, dwContextHandle_t sdk)
{
	std::cout << "Start camera thread for port:  " << port << std::endl;
	// cv publishers
	std::vector<std::unique_ptr<OpenCVConnector>> cv_connectors;
	// init multiple cv cameras connection and topic name
  for (uint32_t cameraIdx = 0; cameraIdx < cameraSensor->numSiblings; cameraIdx++)
  {
	 // Topic mapping e.g. gmsl_image_raw_<nvidia cam port A=0, B=1, C=2>_<sibling id 0,1,2,3> : port_0/camera_1/(image_raw,image_raw/compressed)
		const std::string topic = std::string("port_") + std::to_string(port) + std::string("/camera_") + std::to_string(cameraIdx);
    const std::string camera_frame_id = std::string("gmsl_camera_") + std::to_string(port) + std::string("_") + std::to_string(cameraIdx);
    const std::string cam_info_file = std::string("file://") + std::string(g_calibFolder) + std::to_string(port) + std::to_string(cameraIdx) + std::string("_calibration.yml");
    std::unique_ptr<OpenCVConnector> cvPtr(new OpenCVConnector(topic, camera_frame_id, cam_info_file, 10));
		cv_connectors.push_back(std::move(cvPtr));
	}

	while (g_run && ros::ok())
	{
		bool eofAny = false;
		// capture from all csi-ports
    // NOTE if cross-csi-synch is active, all cameras will capture at the same time
    {
			if (eof)
			{
				eofAny = true;
        continue;
      }

      if (cameraSensor->rgbaPool.empty())
      {
				std::cerr << "Ran out of RGBA buffers, continuing" << std::endl;
				continue;
			}

      // capture from all cameras within a csi port
			for (uint32_t cameraIdx = 0;cameraIdx < cameraSensor->numSiblings && !cameraSensor->rgbaPool.empty();cameraIdx++)
			{
				// capture, convert to rgba and return it
        eof = captureCamera(cameraSensor->rgbaPool.front(),
                            cameraSensor->sensor,
                            port, cameraIdx,
                            cameraSensor->yuv2rgba,
                            cameraSensor->jpegPool.front(),
                            cameraSensor->jpegEncoders[cameraIdx]);
				g_frameRGBAPtr[port][cameraIdx] = cameraSensor->rgbaPool.front();
        cameraSensor->rgbaPool.pop();

        g_frameJPGPtr[port][cameraIdx] =  cameraSensor->jpegPool.front();
				cameraSensor->jpegPool.pop();

        if (!eof)
        {
					cameraSensor->rgbaPool.push(g_frameRGBAPtr[port][cameraIdx]);
					cameraSensor->jpegPool.push(g_frameJPGPtr[port][cameraIdx]);
				}

					eofAny |= eof;
			}
     }

     // stop to take screenshot (will cause a delay)
     if (gTakeScreenshot)
     {
			{

				for (uint32_t cameraIdx = 0; cameraIdx < cameraSensor->numSiblings && !cameraSensor->rgbaPool.empty(); cameraIdx++)
				{
					//copy to memory replacing by //takeScreenshot(g_frameRGBAPtr[port][cameraIdx], port, cameraIdx);
					dwImageNvMedia *frameNVMrgba = g_frameRGBAPtr[port][cameraIdx];
					NvMediaImageSurfaceMap surfaceMap;

					if (NvMediaImageLock(frameNVMrgba->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
					{
						// publish an image
						if(gImageCompressed)
						{
							// compressed
							cv_connectors[cameraIdx]->WriteToJpeg( g_frameJPGPtr[port][cameraIdx],  g_compressedSize[port][cameraIdx]);
						}
						else
						{
							//raw (resize if set)
							cv_connectors[cameraIdx]->WriteToOpenCV((unsigned char*)surfaceMap.surface[0].mapping,
																											frameNVMrgba->prop.width, frameNVMrgba->prop.height,
																											g_imageWidth,
																											g_imageHeight);
						}

						NvMediaImageUnlock(frameNVMrgba->img);
					}
					else
					{
						std::cout << "CANNOT LOCK NVMEDIA IMAGE - NO SCREENSHOT\n";
					}

				}
			}
      gScreenshotCount++;
     }
		 std::this_thread::sleep_for(std::chrono::milliseconds(5));
		 g_run = g_run && !eofAny;
    }//end while
}


/*
 * Function to capture an image frame and convert to RGBA
 */
dwStatus DriveWorksApi::captureCamera(dwImageNvMedia *frameNVMrgba,
                       dwSensorHandle_t cameraSensor, uint32_t port,
                       uint32_t sibling, dwImageFormatConverterHandle_t yuv2rgba,
                       uint8_t* jpeg_image, NvMediaIJPE *jpegEncoder)
{
  dwCameraFrameHandle_t frameHandle;
  dwImageNvMedia *frameNVMyuv = nullptr;
  dwStatus result = DW_FAILURE;
  result = dwSensorCamera_readFrame(&frameHandle, sibling, 300000, cameraSensor);
  if (result != DW_SUCCESS)
  {
    std::cerr << "readFrameNvMedia: " << dwGetStatusName(result) << std::endl;
    return result;
  }

  result = dwSensorCamera_getImageNvMedia(&frameNVMyuv, DW_CAMERA_PROCESSED_IMAGE, frameHandle);
  if( result != DW_SUCCESS )
  {
    std::cerr << "readImageNvMedia: " << dwGetStatusName(result) << std::endl;
  }

  result = dwImageFormatConverter_copyConvertNvMedia(frameNVMrgba, frameNVMyuv, yuv2rgba);
  if( result != DW_SUCCESS )
  {
    std::cerr << "copyConvertNvMedia: " << dwGetStatusName(result) << std::endl;
  }


  if(gImageCompressed)
  {
		NvMediaStatus nvStatus = NvMediaIJPEFeedFrame(jpegEncoder,frameNVMyuv->img,JPEG_quality);
		if(nvStatus != NVMEDIA_STATUS_OK)
		{
			std::cerr <<"NvMediaIJPEFeedFrameQuality failed: %x\n" << nvStatus <<  std::endl;
		}
		nvStatus = NvMediaIJPEBitsAvailable(jpegEncoder, &g_compressedSize[port][sibling],NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING , 10000);
		nvStatus = NvMediaIJPEGetBits(jpegEncoder, &g_compressedSize[port][sibling], jpeg_image, 0);
		if(nvStatus != NVMEDIA_STATUS_OK && nvStatus != NVMEDIA_STATUS_NONE_PENDING)
		{
			std::cerr <<"main: Error getting encoded bits\n"<<  std::endl;
		}
	}
  result = dwSensorCamera_returnFrame(&frameHandle);
  if( result != DW_SUCCESS )
  {
    std::cout << "returnFrameNvMedia: " << dwGetStatusName(result) << std::endl;
  }

  return DW_SUCCESS;
}


/*
 * Function to release camera session and frame handler
 * as well as preallocate image pools
 */
void DriveWorksApi::releaseCameras(Camera* cameraSensor)
{
  // release sensor
  std::cout << "Cleaning camera thread .. " << std::endl;
  {
    dwSensor_stop(cameraSensor->sensor);
    std::cout << "Cleaning camera thread .. dwSensor " << std::endl;
    dwSAL_releaseSensor(&cameraSensor->sensor);
    std::cout << "Cleaning camera thread .. dwSAL " << std::endl;
    dwImageFormatConverter_release(&cameraSensor->yuv2rgba);
    std::cout << "Cleaning camera thread .. dwConvert " << std::endl;
  }

  // cleanup nvmedia preallocate image frames
  for (dwImageNvMedia& frame : cameraSensor->frameRGBA)
  {
	  dwStatus result = dwImageNvMedia_destroy(&frame);
    if (result != DW_SUCCESS)
    {
	    std::cerr << "Cannot destroy nvmedia: " << dwGetStatusName(result) << std::endl;
      g_run = false;
      break;
    }
  }

  // cleanup jpeg compression
  for (auto jpegEncoder_ : cameraSensor->jpegEncoders)
  {
		NvMediaIJPEDestroy(jpegEncoder_);
  }
}


/*
 * Function to clean-up SDK/SAL/LOGGER
 */
void DriveWorksApi::releaseSDK()
{
  // release sdk and sal
  // release used objects in correct order
  std::cout << "Release SDK .." << std::endl;
  dwSAL_release(&sal);
  dwRelease(&sdk);
  dwLogger_release();
}


/*
 * Function to start all initialisation and states
 */
void DriveWorksApi::startCameras()
{
  std::cout << "Start camera... " << std::endl;
  // set run flag
  g_run = true;
  // Create GMSL Camera interface, based on the camera selector mask
  initSdk(&sdk);
  initSAL(&sal, sdk);
  // Init a number of cameras base on arguments
  initSensors(&cameras, &g_numCameras, sal, g_arguments);
  // Init image frames and start camera image acquisition
  initFramesStart();
  // Set values
  g_numPort = cameras.size();
  // Set done init state
  g_initState = true;
  // Start image publishing thread
  startCameraPipline();
}

/*
 * Function to clean up all camera connections and states
 */
void DriveWorksApi::stopCameras()
{
  std::cout << "Stop camera... " << std::endl;
  // loop through all camera ports to cleanup all connected cameras
  for (size_t csiPort = 0; csiPort < cameras.size(); csiPort++)
  {
    releaseCameras(&cameras[csiPort]);
  }
  // sdk instances release
  releaseSDK();
  // set init and run state
  g_exitCompleted = true;
  g_run = false;
  g_initState = false;
}


/*
 * Function to check if initialisation has been completed
 */
bool DriveWorksApi::isCamReady()
{
  return g_run && g_initState;
}

/*
 * Function to get a number of camera ports(groups) that has been assigned
 */
uint32_t DriveWorksApi::getNumPort()
{
  return g_numPort;
}

/*
 * Function to get a number of cameras connected to a port
 */
std::vector<uint32_t> DriveWorksApi::getCameraPort()
{
  return g_numCameraPort;
}

/*
 * Function to check if the shutdown process is completed
 */
bool DriveWorksApi::isShutdownCompleted()
{
  return g_exitCompleted && !g_run;
}

}//DriveWorks ns
