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


#include "DriveWorksApi.hpp"

namespace DriveWorks
{
	
/*
 * Constructor
 *  This will initialise csi camera configuration and setting up sdk and sensors
 */	
DriveWorksApi::DriveWorksApi(DeviceArguments arguments):g_arguments(arguments)
{
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
  const int totalCamera = px_numPort * px_numCameraPort;
  for (size_t i = 0; i < selector.length() && i < totalCamera; i++, idx++) 
  {
	  const char s = selector[i];
	  if (s == '1') 
	  {
	    cnt[idx / px_numCameraPort]++;
    }
  }

  // parsing arguments
  (*numCameras) = 0;
  for (size_t p = 0; p < px_numCameraPort-1; p++) 
  {
    if (cnt[p] > 0) 
    {
      std::string params;
      params += std::string("csi-port=") + port[p];
      params += ",camera-type=" + arguments.get((std::string("type_") + port[p]).c_str());
      params += ",camera-count=4"; // when using the mask, just ask for all cameras, mask will select properly

      if (selector.size() >= p*px_numCameraPort) 
      {
        params += ",camera-mask="+ selector.substr(p*px_numCameraPort, std::min(selector.size() - p*px_numCameraPort, size_t{px_numCameraPort}));
      }

      params += ",slave="  + arguments.get("slave");
      params += ",cross-csi-sync="  + arguments.get("cross_csi_sync");
      params += ",fifo-size="  + arguments.get("fifo_size");

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
    for (size_t cameraIdx = 0; cameraIdx < cameras[csiPort].numSiblings; ++cameraIdx) 
    {
	  pool.push_back(nullptr);
    }
    g_frameRGBAPtr.push_back(pool);  
  }
  
  // init RGBA frames for each camera in the port(e.g. 4 cameras/port)
  for (size_t csiPort = 0; csiPort < cameras.size() && g_run; csiPort++) 
  {
    initFrameRGBA(&cameras[csiPort]);
	  // record a number of connected camera
	  g_numCameraPort.push_back(cameras[csiPort].numSiblings);  
  }
	
  std::cout << "Cameras start capturing .. " << std::endl;
}



/*
 * Init camera frames convertion to RGBA and image pools per port
 */
void DriveWorksApi::initFrameRGBA(Camera* camera)
{
  std::cout << "Init Camera Frame Pools .. " << std::endl;
  // RGBA image pool for conversion from YUV camera output
  // two RGBA frames per camera per sibling for a pool
  // since image streamer might hold up-to one frame when using egl stream
  dwStatus result;
  int32_t pool_size = px_imagePoolSize;
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
    for (uint32_t cameraIdx = 0; cameraIdx < camera->numSiblings; cameraIdx++) 
    {
	    for (int32_t k = 0; k < pool_size; k++) 
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
    // start camera capturing
    g_run = g_run && dwSensor_start(camera->sensor) == DW_SUCCESS;
    eof_capture = false;
  }
}


/*
 * Function to capture an image frame and convert to RGBA
 */
dwStatus DriveWorksApi::captureCamera(dwImageNvMedia *frameNVMrgba,
                       dwSensorHandle_t cameraSensor,
                       uint32_t sibling,
                       dwImageFormatConverterHandle_t yuv2rgba)
{
  dwCameraFrameHandle_t frameHandle;
  dwImageNvMedia *frameNVMyuv = nullptr;
  dwStatus result = DW_FAILURE;
  result = dwSensorCamera_readFrame(&frameHandle, sibling, 300000, cameraSensor);
  if (result != DW_SUCCESS) 
  {
    std::cout << "readFrameNvMedia: " << dwGetStatusName(result) << std::endl;
    return result;
  }

  result = dwSensorCamera_getImageNvMedia(&frameNVMyuv, DW_CAMERA_PROCESSED_IMAGE, frameHandle);
  if( result != DW_SUCCESS )
  {
    std::cout << "readFrameNvMedia: " << dwGetStatusName(result) << std::endl;
  }

  result = dwImageFormatConverter_copyConvertNvMedia(frameNVMrgba, frameNVMyuv, yuv2rgba);
  if( result != DW_SUCCESS )
  {
    std::cout << "copyConvertNvMedia: " << dwGetStatusName(result) << std::endl;
  }

  result = dwSensorCamera_returnFrame(&frameHandle);
  if( result != DW_SUCCESS )
  {
    std::cout << "copyConvertNvMedia: " << dwGetStatusName(result) << std::endl;
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
 * Function to read images from a pool of cameras connected to a port
 *   @param port - a given port that camera(s) connected to
 *   @param data_out - reference to the raw data RGBA data
 *   @param width  - raw image output width
 *   @param height - raw image output height  
 */
void DriveWorksApi::grabImages(int port, std::vector<unsigned char*>& data_out, uint32_t& width, uint32_t& height)
{
  bool eof = false;
  // grab images from one of the camera port
  Camera* cameraSensor = &cameras[port];
  {
    if (cameraSensor->rgbaPool.empty()) 
     {
       std::cerr << "Ran out of RGBA buffers" << std::endl;
       return;
     }

     // capture from all cameras within a csi port
     for (uint32_t cameraIdx = 0;cameraIdx < cameraSensor->numSiblings && !cameraSensor->rgbaPool.empty();cameraIdx++) 
     {
       // capture, convert to rgba and return it
       eof = captureCamera(cameraSensor->rgbaPool.front(),
                            cameraSensor->sensor, cameraIdx,
                            cameraSensor->yuv2rgba);
       g_frameRGBAPtr[port][cameraIdx] = cameraSensor->rgbaPool.front();
	     cameraSensor->rgbaPool.pop();
	     cameraSensor->rgbaPool.push(g_frameRGBAPtr[port][cameraIdx]); 
	     g_run |= eof;    
     }
  }    
        
  gTakeScreenshot = true;
  if (gTakeScreenshot) 
  {
	  for (uint32_t cameraIdx = 0; cameraIdx < cameraSensor->numSiblings && !cameraSensor->rgbaPool.empty();cameraIdx++) 
    {
	    //copy to memory
	    dwImageNvMedia *frameNVMrgba = g_frameRGBAPtr[port][cameraIdx];
	    NvMediaImageSurfaceMap surfaceMap;
      //std::cout << "port: " << port << " camid:" << cameraIdx << std::endl;
	    if (NvMediaImageLock(frameNVMrgba->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
	    {
	      // reference data from cameras[i] to output buffer
		    data_out.push_back((unsigned char*)surfaceMap.surface[0].mapping);
		    width  = frameNVMrgba->prop.width;
		    height = frameNVMrgba->prop.height;
		    // unlock nvmedia if lock  
		    NvMediaImageUnlock(frameNVMrgba->img);
						
	    }
	    else
	    {
	      std::cout << "CANNOT LOCK NVMEDIA IMAGE - NO SCREENSHOT\n";
	    }		 
                    
    }     
	  gScreenshotCount++;
    gTakeScreenshot = false;
  }
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
