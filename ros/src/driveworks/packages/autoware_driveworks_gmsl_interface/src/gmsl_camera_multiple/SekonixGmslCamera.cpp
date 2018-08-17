/*
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

#include "SekonixGmslCamera.hpp"


namespace DrivesWork
{
	
/* Constructor 
*   @param argument - connected camera configuration
*   This will start a camera initialisation and setting up memory pools based on given argument   
*/
SekonixGmslCamera::SekonixGmslCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh, DeviceArguments CameraArguments)
: node(comm_nh), pnode(param_nh)
{
  // read ros param here for device configurations
  param_nh.param("image_width", pub_width, int(640));
  param_nh.param("image_height", pub_height, int(480));
  param_nh.param("image_rate", pub_rate, int(10));
  param_nh.param("image_buffer", pub_buffer, int(5));

  std::string type_ab_value = "";                                                                        
  std::string type_cd_value = "";                                                                        
  std::string type_ef_value = "";                                                                        
  std::string selector_mask_value = "";                                                                  
  std::string cross_csi_sync_value = "";                                                                 
  std::string fifo_size_value = "";                                                                      
  std::string slave_value = "";                                                                          
  // reading new configuration                                                                                                       
  param_nh.param<std::string>("type_ab", type_ab_value,"ar0231-rccb");                                   
  param_nh.param<std::string>("type_cd", type_cd_value,"ar0231-rccb");                                   
  param_nh.param<std::string>("type_ef", type_ef_value,"ar0231-rccb");                                   
  param_nh.param<std::string>("selector_mask", selector_mask_value,"0001");                              
  param_nh.param<std::string>("cross_csi_sync", cross_csi_sync_value,"0");                               
  param_nh.param<std::string>("fifo_size", fifo_size_value,"3");                                         
  param_nh.param<std::string>("slave", slave_value,"0");


  // setting new configurations
  CameraArguments.set("type_ab", type_ab_value);
  CameraArguments.set("type_cd", type_cd_value);
  CameraArguments.set("type_ef", type_ef_value);
  CameraArguments.set("selector_mask", selector_mask_value);
  CameraArguments.set("cross_csi_sync", cross_csi_sync_value);
  CameraArguments.set("fifo_size", fifo_size_value);
  CameraArguments.set("slave", slave_value); 
 	
  // create gmsl camera instance with the arguments 
  gmsl_cam = new DrivesWorkApi(CameraArguments);
  // start camera frame grabber threads
  this->startup();
}

/* 
 * Destructor 
 */
SekonixGmslCamera::~SekonixGmslCamera()
{
  if(gmsl_cam) delete gmsl_cam;
}

/*
 * Start the polling threads to grab an image from the camera and publish it 
 */	
void SekonixGmslCamera::startup()
{
  // After gmsl cameras init - start image publishing thread(s)
  while(!(gmsl_cam->isCamReady()))
  {
		sleep(1);
	}
	// Ready
	numPort = gmsl_cam->getNumPort();
	for (uint32_t port = 0; port < numPort; ++port) 
  {
		camThreads.push_back(std::thread(&SekonixGmslCamera::feedImages, this, port));    
		camThreadsActive.push_back(true);
  }
  // TODO::Check if the feed image is ready to run by acquire the first capture before joining
  for (uint32_t port = 0; port < numPort; ++port) 
  {
    // Continue the thread without waiting to finish
		camThreads.at(port).detach();    
  }
	
	std::cout << "Start camera frame grabber.." << std::endl;
}

/*
 * Stop the polling threads to grab an image from the camera and publish it
 * Send a request to cleanup the camera connections all at once
 */	

void SekonixGmslCamera::shutdown()
{
	// Stop reading cameras and publishing images from the loop (feedImages)
	Shutdown = true;
	
	// Wait here until all the camera threads- feedImage are all exit 
	// so we clean up image pools and sdk all at once
	while(true)
	{
		// Get all active state
		int result = 0;
		for(std::size_t i=0;i<camThreadsActive.size();++i)
		{
			// sum to '0' if all threads stop polling
			result += (int)camThreadsActive[i];
		}
		// Check if all thread inactive
		if(result != 0) 
		{
			// continue waiting
			continue;
		}
		else
		{
			// break the loop to the next clean up
			break;
		}
		sleep(1);
	}
	
	// Clean up camera frames & sdk all at once
	gmsl_cam->stopCameras();
}


/*
 * A polling thread to grab and publish ROS image message via OpenCV
 */
void SekonixGmslCamera::feedImages(uint32_t port)
{
  // Init multiple cv cameras connection and topic name
  // Easy readable number of camera per port
  std::vector<uint32_t> numSiblings = gmsl_cam->getCameraPort();
  // ROS Opencv image vector
  std::vector<std::unique_ptr<OpenCVConnector>> cv_connectors;
  for (uint32_t cameraIdx = 0; cameraIdx < numSiblings[port]; cameraIdx++)
  {
		// Topic mapping e.g. gmsl_image_raw_<nvidia cam port A=0, B=1, C=2>_<sibling id 0,1,2,3> : gmsl_image_raw_0_0 
		const std::string topic = std::string("gmsl_image_raw_") + std::to_string(port) + std::string("_") + std::to_string(cameraIdx); 
		std::unique_ptr<OpenCVConnector> cvPtr(new OpenCVConnector(topic, pub_buffer));
		cv_connectors.push_back(std::move(cvPtr));
	}
	
  // Set publishing rate
  ros::Rate loop_rate(pub_rate);	
	// Read image from nvidia thread
	while(!Shutdown && ros::ok())
	{
		// Data buffer
	  std::vector<unsigned char*> images_raw;
	  uint32_t width;
	  uint32_t height;
	  
		// Reading data to image_raw from a particular csi camera port
		gmsl_cam->grabImages(port,images_raw,width,height);
		
		// Convert to ROS images using cv_connection + publishing images
		// There might be >1 camera per port
		int buffer_size = images_raw.size();
		// Publish images
		for(int buffer_idx=0; buffer_idx<buffer_size;buffer_idx++)
		{
			cv_connectors[buffer_idx]->WriteToOpenCV(images_raw[buffer_idx], width, height, pub_width, pub_height);
      // image publishing rate * Note: this is not really a camera publishing rate
      loop_rate.sleep();
		}
	}
   
  // Set this thread inactive flag
  camThreadsActive[port] = false;
}


}/* DrivesWork ns */
