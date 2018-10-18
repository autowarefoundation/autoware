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

/*
  This program requires ROS and Nvidia SDK installed
  Author: Punnu Phairatt
  Initial Date: 10/05/18
*/


#include "SekonixGmslCamera.hpp"


namespace DriveWorks
{

/* Constructor
*   @param argument - connected camera configuration
*   This will start a camera initialisation and setting up memory pools based on given argument
*/
SekonixGmslCamera::SekonixGmslCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh, DeviceArguments CameraArguments)
: node(comm_nh), pnode(param_nh)
{
  // read ros param here for image publishing configurations
  param_nh.param("image_width", pub_width, int(640));
  param_nh.param("image_height", pub_height, int(480));
  param_nh.param("image_buffer", pub_buffer, int(5));
  param_nh.param("image_compressed", pub_compressed, true);
  param_nh.param("image_compressed_quality", pub_compressed_quality, int(70));

  // read ros param for camera info publish
  std::string calib_folder = "";
  param_nh.param<std::string>("calib_folder", calib_folder,"");

  // from DriveWorksApi.hpp
  ImageConfig imageConfig = {
    (uint32_t)pub_width,         				//publish image width
    (uint32_t)pub_height,        				//publish image height
    (uint32_t)pub_buffer,        				//publish buffer
              pub_compressed,        		//publish raw or compressed image
    (uint32_t)pub_compressed_quality,   //image compressed quality
              calib_folder,             //camera calibration folder
  };

  // read ros param here for camera configurations
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
  gmsl_cam = new DriveWorksApi(CameraArguments, imageConfig);
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
	std::cout << "Start camera threads .." << std::endl;
}

/*
 * Stop the polling threads to grab an image from the camera and publish it
 * Send a request to cleanup the camera connections all at once
 */

void SekonixGmslCamera::shutdown()
{
	// Clean up camera frames & sdk all at once
	gmsl_cam->stopCameras();
}

}/* DriveWorks ns */
