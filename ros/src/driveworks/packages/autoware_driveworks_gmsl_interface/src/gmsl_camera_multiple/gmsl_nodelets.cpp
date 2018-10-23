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

// ROS
#include <ros/ros.h>
#include <ros/time.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

// Nvidia GMSL camera
#include "SekonixGmslCamera.hpp"

// Default device options
std::vector<DriveWorks::option_t> options =
{
  // making pair camera config key:value
  std::make_pair("type_ab", "ar0231-rccb"),
  std::make_pair("type_cd", "ar0231-rccb"),
  std::make_pair("type_ef", "ar0231-rccb"),
  std::make_pair("selector_mask", "11110001"),
  std::make_pair("cross_csi_sync", "0"),
  std::make_pair("fifo_size", "3"),
  std::make_pair("slave", "0"),
};

// Device arguments
DriveWorks::DeviceArguments CameraArguments(options);

namespace gmsl_camera
{
  class CameraNodelet: public nodelet::Nodelet
	{
		public:
			CameraNodelet():running(false)
			{
			}

			~CameraNodelet()
			{
				// signal gmsl camera to stop before nodelet exit
				// note: we are not using a signal handler: just using destructor to stop camera(s)
				// on the programme termination
				if(camera && running)
				{
				  try
				  {
					  NODELET_INFO("shutting down camera thread");
					  running = false;
						camera->shutdown();
						delete camera;
						NODELET_INFO("camera stopped");
				  }
				  catch(std::runtime_error& e)
				  {
						NODELET_ERROR("%s", e.what());
					}

				}
			}

			void onInit()
			{
				// global and private node handler
				ros::NodeHandle node  = getNodeHandle();
				ros::NodeHandle pnode = getPrivateNodeHandle();
				// spawn device thread when create this instance
				camera = new DriveWorks::SekonixGmslCamera(node, pnode, CameraArguments);
				running = true;
			}

	  private:
	    DriveWorks::SekonixGmslCamera *camera;
	    volatile bool running;

	};

	//PLUGINLIB_DECLARE_CLASS(gmsl_camera, CameraNodelet, gmsl_camera::CameraNodelet, nodelet::Nodelet);
	PLUGINLIB_EXPORT_CLASS(gmsl_camera::CameraNodelet, nodelet::Nodelet);
};

