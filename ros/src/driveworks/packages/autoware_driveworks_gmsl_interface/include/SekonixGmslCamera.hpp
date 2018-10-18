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

#ifndef _SEKONIX_GMSL_CAMERA_H_
#define _SEKONIX_GMSL_CAMERA_H_

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

//#include <lodepng.h>

// ROS
#include <ros/ros.h>




// Camera driver api
#include "DriveWorksApi.hpp"



namespace DriveWorks
{

// This class is running multiple threads to acquire images from gmsl cameras connected on PX2
// Each acquired image is published under "gmsl_image_raw_[GROUP_ID]_[CAM_ID]"
class SekonixGmslCamera
{
public:
  /* Constructor
   * @param argument - connected camera configuration
   * This will start a camera initialisation and setting up memory pools based on given argument
   */
  SekonixGmslCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh, DeviceArguments CameraArguments);

  /* Destructor */
  ~SekonixGmslCamera();

  /*
   * Start the polling threads to grab an image from the camera and publish it
   */
	void startup();

	/*
   * Stop the polling threads to grab an image from the camera and publish it
   * Send a request to cleanup the camera connections all at once
   */
	void shutdown();

private:
  ros::NodeHandle node;                                           // Global ns
  ros::NodeHandle pnode;                                          // Private ns
  DriveWorksApi *gmsl_cam;                                        // GMSL camera instance
  int pub_width;                                                  // Image publishing width
  int pub_height;                                                 // Image publishing height
  int pub_buffer;                                                 // Image buffer for publishing
  uint32_t numPort;                                               // PX2 camera port
  bool pub_compressed;                                            // Publishing compressed images
  int pub_compressed_quality;                                     // jpeg quality

};//SekonixGmslCamera class

};//DriveWorks ns




#endif
