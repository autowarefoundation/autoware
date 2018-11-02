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
<<<<<<< HEAD
 * 
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
=======
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
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
<<<<<<< HEAD
  Author: Punnu Phairatt 
=======
  Author: Punnu Phairatt
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
  Initial Date: 10/05/18
*/


// ROS
#include <ros/ros.h>
// Nvidia GMSL camera
#include "SekonixGmslCamera.hpp"

// Clock
typedef std::chrono::high_resolution_clock Clock;

// Running state
<<<<<<< HEAD
static bool volatile running = true; 
=======
static bool volatile running = true;
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
// Signal handler to safely exit camera session
void signalHandler(int sig)
{
  (void)sig;
  running = false;
  ros::shutdown();
}

// Default device configuration (see DeviceArguments.hpp)
std::vector<DriveWorks::option_t> options =
{
  // making pair camera config key:value
  std::make_pair("type_ab", "ar0231-rccb"),
  std::make_pair("type_cd", "ar0231-rccb"),
  std::make_pair("type_ef", "ar0231-rccb"),
  std::make_pair("selector_mask", "11111111"),
  std::make_pair("cross_csi_sync", "0"),
  std::make_pair("fifo_size", "3"),
  std::make_pair("slave", "0"),
};


// -------------------------------------------------------------------------------
<<<<<<< HEAD
//     MAIN     
// -------------------------------------------------------------------------------
int main(int argc, const char **argv)
{
	
  // Create and init device arguments
  DriveWorks::DeviceArguments CameraArguments(options);   
 
  // Init ros node
  ros::init(argc, (char**)argv, "gmsl_cameras");
  ros::NodeHandle comm_nh;
  ros::NodeHandle param_nh("~");  
=======
//     MAIN
// -------------------------------------------------------------------------------
int main(int argc, const char **argv)
{

  // Create and init device arguments
  DriveWorks::DeviceArguments CameraArguments(options);

  // Init ros node
  ros::init(argc, (char**)argv, "gmsl_cameras");
  ros::NodeHandle comm_nh;
  ros::NodeHandle param_nh("~");
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
  ros::Rate loop_rate(10);
  // Detect exit signals
  signal(SIGHUP, signalHandler);  // controlling terminal closed, Ctrl-D
  signal(SIGINT, signalHandler);  // Ctrl-C
  signal(SIGQUIT, signalHandler); // Ctrl-\, clean quit with core dump
  signal(SIGABRT, signalHandler); // abort() called.
  signal(SIGTERM, signalHandler); // kill command
  signal(SIGSTOP, signalHandler); // kill command
<<<<<<< HEAD
  
  // Gmsl camera instance run
  DriveWorks::SekonixGmslCamera gmsl_multiple_cam(comm_nh, param_nh, CameraArguments);

  // other main thread: waiting for a signal to stop&shutdown 
=======

  // Gmsl camera instance run
  DriveWorks::SekonixGmslCamera gmsl_multiple_cam(comm_nh, param_nh, CameraArguments);

  // other main thread: waiting for a signal to stop&shutdown
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
  while(running & ros::ok())
  {
	 ros::spinOnce();
	 loop_rate.sleep();
  }
<<<<<<< HEAD
  
  // Gmsl camera signal termination- call camera cleanup 
=======

  // Gmsl camera signal termination- call camera cleanup
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
  std::cout << "Shutting down cameras .." << std::endl;
  gmsl_multiple_cam.shutdown();

  return 0;
}

