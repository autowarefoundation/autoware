/*
 * This code has been modified from
 * 1. https://github.com/vehicularkech/gmsl-camera-ros-driver
 * 2. https://github.com/cshort101/gmsl_driver
 * 3. https://github.com/DavidTorresOcana/ros_gmsl_driver
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
  This program requires ROS
  Author: Punnu Phairatt
  Initial Date: 10/05/18
*/



#ifndef _OPEN_CV_CONNECTOR_
#define _OPEN_CV_CONNECTOR_

#include <string>
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>



class OpenCVConnector {

public:
	/* Constructor */
  OpenCVConnector(std::string topic_name, std::string camera_frame_id, std::string cam_info_file, int buffer);
  /* Destructor */
  ~OpenCVConnector();
  /* Copy data to Opencv format and Publish ROS image message type */
  void WriteToOpenCV(unsigned char* data, int width_in, int height_in, int width_pub, int height_pub);
  void WriteToJpeg(uint8_t* data, uint32_t compressed_size);

  /* Ros node and image transport variables */
	ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Publisher pub;
  ros::Publisher pub_jpg;
  std::string topic_name;
  std::string camera_id;
  unsigned int counter;
  /* Ros camera info manager */
  sensor_msgs::CameraInfo camera_info;
  camera_info_manager::CameraInfoManager camera_info_manager;
	ros::Publisher pub_caminfo;
	std::string calib_folder;

};



#endif

