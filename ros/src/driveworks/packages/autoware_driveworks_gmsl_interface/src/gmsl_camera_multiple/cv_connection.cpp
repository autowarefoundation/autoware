/*
 * This code has been modified from 
 * 1. https://github.com/vehicularkech/gmsl-camera-ros-driver
 * 2. https://github.com/cshort101/gmsl_driver
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


#include "cv_connection.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

OpenCVConnector::OpenCVConnector(std::string topic_name, int buffer) : it(nh), counter(0)	
{
   pub = it.advertise(topic_name, buffer);
}


OpenCVConnector::~OpenCVConnector()
{
}


void OpenCVConnector::WriteToOpenCV(unsigned char* buffer, int width_in, int height_in, int width_pub, int height_pub) 
{
  cv::Mat mat_img(cv::Size(width_in, height_in), CV_8UC4, buffer);		// create a cv::Mat from rgbaImage
  cv::Mat dst;
  cv::resize(mat_img, dst, cv::Size(width_pub, height_pub));          // resize to the publishing size
  
  cv::Mat converted;																						      // new cv::Mat();
  cv::cvtColor(dst,converted,cv::COLOR_RGBA2RGB);   				          // COLOR_BGRA2BGR
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg; 																	      // message to be sent

  std_msgs::Header header; 																			      // empty header
  header.seq = counter; 																				      // user defined counter
  header.stamp = ros::Time::now(); 															      // time
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, converted);
  img_bridge.toImageMsg(img_msg); 															      // from cv_bridge to sensor_msgs::Image
  pub.publish(img_msg); 																				      // pub image
}


