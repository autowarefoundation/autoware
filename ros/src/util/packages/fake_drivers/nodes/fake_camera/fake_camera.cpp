/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cstdlib>
#include <cstdint>
#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_camera");
	ros::NodeHandle n;

	if (argc < 2) {
		std::cerr << "Usage: fake_driver image_file" << std::endl;
		std::exit(1);
	}

	const char *image_file = argv[1];
	std::cerr << "Image='" << image_file << "'" << std::endl;

	IplImage* img = cvLoadImage(image_file, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
	if (img == nullptr) {
		std::cerr << "Can't load " << image_file << "'" << std::endl;
		std::exit(1);
	}

	ros::Publisher pub = n.advertise<sensor_msgs::Image>("image_raw", 1000);

	sensor_msgs::Image msg;
	msg.width = img->width;
	msg.height = img->height;
	msg.is_bigendian = 0;
	msg.step = img->widthStep;

	uint8_t *data_ptr = reinterpret_cast<uint8_t*>(img->imageData);
	std::vector<uint8_t> data(data_ptr, data_ptr + img->imageSize);
	msg.data = data;

	msg.encoding = (img->nChannels == 1) ? 
		sensor_msgs::image_encodings::MONO8 : 
		sensor_msgs::image_encodings::RGB8;

	int fps;
	n.param<int>("/fake_camera/fps", fps, 30);
	fprintf(stderr, "%d fps\n", fps);
	ros::Rate loop_rate(fps); // Hz

	uint32_t count = 0;
	while (ros::ok()) {
		msg.header.seq = count;
		msg.header.frame_id = count;
		msg.header.stamp.sec = ros::Time::now().toSec();
		msg.header.stamp.nsec = ros::Time::now().toNSec();
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}
	cvReleaseImage(&img);//Free allocated data
	return 0;
}
