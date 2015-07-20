/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
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

#include <string>
#include <vector>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "dpm/ImageObjects.h"
#include <runtime_manager/ConfigCarDpm.h>

#include <dpm.hpp>

#define XSTR(x) #x
#define STR(x) XSTR(x)

double config_overlap = 0.4;
double config_threshold = -0.5;
int config_lambda = 10;
int config_num_cells = 8;
int config_num_bins = 9;

int num_threads;
std::vector<std::string> model_files;
ros::Publisher car_pixel_publisher;

static void image_raw_cb(const sensor_msgs::Image& image)
{
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	cv::Mat mat = cv_image->image;
	std::vector<DPMObject> cars = dpm_detect_objects(mat,
													model_files,
													config_overlap,
													num_threads,
													config_threshold,
													config_lambda,
													config_num_cells,
													config_num_bins
													);
	size_t car_num = cars.size();

	std::vector<int> corner_point_array;
	std::vector<int> car_type_array;
	std::vector<float> car_score_array;

	for (int i = 0; i < static_cast<int>(car_num); ++i) {
		car_type_array.push_back(cars[i].class_id);

		corner_point_array.push_back(cars[i].rect.x);
		corner_point_array.push_back(cars[i].rect.y);
		corner_point_array.push_back(cars[i].rect.width);
		corner_point_array.push_back(cars[i].rect.height);

		car_score_array.push_back(cars[i].score);
	}

	dpm::ImageObjects message;
	message.car_num = car_num;
	message.corner_point = corner_point_array;
	message.car_type = car_type_array;
	message.score = car_score_array;

	message.header = image.header;
	message.header.stamp = image.header.stamp;

	car_pixel_publisher.publish(message);
}

static void set_default_parameters(const ros::NodeHandle& n)
{
	if (n.hasParam("/car_detector/threads")){
		int val;
		n.getParam("/car_detector/threads", val);
		num_threads = val;
	} else {
		num_threads = 8;
	}
}

void car_config_cb(const runtime_manager::ConfigCarDpm::ConstPtr& param)
{
	config_threshold = param->score_threshold;
	config_overlap   = param->group_threshold;
	config_lambda    = param->Lambda;
	config_num_cells = param->num_cells;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "car_dpm");

	ros::NodeHandle n;
	set_default_parameters(n);

	ros::Subscriber sub = n.subscribe("/image_raw", 1, image_raw_cb);
	car_pixel_publisher = n.advertise<dpm::ImageObjects>("car_pixel_xy", 1);

	std::string model_file(STR(MODEL_DIR) "car_2008.xml");
	model_files.push_back(model_file);
	
	ros::Subscriber config_subscriber;
	config_subscriber = n.subscribe("/config/car_dpm", 1, car_config_cb);

	ros::spin();
	return 0;
}
