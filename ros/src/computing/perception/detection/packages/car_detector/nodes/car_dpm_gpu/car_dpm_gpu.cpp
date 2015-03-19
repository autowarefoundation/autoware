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
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "dpm/ImageObjects.h"

#include <runtime_manager/ConfigCarDpm.h>
#include <dpm_gpu.hpp>

#define XSTR(x) #x
#define STR(x) XSTR(x)

static double config_threshold;
static double config_overlap;
static int config_lambda;
static int config_num_cells;

static ros::Publisher car_pixel_publisher;

static void image_raw_cb(const sensor_msgs::Image& image_source)
{
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
	IplImage img = cv_image->image;
	IplImage *img_ptr = &img;

	DPMGPUResult result = dpm_gpu_detect_objects(img_ptr,
						     config_threshold,
						     config_overlap,
						     config_lambda,
						     config_num_cells);

	dpm::ImageObjects message;
	message.header = image_source.header;
	message.car_num = result.num;
	message.corner_point = result.corner_points;
	message.car_type = result.type;
	message.header.stamp = image_source.header.stamp;

	car_pixel_publisher.publish(message);
}

static void car_config_cb(const runtime_manager::ConfigCarDpm::ConstPtr& param)
{
	config_threshold = param->score_threshold;
	config_overlap   = param->group_threshold;
	config_lambda    = param->Lambda;
	config_num_cells = param->num_cells;
}

static std::string get_cubin_path(const ros::NodeHandle& n, const char *default_path)
{
	std::string cubin();
	std::string path;
	if (n.hasParam("/car_detector/cubin")) {
		n.getParam("/car_detector/cubin", path);
	} else {
		path = std::string(default_path);
	}

	return path;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "car_dpm_gpu");

	ros::NodeHandle n;

	std::string cubin = get_cubin_path(n, STR(DEFAULT_CUBIN));
	dpm_gpu_init_cuda(cubin);

	std::string com_csv(STR(MODEL_DIR) "car_comp.csv");
	std::string root_csv(STR(MODEL_DIR) "car_root.csv");
	std::string part_csv(STR(MODEL_DIR) "car_part.csv");
	dpm_gpu_load_models(com_csv, root_csv, part_csv);

	ros::Subscriber sub = n.subscribe("/image_raw", 1, image_raw_cb);
	car_pixel_publisher = n.advertise<dpm::ImageObjects>("car_pixel_xy", 1);

	ros::Subscriber config_subscriber;
	config_subscriber = n.subscribe("/config/car_dpm", 1, car_config_cb);

	ros::spin();

	dpm_gpu_cleanup_cuda();
	return 0;
}
