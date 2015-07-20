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

#include <cv_tracker/image_obj.h>
#include <runtime_manager/ConfigCarDpm.h>
#include <runtime_manager/ConfigPedestrianDpm.h>

#include <dpm_ttic.hpp>

#define XSTR(x) #x
#define STR(x) XSTR(x)

static ros::Publisher car_image_obj_pub;
static ros::Publisher pedestrian_image_obj_pub;

static DPMGPUModel *car_gpu_model;
static DPMGPUModel *pedestrian_gpu_model;
static DPMTTICModel *car_ttic_model;
static DPMTTICModel *pedestrian_ttic_model;

static DPMTTICParam car_ttic_param;
static DPMTTICParam pedestrian_ttic_param;

 // XXX Should be configurable(rosparam etc)
static bool use_gpu = false;
static bool is_car_check = true;
static bool is_pedestrian_check = true;

static void set_default_param(DPMTTICParam& param)
{
	param.overlap = 0.4;
	param.threshold = -0.5;
	param.lambda = 10;
	param.num_cells = 8;
}

static void result_to_image_obj_message(cv_tracker::image_obj& msg, const DPMTTICResult result)
{
	for (int i = 0; i < result.num; ++i) {
		cv_tracker::image_rect rect;

		int base = i * 4;
		rect.x = result.corner_points[base];
		rect.y = result.corner_points[base+1];
		rect.width = result.corner_points[base+2];
		rect.height = result.corner_points[base+3];

		msg.obj.push_back(rect);
	}
}

static void image_raw_car_cb(const sensor_msgs::Image& image_source)
{
	if (!is_car_check)
		return;

	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
	IplImage img = cv_image->image;
	IplImage *img_ptr = &img;

	cv_tracker::image_obj msg;
	msg.header = image_source.header;
	msg.type = "car";

	if (use_gpu) {
		DPMTTICResult result = car_gpu_model->detect_objects(img_ptr, car_ttic_param);
		result_to_image_obj_message(msg, result);
	} else {
		DPMTTICResult result = car_ttic_model->detect_objects(img_ptr, car_ttic_param);
		result_to_image_obj_message(msg, result);
	}

	car_image_obj_pub.publish(msg);
}

static void image_raw_pedestrian_cb(const sensor_msgs::Image& image_source)
{
	if (!is_pedestrian_check)
		return;

	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
	IplImage img = cv_image->image;
	IplImage *img_ptr = &img;

	cv_tracker::image_obj msg;
	msg.header = image_source.header;
	msg.type = "pedestrian";

	if (use_gpu) {
		DPMTTICResult result = pedestrian_gpu_model->detect_objects(img_ptr, pedestrian_ttic_param);
		result_to_image_obj_message(msg, result);
	} else {
		DPMTTICResult result = pedestrian_ttic_model->detect_objects(img_ptr, pedestrian_ttic_param);
		result_to_image_obj_message(msg, result);
	}

	pedestrian_image_obj_pub.publish(msg);
}

static void car_config_cb(const runtime_manager::ConfigCarDpm::ConstPtr& param)
{
	car_ttic_param.threshold = param->score_threshold;
	car_ttic_param.overlap   = param->group_threshold;
	car_ttic_param.lambda    = param->Lambda;
	car_ttic_param.num_cells = param->num_cells;
}

static void pedestrian_config_cb(const runtime_manager::ConfigPedestrianDpm::ConstPtr& param)
{
	pedestrian_ttic_param.threshold = param->score_threshold;
	pedestrian_ttic_param.overlap   = param->group_threshold;
	pedestrian_ttic_param.lambda    = param->Lambda;
	pedestrian_ttic_param.num_cells = param->num_cells;
}

static std::string get_cubin_path(const ros::NodeHandle& n, const char *default_path)
{
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
	ros::init(argc, argv, "dpm_ttic");

	ros::NodeHandle n;

	std::string cubin = get_cubin_path(n, STR(DEFAULT_CUBIN));
	dpm_ttic_gpu_init_cuda(cubin);

	set_default_param(car_ttic_param);
	set_default_param(pedestrian_ttic_param);

	const char *car_com_csv  = STR(MODEL_DIR) "car_comp.csv";
	const char *car_root_csv = STR(MODEL_DIR) "car_root.csv";
	const char *car_part_csv = STR(MODEL_DIR) "car_part.csv";
	car_gpu_model = new DPMGPUModel(car_com_csv, car_root_csv, car_part_csv);
	car_ttic_model = new DPMTTICModel(car_com_csv, car_root_csv, car_part_csv);

	const char *pedestrian_com_csv  = STR(MODEL_DIR) "person_comp.csv";
	const char *pedestrian_root_csv = STR(MODEL_DIR) "person_root.csv";
	const char *pedestrian_part_csv = STR(MODEL_DIR) "person_part.csv";
	pedestrian_gpu_model = new DPMGPUModel(pedestrian_com_csv, pedestrian_root_csv, pedestrian_part_csv);
	pedestrian_ttic_model = new DPMTTICModel(pedestrian_com_csv, pedestrian_root_csv, pedestrian_part_csv);

	ros::Subscriber car_image_sub = n.subscribe("/image_raw", 1, image_raw_car_cb);
	car_image_obj_pub = n.advertise<cv_tracker::image_obj>("image_obj", 1);

	ros::Subscriber pedestrian_sub = n.subscribe("/image_raw", 1, image_raw_pedestrian_cb);
	pedestrian_image_obj_pub = n.advertise<cv_tracker::image_obj>("image_obj", 1);

	ros::Subscriber car_config_sub;
	car_config_sub = n.subscribe("/config/car_dpm", 1, car_config_cb);

	ros::Subscriber pedestrian_config_sub;
	pedestrian_config_sub = n.subscribe("/config/pedestrian_dpm", 1, pedestrian_config_cb);

	ros::spin();

	dpm_ttic_gpu_cleanup_cuda();
	delete car_gpu_model;
	delete pedestrian_gpu_model;
	delete car_ttic_model;
	delete pedestrian_ttic_model;

	return 0;
}
