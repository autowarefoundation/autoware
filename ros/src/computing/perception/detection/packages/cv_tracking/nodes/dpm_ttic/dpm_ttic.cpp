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

#include <dpm/ImageObjects.h>
#include <runtime_manager/ConfigCarDpm.h>
#include <runtime_manager/ConfigPedestrianDpm.h>

#include <dpm_gpu.hpp>

#define XSTR(x) #x
#define STR(x) XSTR(x)

static ros::Publisher car_image_obj_pub;
static ros::Publisher pedestrian_image_obj_pub;

static DPMGPUModel *car_model;
static DPMGPUModel *pedestrian_model;

static DPMGPUParam car_param;
static DPMGPUParam pedestrian_param;

static void set_default_param(DPMGPUParam& param)
{
	param.overlap = 0.4;
	param.threshold = -0.5;
	param.lambda = 10;
	param.num_cells = 8;
}

static void image_raw_car_cb(const sensor_msgs::Image& image_source)
{
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
	IplImage img = cv_image->image;
	IplImage *img_ptr = &img;

	DPMGPUResult result = car_model->detect_objects(img_ptr, car_param);

	dpm::ImageObjects message;
	message.header = image_source.header;
	message.car_num = result.num;
	message.corner_point = result.corner_points;
	message.car_type = result.type;
	message.header.stamp = image_source.header.stamp;
	message.score = result.score;

	car_image_obj_pub.publish(message);
}

static void image_raw_pedestrian_cb(const sensor_msgs::Image& image_source)
{
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
	IplImage img = cv_image->image;
	IplImage *img_ptr = &img;

	DPMGPUResult result = pedestrian_model->detect_objects(img_ptr, pedestrian_param);

	dpm::ImageObjects message;
	message.header = image_source.header;
	message.car_num = result.num;
	message.corner_point = result.corner_points;
	message.car_type = result.type;
	message.header.stamp = image_source.header.stamp;
	message.score = result.score;

	pedestrian_image_obj_pub.publish(message);
}

static void car_config_cb(const runtime_manager::ConfigCarDpm::ConstPtr& param)
{
	car_param.threshold = param->score_threshold;
	car_param.overlap   = param->group_threshold;
	car_param.lambda    = param->Lambda;
	car_param.num_cells = param->num_cells;
}

static void pedestrian_config_cb(const runtime_manager::ConfigPedestrianDpm::ConstPtr& param)
{
	pedestrian_param.threshold = param->score_threshold;
	pedestrian_param.overlap   = param->group_threshold;
	pedestrian_param.lambda    = param->Lambda;
	pedestrian_param.num_cells = param->num_cells;
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
	ros::init(argc, argv, "dpm_ttic");

	ros::NodeHandle n;

	std::string cubin = get_cubin_path(n, STR(DEFAULT_CUBIN));
	dpm_gpu_init_cuda(cubin);

	set_default_param(car_param);
	set_default_param(pedestrian_param);

	const char *car_com_csv  = STR(MODEL_DIR) "car_comp.csv";
	const char *car_root_csv = STR(MODEL_DIR) "car_root.csv";
	const char *car_part_csv = STR(MODEL_DIR) "car_part.csv";
	car_model = new DPMGPUModel(car_com_csv, car_root_csv, car_part_csv);

	const char *pedestrian_com_csv  = STR(MODEL_DIR) "person_comp.csv";
	const char *pedestrian_root_csv = STR(MODEL_DIR) "person_root.csv";
	const char *pedestrian_part_csv = STR(MODEL_DIR) "person_part.csv";
	pedestrian_model = new DPMGPUModel(pedestrian_com_csv, pedestrian_root_csv, pedestrian_part_csv);

	ros::Subscriber car_image_sub = n.subscribe("/image_raw", 1, image_raw_car_cb);
	car_image_obj_pub = n.advertise<dpm::ImageObjects>("image_obj", 1);

	ros::Subscriber pedestrian_sub = n.subscribe("/image_raw", 1, image_raw_pedestrian_cb);
	pedestrian_image_obj_pub = n.advertise<dpm::ImageObjects>("image_obj", 1);

	ros::Subscriber car_config_sub;
	car_config_sub = n.subscribe("/config/car_dpm", 1, car_config_cb);

	ros::Subscriber pedestrian_config_sub;
	pedestrian_config_sub = n.subscribe("/config/pedestrian_dpm", 1, pedestrian_config_cb);

	ros::spin();

	dpm_gpu_cleanup_cuda();
	delete car_model;
	delete pedestrian_model;

	return 0;
}
