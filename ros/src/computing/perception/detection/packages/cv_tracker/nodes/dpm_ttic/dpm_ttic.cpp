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

#include <cstdio>
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

static ros::Publisher image_obj_pub;

#if defined(HAS_GPU)
static DPMTTICGPU *gpu_model;
static bool use_gpu = true;
#endif
static DPMTTIC *ttic_model;

static DPMTTICParam ttic_param;

static std::string object_class;static long int counter;

static void set_default_param(DPMTTICParam& param)
{
	param.overlap = 0.4;
	param.threshold = -0.5;
	param.lambda = 10;
	param.num_cells = 8;counter =0;
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
		rect.score = result.score[i];

		msg.obj.push_back(rect);
	}
}

static void image_raw_cb(const sensor_msgs::Image& image_source)
{

	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
	IplImage img = cv_image->image;
	IplImage *img_ptr = &img;

	cv_tracker::image_obj msg;
	msg.header = image_source.header;
	msg.type = object_class;

#if defined(HAS_GPU)
	if (use_gpu) {
		DPMTTICResult result = gpu_model->detect_objects(img_ptr, ttic_param);
		result_to_image_obj_message(msg, result);
	} else {
#endif
		DPMTTICResult result = ttic_model->detect_objects(img_ptr, ttic_param);
		result_to_image_obj_message(msg, result);
#if defined(HAS_GPU)
	}
#endif

	image_obj_pub.publish(msg);
	counter++;
}

static void config_cb(const runtime_manager::ConfigPedestrianDpm::ConstPtr& param)
{
	ttic_param.threshold = param->score_threshold;
	ttic_param.overlap   = param->group_threshold;
	ttic_param.lambda    = param->Lambda;
	ttic_param.num_cells = param->num_cells;
}

#if defined(HAS_GPU)
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
#endif

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "dpm_ttic");

	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	if (!private_nh.getParam("detection_class_name", object_class)) {
		object_class = "car";
	}

	std::string comp_csv_path;
	if (!private_nh.getParam("comp_model_path", comp_csv_path)) {
		comp_csv_path = STR(MODEL_DIR) "car_comp.csv";
	}

	std::string root_csv_path;
	if (!private_nh.getParam("root_model_path", root_csv_path)) {
		root_csv_path = STR(MODEL_DIR) "car_root.csv";
	}

	std::string part_csv_path;
	if (!private_nh.getParam("part_model_path", part_csv_path)) {
		part_csv_path = STR(MODEL_DIR) "car_part.csv";
	}

#if defined(HAS_GPU)
	if (!private_nh.getParam("use_gpu", use_gpu)) {
		use_gpu = false;
	}

	std::string cubin = get_cubin_path(n, STR(DEFAULT_CUBIN));
	if (use_gpu) {
		dpm_ttic_gpu_init_cuda(cubin);
	}
#endif

	set_default_param(ttic_param);

	const char *com_csv  = comp_csv_path.c_str();
	const char *root_csv = root_csv_path.c_str();
	const char *part_csv = part_csv_path.c_str();

#if defined(HAS_GPU)
	if (use_gpu) {
		gpu_model = new DPMTTICGPU(com_csv, root_csv, part_csv);
	} else {
#endif
		ttic_model = new DPMTTIC(com_csv, root_csv, part_csv);
#if defined(HAS_GPU)
	}
#endif

	ros::Subscriber sub = n.subscribe("/image_raw", 1, image_raw_cb);
	image_obj_pub = n.advertise<cv_tracker::image_obj>("image_obj", 1);

	ros::Subscriber config_sub;
	std::string config_topic("/config");
	config_topic += ros::this_node::getNamespace() + "/dpm";
	config_sub = n.subscribe(config_topic, 1, config_cb);

	ros::spin();
#if defined(HAS_GPU)
	if (use_gpu) {
		dpm_ttic_gpu_cleanup_cuda();
		delete gpu_model;
	} else {
#endif
		delete ttic_model;
#if defined(HAS_GPU)
	}
#endif

	return 0;
}
