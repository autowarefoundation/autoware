#include "darknet/yolo2.h"

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include <rect_class_score.h>

extern "C"
{
	#undef __cplusplus
		#include "detection_layer.h"
		#include "parser.h"
		#include "region_layer.h"
		#include "utils.h"
		#include "image.h"
	#define __cplusplus
}

namespace darknet
{
	uint32_t Yolo2Detector::get_network_height()
	{
		return darknet_network_.h;
	}
	uint32_t Yolo2Detector::get_network_width()
	{
		return darknet_network_.w;
	}
	void Yolo2Detector::load(std::string& in_model_file, std::string& in_trained_file, double in_min_confidence, double in_nms_threshold)
	{
		min_confidence_ = in_min_confidence;
		nms_threshold_ = in_nms_threshold;
		darknet_network_ = parse_network_cfg(&in_model_file[0]);
		load_weights(&darknet_network_, &in_trained_file[0]);
		set_batch_network(&darknet_network_, 1);

		layer output_layer = darknet_network_.layers[darknet_network_.n - 1];
		darknet_boxes_.resize(output_layer.w * output_layer.h * output_layer.n);
		darknet_box_scores_.resize(output_layer.w * output_layer.h * output_layer.n);
		float *probs_mem = static_cast<float *>(calloc(darknet_box_scores_.size() * output_layer.classes, sizeof(float)));
		for (auto& i : darknet_box_scores_)
		{
			i = probs_mem;
			probs_mem += output_layer.classes;
		}
	}

	Yolo2Detector::~Yolo2Detector()
	{
		free(darknet_box_scores_[0]);
		free_network(darknet_network_);
	}

	std::vector< RectClassScore<float> > Yolo2Detector::detect(image& in_darknet_image)
	{
		return forward(in_darknet_image);
	}

	image Yolo2Detector::convert_image(const sensor_msgs::ImageConstPtr& msg)
	{
		if (msg->encoding != sensor_msgs::image_encodings::BGR8)
		{
			ROS_ERROR("Unsupported encoding");
			exit(-1);
		}

		auto data = msg->data;
		uint32_t height = msg->height, width = msg->width, offset = msg->step - 3 * width;
		uint32_t i = 0, j = 0;
		image im = make_image(width, height, 3);

		for (uint32_t line = height; line; line--)
		{
			for (uint32_t column = width; column; column--)
			{
				for (uint32_t channel = 0; channel < 3; channel++)
					im.data[i + width * height * channel] = data[j++] / 255.;
				i++;
			}
			j += offset;
		}

		if (darknet_network_.w == (int) width && darknet_network_.h == (int) height)
		{
			return im;
		}
		image resized = resize_image(im, darknet_network_.w, darknet_network_.h);
		free_image(im);
		return resized;
	}

	std::vector< RectClassScore<float> > Yolo2Detector::forward(image& in_darknet_image)
	{
		float * in_data = in_darknet_image.data;
		float *prediction = network_predict(darknet_network_, in_data);
		layer output_layer = darknet_network_.layers[darknet_network_.n - 1];

		output_layer.output = prediction;
		if (output_layer.type == DETECTION)
			get_detection_boxes(output_layer, 1, 1, min_confidence_, darknet_box_scores_.data(), darknet_boxes_.data(), 0);
		else if (output_layer.type == REGION)
		{
			get_region_boxes(output_layer, in_darknet_image.w, in_darknet_image.h,
							darknet_network_.w, darknet_network_.h,
							min_confidence_, darknet_box_scores_.data(), darknet_boxes_.data(),
							0, 0, 0.5, 1);
		}
		else
			error("Last layer must produce detections\n");

		int num_classes = output_layer.classes;
		do_nms(darknet_boxes_.data(), darknet_box_scores_.data(), output_layer.w * output_layer.h * output_layer.n, num_classes, nms_threshold_);
		std::vector< RectClassScore<float> > detections;

		for (unsigned i = 0; i < darknet_box_scores_.size(); i++)
		{
			int class_id = max_index(darknet_box_scores_[i], num_classes);
			float prob = darknet_box_scores_[i][class_id];
			//if (prob > 0.3)
			{
				RectClassScore<float> detection;
				box b = darknet_boxes_[i];

				detection.x = b.x - b.w/2.;
				detection.y = b.y - b.h/2.;
				detection.w = b.w;
				detection.h = b.h;
				detection.score = prob;
				detection.class_type = class_id;
				//std::cout << "Box:"  <<detection.toString() << std::endl;

				detections.push_back(detection);
			}
		}
		return detections;
	}
}  // namespace darknet
