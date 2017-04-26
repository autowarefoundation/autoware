#ifndef DARKNET_YOLO2_H
#define DARKNET_YOLO2_H

#include <image_transport/image_transport.h>

#include <string>
#include <vector>

#include <rect_class_score.h>

extern "C"
{
	#undef __cplusplus
		#include "box.h"
		#include "image.h"
		#include "network.h"
	#define __cplusplus
}

namespace darknet
{
	class Yolo2Detector
	{
		private:
			std::vector< RectClassScore<float> > forward(float *data);

			double min_confidence_, nms_threshold_;
			network darknet_network_;
			std::vector<box> darknet_boxes_;
			std::vector<float *> darknet_box_scores_;
		public:
			Yolo2Detector() {}
			void load(std::string& in_model_file, std::string& in_trained_file, double in_min_confidence, double in_nms_threshold);
			~Yolo2Detector();
			image convert_image(const sensor_msgs::ImageConstPtr& in_image_msg);
			std::vector< RectClassScore<float> > detect(float *in_data);


	};
}  // namespace darknet

#endif  // DARKNET_YOLO2_H
