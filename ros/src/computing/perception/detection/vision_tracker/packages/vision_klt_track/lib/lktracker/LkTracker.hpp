#ifndef LKTRACKER_HPP_
#define LKTRACKER_HPP_

#include <stdio.h>
#include <math.h>
#include <iostream>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <opencv2/core/version.hpp>
#if (CV_MAJOR_VERSION == 3)
#include "gencolors.cpp"
#else
#include <opencv2/contrib/contrib.hpp>
#endif

struct ObjectDetection
{
	//ObjectDetection();
	//ObjectDetection(const cv::Rect& rect, float score, int classId=1);
	cv::Rect rect;
	float score;
	int classID;
};

class LkTracker
{
	int 					max_point_count_;
	int						criteria_max_iteration_;
	float 					criteria_epsilon_;
	int						corner_window_size_;
	int						corner_subwindow_size_;

	unsigned long int 		frame_count_;
	unsigned int 			lifespan_;

	cv::Mat 				prev_image_;
	cv::Mat 				current_image_;
	cv::TermCriteria 		term_criteria_;
	cv::Size 				sub_pixel_window_size_;
	cv::Size 				window_size_;

	ObjectDetection	matched_detection_;
	ObjectDetection	current_rect_;//stores the current tracked object

	int 					current_centroid_x_;
	int 					current_centroid_y_;

	int 					previous_centroid_x_;
	int 					previous_centroid_y_;

	std::vector<cv::Scalar>	colors_;

	std::vector<cv::Point2f> prev_points_;
	std::vector<cv::Point2f> current_points_;
	void 					GetRectFromPoints(std::vector< cv::Point2f > in_corners_points, cv::Rect& out_boundingbox);
	void 					ArrowedLine(cv::Mat& in_image, cv::Point in_point1, cv::Point in_point2, const cv::Scalar& in_color,
								int in_thickness=1, int in_line_type=8, int in_shift=0, double in_tip_length=0.1);
public:
	int						object_id;
	float					min_height_;
	float					max_height_;
	float					range_;
	unsigned int 			DEFAULT_LIFESPAN_;


	LkTracker(int in_id, float in_min_height, float in_max_height, float in_range);
	cv::Mat 								Track(cv::Mat image, ObjectDetection in_detections, bool in_update);
	ObjectDetection	GetTrackedObject();
	unsigned int							GetRemainingLifespan();
	void 									NullifyLifespan();
	unsigned long int						GetFrameCount();
};

extern int klt_main(int argc, char* argv[]);

#endif /* LKTRACKER_HPP_ */
