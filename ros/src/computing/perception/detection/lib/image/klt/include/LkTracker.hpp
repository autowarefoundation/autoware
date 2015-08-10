#ifndef LKTRACKER_HPP_
#define LKTRACKER_HPP_

#include <stdio.h>
#include <math.h>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>


class LkTracker
{
	int 		max_point_count_;
	int			criteria_max_iteration_;
	float 		criteria_epsilon_;
	int			corner_window_size_;
	int			corner_subwindow_size_;

	int			frame_count_;

	cv::Mat 	prev_image_;
	cv::Mat 	current_image_;
	cv::TermCriteria term_criteria_;
	cv::Size 	sub_pixel_window_size_;
	cv::Size 	window_size_;

	std::vector<cv::Point2f> prev_points_;
	std::vector<cv::Point2f> current_points_;
	void GetRectFromPoints(std::vector< cv::Point2f > corners, cv::Rect& outBoundingBox);
public:
	LkTracker();
	void Track(cv::Mat image, cv::Rect detection);
};


#endif /* LKTRACKER_HPP_ */
