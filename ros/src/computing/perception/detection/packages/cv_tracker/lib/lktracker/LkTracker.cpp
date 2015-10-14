#include "LkTracker.hpp"


LkTracker::LkTracker()
{
	max_point_count_ 		= 500;
	criteria_max_iteration_	= 20;
	criteria_epsilon_		= 0.03;
	corner_window_size_		= 31;
	corner_subwindow_size_	= 10;
	term_criteria_ 			= cv::TermCriteria(	CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,	//type
										criteria_max_iteration_, 					//max iteration count
										criteria_epsilon_							//epsilon
										);
	sub_pixel_window_size_ 	= cv::Size(corner_subwindow_size_, corner_subwindow_size_);
	window_size_ 			= cv::Size(corner_window_size_, corner_window_size_);

	frame_count_			= 0;
}

void LkTracker::Track(cv::Mat in_image, cv::Rect in_detection, bool in_update)
{
	cv::Mat gray_image;
	cv::cvtColor(in_image, in_image, cv::COLOR_RGB2BGR);
	cv::cvtColor(in_image, gray_image, cv::COLOR_BGR2GRAY);
	cv::Mat mask(gray_image.size(), CV_8UC1);

	if (in_detection.x < 0) in_detection.x = 0;
	if (in_detection.y < 0) in_detection.y = 0;
	if (in_detection.x + in_detection.width > in_image.cols) in_detection.width = in_image.cols - in_detection.x;
	if (in_detection.y + in_detection.height > in_image.rows) in_detection.height = in_image.rows - in_detection.y;

	mask.setTo(cv::Scalar::all(0));
	mask(in_detection) = 1;							//fill with ones only the ROI

	if ( (in_detection.width>0 && in_detection.height >0 ) &&
		( (in_update && (frame_count_%5 == 0)) || prev_image_.empty() ))						//add as new object
	{
		cv::goodFeaturesToTrack(gray_image,			//input to extract corners
								current_points_,	//out array with corners in the image
								max_point_count_,	//maximum number of corner points to obtain
								0.01,				//quality level
								10,					//minimum distance between corner points
								mask,//mask ROI
								3,					//block size
								true,				//true to use harris corner detector, otherwise use tomasi
								0.04);				//harris detector free parameter
		/*cv::cornerSubPix(gray_image,
					current_points_,
					sub_pixel_window_size_,
					cv::Size(-1,-1),
					term_criteria_);*/
		//frame_count_ = 0;
		for (std::size_t i = 0; i < prev_points_.size(); i++)
		{
			cv::circle(in_image, current_points_[i], 3 , cv::Scalar(0,255,0), 2);
		}
	}
	else if ( !prev_points_.empty() )//try to match current object
	{
		std::vector<uchar> status;
		std::vector<float> err;
		if(prev_image_.empty())
			in_image.copyTo(prev_image_);
		cv::calcOpticalFlowPyrLK(prev_image_, 			//previous image frame
								gray_image, 					//current image frame
								prev_points_, 			//previous corner points
								current_points_, 		//current corner points (tracked)
								status,
								err,
								window_size_,
								3,
								term_criteria_,
								0,
								0.001);
		std::size_t i = 0, k = 0;
		int sum_angle = 0;
		for (i=0, k=0 ; i < prev_points_.size(); i++)
		{
			if( !status[i] )
				continue;




			current_points_[k++] = current_points_[i];
			cv::circle(in_image, current_points_[i], 3 , cv::Scalar(0,255,0), 2);
		}

		current_points_.resize(k);

		frame_count_++;
	}
	cv::Rect obj_rect;
	GetRectFromPoints(current_points_, obj_rect);

	if (prev_points_.size() > 0 )
	{
		cv::Point center_point = cv::Point(obj_rect.x + obj_rect.width/2, obj_rect.y + obj_rect.height/2);

		cv::Point direction_point;
		direction_point.x = (center_point.x - 100 * cos(sum_angle));
		direction_point.y = (center_point.y - 100 * sin(sum_angle));

		cv::line(in_image, center_point, direction_point, cv::Scalar(0,0,255), 2);
	}
	cv::rectangle(in_image, obj_rect, cv::Scalar(0,0,255), 2);



	imshow("KLT debug", in_image);
	cvWaitKey(1);
	//finally store current state into previous
	std::swap(current_points_, prev_points_);
	cv::swap(prev_image_, gray_image);
}

void LkTracker::GetRectFromPoints(std::vector< cv::Point2f > corners, cv::Rect& outBoundingBox)
{
	if (corners.empty())
		return;

	int min_x=corners[0].x, min_y=corners[0].y, max_x=corners[0].x, max_y=corners[0].y;

	for (unsigned int i=0; i<corners.size(); i++)
	{
		if (corners[i].x > 0)
		{
			if (corners[i].x < min_x)
				min_x = corners[i].x;
			if (corners[i].x > max_x)
				max_x = corners[i].x;
		}
		if (corners[i].y > 0)
		{
			if (corners[i].y < min_y)
				min_y = corners[i].y;
			if (corners[i].y > max_y)
				max_y = corners[i].y;
		}
	}
	outBoundingBox.x 		= min_x;
	outBoundingBox.y 		= min_y;
	outBoundingBox.width 	= max_x - min_x;
	outBoundingBox.height 	= max_y - min_y;

}

