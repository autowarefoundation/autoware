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

void LkTracker::ArrowedLine(cv::Mat& in_image, cv::Point in_point1, cv::Point in_point2, const cv::Scalar& in_color,
				int in_thickness, int in_line_type, int in_shift, double in_tip_length)
{
	const double tipSize = cv::norm(in_point1-in_point2) * in_tip_length; // Factor to normalize the size of the tip depending on the length of the arrow
	cv::line(in_image, in_point1, in_point2, in_color, in_thickness, in_line_type, in_shift);

	const double angle = atan2( (double) in_point1.y - in_point2.y, (double) in_point1.x - in_point2.x );
	cv::Point p(cvRound(in_point2.x + tipSize * cos(angle + CV_PI / 4)),
	cvRound(in_point2.y + tipSize * sin(angle + CV_PI / 4)));

	cv::line(in_image, p, in_point2, in_color, in_thickness, in_line_type, in_shift);

	p.x = cvRound(in_point2.x + tipSize * cos(angle - CV_PI / 4));
	p.y = cvRound(in_point2.y + tipSize * sin(angle - CV_PI / 4));

	cv::line(in_image, p, in_point2, in_color, in_thickness, in_line_type, in_shift);
}

cv::Mat LkTracker::Track(cv::Mat in_image, cv::Rect in_detection, bool in_update)
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
	int sum_angle = 0;

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

		for (i=0, k=0 ; i < prev_points_.size(); i++)
		{
			if( !status[i] )
				continue;
			cv::Point2f p,q;
			p.x = (int)prev_points_[i].x;		p.y = (int)prev_points_[i].y;
			q.x = (int)current_points_[i].x;	q.y = (int)current_points_[i].y;

			sum_angle += atan2(p.y-q.y, p.x -q.x);

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

		ArrowedLine(in_image, center_point, direction_point, cv::Scalar(0,0,255), 2);
	}
	cv::rectangle(in_image, obj_rect, cv::Scalar(0,0,255), 2);

	//imshow("KLT debug", in_image);
	//cvWaitKey(1);
	//finally store current state into previous
	std::swap(current_points_, prev_points_);
	cv::swap(prev_image_, gray_image);

	return in_image;
}

void LkTracker::GetRectFromPoints(std::vector< cv::Point2f > in_corners_points, cv::Rect& out_boundingbox)
{
	if (in_corners_points.empty())
		return;

	int min_x=in_corners_points[0].x, min_y=in_corners_points[0].y, max_x=in_corners_points[0].x, max_y=in_corners_points[0].y;

	for (unsigned int i=0; i<in_corners_points.size(); i++)
	{
		if (in_corners_points[i].x > 0)
		{
			if (in_corners_points[i].x < min_x)
				min_x = in_corners_points[i].x;
			if (in_corners_points[i].x > max_x)
				max_x = in_corners_points[i].x;
		}
		if (in_corners_points[i].y > 0)
		{
			if (in_corners_points[i].y < min_y)
				min_y = in_corners_points[i].y;
			if (in_corners_points[i].y > max_y)
				max_y = in_corners_points[i].y;
		}
	}
	out_boundingbox.x 		= min_x;
	out_boundingbox.y 		= min_y;
	out_boundingbox.width 	= max_x - min_x;
	out_boundingbox.height 	= max_y - min_y;

}

