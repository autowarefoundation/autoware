#include "LkTracker.hpp"


LkTracker::LkTracker(int in_id, float in_min_height, float in_max_height, float in_range)
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

	current_centroid_x_		= 0;
	current_centroid_y_		= 0;
	previous_centroid_x_	= 0;
	previous_centroid_y_	= 0;

#if (CV_MAJOR_VERSION == 3)
	generateColors(colors_, 2);
#else
	cv::generateColors(colors_, 2);
#endif
	lifespan_				= 45;
	DEFAULT_LIFESPAN_		= 45;
	object_id				= in_id;

	min_height_ 			= in_min_height;
	max_height_ 			= in_max_height;
	range_					= in_range;
}
ObjectDetection LkTracker::GetTrackedObject()
{
	return current_rect_;
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

/*void OrbFeatures(cv::Mat in_image)
{
	cv::OrbFeatureDetector orb(500);
	std::vector< cv::KeyPoint > keypoints;
	orb.detect(in_image, keypoints);

	cv::OrbDescriptorExtractor extractor;
	cv::Mat descriptors;
	cv::Mat training_descriptors(1, extractor.descriptorSize(), extractor.descriptorType());
	extractor.compute(in_image, keypoints, descriptors);
	training_descriptors.push_back(descriptors);

	cv::BOWKMeansTrainer bow_trainer(2);
	bow_trainer.add(descriptors);

	cv::Mat vocabulary = bow_trainer.cluster();
}*/

unsigned int	LkTracker::GetRemainingLifespan()
{
	return lifespan_;
}

void LkTracker::NullifyLifespan()
{
	lifespan_ = 0;
}

unsigned long int LkTracker::GetFrameCount()
{
	return frame_count_;
}

cv::Mat LkTracker::Track(cv::Mat in_image, ObjectDetection in_detection, bool in_update)
{
	cv::Mat gray_image;
	//cv::cvtColor(in_image, in_image, cv::COLOR_RGB2BGR);
	cv::cvtColor(in_image, gray_image, cv::COLOR_BGR2GRAY);
	cv::Mat mask(gray_image.size(), CV_8UC1);
	//cv::TickMeter timer;

	//timer.start();

	if (in_update && in_detection.rect.width > 0)
	{
		//MATCH
		matched_detection_ = in_detection;
		if (matched_detection_.rect.x < 0) matched_detection_.rect.x = 0;
		if (matched_detection_.rect.y < 0) matched_detection_.rect.y = 0;
		if (matched_detection_.rect.x + matched_detection_.rect.width > in_image.cols) matched_detection_.rect.width = in_image.cols - matched_detection_.rect.x;
		if (matched_detection_.rect.y + matched_detection_.rect.height > in_image.rows) matched_detection_.rect.height = in_image.rows - matched_detection_.rect.y;

		mask.setTo(cv::Scalar::all(0));
		mask(matched_detection_.rect) = 1;							//fill with ones only the ROI

		lifespan_ = DEFAULT_LIFESPAN_;
	}
	int sum_x = 0;
	int sum_y = 0;
	std::vector<cv::Point2f> valid_points;

	lifespan_--;
	if ( ( in_update || prev_image_.empty() ) &&
		 ( matched_detection_.rect.width>0 && matched_detection_.rect.height >0 )
		)																//add as new object
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
		if (current_points_.size()<=0)
		{
			ObjectDetection tmp_det; tmp_det.rect = cv::Rect(0,0,0,0); tmp_det.score=0;
			current_rect_ = tmp_det;
			return in_image;
		}
		cv::cornerSubPix(gray_image,
					current_points_,
					sub_pixel_window_size_,
					cv::Size(-1,-1),
					term_criteria_);
		//frame_count_ = 0;
		current_centroid_x_ = 0;
		current_centroid_y_ = 0;
		//current_points_.push_back(cv::Point(matched_detection_.x, matched_detection_.y));

		for (std::size_t i = 0; i < current_points_.size(); i++)
		{
			//cv::circle(in_image, current_points_[i], 3 , cv::Scalar(0,255,0), 2);
			current_centroid_x_+= current_points_[i].x;
			current_centroid_y_+= current_points_[i].y;
			valid_points.push_back(current_points_[i]);
		}
		//std::cout << "CENTROID" << current_centroid_x_ <<","<< current_centroid_y_<< std::endl << std::endl;

	}
	else if ( !prev_points_.empty() )//try to match current object
	{
		std::vector<uchar> status;
		std::vector<float> err;
		if(prev_image_.empty())
			in_image.copyTo(prev_image_);
		cv::calcOpticalFlowPyrLK(prev_image_, 			//previous image frame
								gray_image, 			//current image frame
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

		current_centroid_x_ = 0;
		current_centroid_y_ = 0;

		//process points
		for (i=0, k=0 ; i < prev_points_.size(); i++)
		{
			if( !status[i] )
			{
				continue;
			}
			cv::Point2f p,q;
			p.x = (int)prev_points_[i].x;		p.y = (int)prev_points_[i].y;
			q.x = (int)current_points_[i].x;	q.y = (int)current_points_[i].y;

			sum_y = p.y-q.y;
			sum_x = p.x -q.x;

			current_centroid_x_+= current_points_[i].x;
			current_centroid_y_+= current_points_[i].y;

			current_points_[k++] = current_points_[i];
			valid_points.push_back(current_points_[i]);
			//cv::circle(in_image, current_points_[i], 3 , cv::Scalar(0,255,0), 2);
		}
	}
	if (valid_points.size()<=2)
	{
		ObjectDetection tmp_det; tmp_det.rect = cv::Rect(0,0,0,0); tmp_det.score=0;
		current_rect_ = tmp_det;
		
		return in_image;
	}
	frame_count_++;

	cv::Mat labels;
	cv::Mat centers;

	cv::kmeans(valid_points,
					2,
					labels,
					cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
					3,
					cv::KMEANS_PP_CENTERS,
					centers);

	cv::Point2f center1 = centers.at<cv::Point2f>(0);
	cv::Point2f center2 = centers.at<cv::Point2f>(1);

	cv::Point centroid(current_centroid_x_/valid_points.size(), current_centroid_y_/valid_points.size());

	int cluster_nums[2] = {0,0};
	std::vector<cv::Scalar> colors(2);
	if (center1.x < center2.x || center1.x < center2.x)
	{
		colors[0]=colors_[0];
		colors[1]=colors_[1];
	}
	else
	{
		colors[0]=colors_[1];
		colors[1]=colors_[0];
	}

	//cv::circle(in_image, center1, 5 , (cv::Scalar)colors[0], 3);
	//cv::circle(in_image, center2, 5 , (cv::Scalar)colors[1], 3);

	std::vector<cv::Point2f> points_clusters[2];
	std::vector<cv::Point2f> close_points;
	//count points for each cluster
	for (std::size_t i = 0; i < valid_points.size(); i++)
	{
		int cluster_index = labels.at<int>(i);
		cluster_nums[cluster_index]++;
		points_clusters[cluster_index].push_back(valid_points[i]);
		if (cv::norm(valid_points[i] - center1) < matched_detection_.rect.width*0.8 &&
				cv::norm(valid_points[i] - center2) < matched_detection_.rect.width*0.8) //distance between point and cluster centroid
		{
			close_points.push_back(valid_points[i]);
		}
		//cv::circle(in_image, valid_points[i], 2, colors[cluster_index], 2);
	}

	std::vector<cv::Point2f> final_points;

	if (cv::norm(center2-center1) > matched_detection_.rect.width*0.75)//if centroids are too far keep only the one with the most points
	{
		if (cluster_nums[0] > cluster_nums[1])
			final_points = points_clusters[0];
		else
			final_points = points_clusters[1];
	}
	else
		final_points = close_points;

	GetRectFromPoints(final_points,
			current_rect_.rect);

	current_rect_.classID = matched_detection_.classID;
	current_rect_.score = matched_detection_.score;

	if (current_rect_.rect.width <= matched_detection_.rect.width*0.15 ||
			current_rect_.rect.height <= matched_detection_.rect.height*0.15
		)
	{
		//std::cout << "TRACK STOPPED" << std::endl;
		prev_points_.clear();
		current_points_.clear();
		ObjectDetection tmp_det; tmp_det.rect = cv::Rect(0,0,0,0); tmp_det.score=0;
		current_rect_ = tmp_det;
		return in_image;
	}

	//cv::rectangle(in_image, current_rect_, cv::Scalar(0,0,255), 2);

	if (prev_points_.size() > 0 )
	{
		cv::Point center_point = cv::Point(current_rect_.rect.x + current_rect_.rect.width/2, current_rect_.rect.y + current_rect_.rect.height/2);
		cv::Point direction_point;
		float sum_angle = atan2(sum_y, sum_x);

		direction_point.x = (center_point.x - 100 * cos(sum_angle));
		direction_point.y = (center_point.y - 100 * sin(sum_angle));

		//ArrowedLine(in_image, center_point, direction_point, cv::Scalar(0,0,255), 2);

		//cv::circle(in_image, center_point, 4 , cv::Scalar(0,0,255), 2);
		//cv::circle(in_image, center_point, 2 , cv::Scalar(0,0,255), 2);
	}

	//finally store current state into previous
	std::swap(final_points, prev_points_);
	cv::swap(prev_image_, gray_image);

	if (current_centroid_x_ > 0 && current_centroid_y_ > 0)
	{
		previous_centroid_x_ = current_centroid_x_;
		previous_centroid_y_ = current_centroid_y_;
	}

	//timer.stop();

	//std::cout << timer.getTimeMilli() << std::endl;

	return in_image;
}

void LkTracker::GetRectFromPoints(std::vector< cv::Point2f > in_corners_points, cv::Rect& out_boundingbox)
{

	if (in_corners_points.empty())
	{
		return;
	}

	int min_x=in_corners_points[0].x, min_y=in_corners_points[0].y, max_x=in_corners_points[0].x, max_y=in_corners_points[0].y;

	for (unsigned int i=0; i<in_corners_points.size(); i++)
	{
		if (in_corners_points[i].x > 0 )
		{
			if (in_corners_points[i].x < min_x)
				min_x = in_corners_points[i].x;
			if (in_corners_points[i].x > max_x)
				max_x = in_corners_points[i].x;
		}

		if (in_corners_points[i].y > 0 )
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

	return;
}

