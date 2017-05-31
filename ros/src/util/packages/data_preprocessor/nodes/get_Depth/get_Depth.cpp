#include "get_Depth.hpp"
// catkin_create_pkg data_preprocessor pcd_tutorial std_msgs roscpp pcl_ros pcl_msgs pcl_conversions libpcl-all-dev cv_bridge

// static cv::Mat CameraExtrinsicMat;
// static cv::Mat CameraMat;
// static cv::Mat DistCoeff;
// static cv::Size ImageSize;

static void check_arguments(int argc, char* argv[])
{
  if (argc != 5){
    cout << "Please set arguments like below\n'rosrun data_preprocessor get_PCD save_dir topic_name'\n";
    exit(EXIT_FAILURE);
  }
}

void SaveDepth::get_pcd(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  stringstream ss;
  ss << msg->header.stamp;
  // string file_name = save_path_ + ss.str() + ".pcd";
  // pcl::PointCloud<pcl::PointXYZ> points;
  // pcl::fromROSMsg(*msg, points_);
  // is_pc2_ = true;
  if (is_pc2_ == false){
    points_ = *msg;
    is_pc2_ = true;
  }
  // pcl::io::savePCDFileASCII(file_name, points);
}


void SaveDepth::get_image(const sensor_msgs::Image& msg)
{
  cv_bridge::CvImagePtr cv_image;
  stringstream ss;
  ss << msg.header.stamp;
  try
  {
    cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exeption: %s", e.what());
  }

  if (is_image_ == false){
    image_ = cv_image->image;
    time_stamp_ = ss.str();
    is_image_ = true;
  }
  // is_image_ = true;
  // cv::imwrite(save_path_ + ss.str() + ".jpg", cv_image->image);
}

void SaveDepth::read_CalibFile(){
  cv::FileStorage fs(calib_path_, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    cout << "Cannot Open " << calib_path_ << endl;
    exit(EXIT_FAILURE);
  }
  fs["CameraExtrinsicMat"] >> CameraExtrinsicMat_;
  fs["CameraMat"] >> CameraMat_;
  fs["DistCoeff"] >> DistCoeff_;
  fs["ImageSize"] >> ImageSize_;
}

void SaveDepth::reset_flags(){
  is_pc2_ = false;
  is_image_ = false;
}

void SaveDepth::create_depth(){
  // cout << CameraMat_ << endl;
  // cout << CameraExtrinsicMat_ << endl;
  // cout << DistCoeff_ << endl;
  // cout << points_.header << endl;
  // stringstream ss;
  // ss << width << " : " << height << endl;
  // cout << ss.str() << endl;
  // cout << points_.header << endl;
  int width = ImageSize_.width;
  int height = ImageSize_.height;
  cv::Mat invR = CameraExtrinsicMat_(cv::Rect(0,0,3,3)).t();
	cv::Mat invT = -invR*(CameraExtrinsicMat_(cv::Rect(3,0,1,3)));
  cv::Mat invR_T = invR.t();
  cv::Mat invT_T = invT.t();

  // cv::Mat depth = cv::Mat::zeros(cv::Size(width, height), CV_8U);
  cv::Mat depth = cv::Mat::zeros(cv::Size(width, height), CV_32F);
  // cv::imwrite(save_path_ + "otameshi" +".jpg", depth);
  // cout << "Height " << points_.height << " Width " << points_.height << endl;
  // cout << "Image height " << ImageSize_.width << " Height " << ImageSize_.height << endl;
  // cout << time_stamp_ << endl;
  // cout << points_.row_step << endl;
  // cout << points_.point_step << endl;
  pcl::PointCloud<pcl::PointXYZI> data_;
  pcl::fromROSMsg(points_, data_);
  int count = 0;



  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = data_.begin(); item!=data_.end(); item++){
    // cout << "x " << item->x << endl;
    // cout << "y " << item->y << endl;
    // cout << "z " << item->z << endl;
    // cout << item->intensity << endl;
    cv::Mat point(1, 3, CV_64F);
		point.at<double>(0) = double(item->x);
		point.at<double>(1) = double(item->y);
		point.at<double>(2) = double(item->z);
		point = point * invR_T + invT_T;

    // if (point.at<double>(2) <= 2.5) {
    //   continue;
    // }

    double tmpx = point.at<double>(0) / point.at<double>(2);
		double tmpy = point.at<double>(1)/point.at<double>(2);
		double r2 = tmpx * tmpx + tmpy * tmpy;
		double tmpdist = 1 + DistCoeff_.at<double>(0) * r2
			+ DistCoeff_.at<double>(1) * r2 * r2
			+ DistCoeff_.at<double>(4) * r2 * r2 * r2;

		cv::Point2d imagepoint;
		imagepoint.x = tmpx * tmpdist
			+ 2 * DistCoeff_.at<double>(2) * tmpx * tmpy
			+ DistCoeff_.at<double>(3) * (r2 + 2 * tmpx * tmpx);
		imagepoint.y = tmpy * tmpdist
			+ DistCoeff_.at<double>(2) * (r2 + 2 * tmpy * tmpy)
			+ 2 * DistCoeff_.at<double>(3) * tmpx * tmpy;
		imagepoint.x = CameraMat_.at<double>(0,0) * imagepoint.x + CameraMat_.at<double>(0,2);
		imagepoint.y = CameraMat_.at<double>(1,1) * imagepoint.y + CameraMat_.at<double>(1,2);
    int px = int(imagepoint.x + 0.5);
		int py = int(imagepoint.y + 0.5);

    // cout << "px " << px << endl;
    // cout << "py " << py << endl;

    if(0 <= px && px < width && 0 <= py && py < height){
      count = count + 1;
      if ((depth.at<double>(py, px) == 0) || (depth.at<double>(py, px) > point.at<double>(2))){
        // point.at<double>(2) = point.at<double>(2) * 2;
        depth.at<double>(py, px) = point.at<double>(2) / 255;
        // depth.at<int>(py+1, px) = int(point.at<double>(2));
        // depth.at<int>(py+2, px) = int(point.at<double>(2));
        // depth.at<int>(py+1, px+1) = int(point.at<double>(2));
        // depth.at<int>(py+2, px+1) = int(point.at<double>(2));
        // depth.at<int>(py-1, px) = int(point.at<double>(2));
        // depth.at<int>(py-1, px) = int(point.at<double>(2));
        // depth.at<int>(py-2, px-1) = int(point.at<double>(2));
        // depth.at<int>(py-2, px-1) = int(point.at<double>(2));

      }
      // double distance = point.at<double>(2);
      // cout << "max " << depth.at<double>(py, px) << endl;
      // depth.at<double>(50, 50) = 1;
      // exit(EXIT_FAILURE);
    }

    // exit(EXIT_FAILURE);
  }
  // cout << "count " << count << endl;
  double min, max;
  // cout << "ok1" << endl;
  cv::minMaxLoc(depth, &min, &max);
  // cout << "ok2 " << max  << endl;
  for (int w; w<width; w++){
    for (int h; h<height; h++){
      if (depth.at<double>(h, w) != 0){
        depth.at<double>(h, w) = depth.at<double>(h, w) / max;
      }
    }
  }
  cv::imwrite(save_path_ + "depth_" + time_stamp_ +".jpg", depth);
  cv::imwrite(save_path_ + "rgb_" + time_stamp_ +".jpg", image_);
  // exit(EXIT_FAILURE);
  // cout << data_ << endl;
  // cv::imwrite(save_path_ + "otameshi" +".jpg", depth);
  // cout << "func finish\n";
  reset_flags();
}

void SaveDepth::depthConverter(int argc, char* argv[]){
  ros::init(argc, argv, "Depth_Conveter");
  ros::NodeHandle n;
  ros::Subscriber image_sub = n.subscribe(image_topic_name_, 1, &SaveDepth::get_image, this);
  ros::Subscriber pc2_sub = n.subscribe(pc2_topic_name_, 1, &SaveDepth::get_pcd, this);
  ros::Rate r(100);
  while (ros::ok())
  {
    // cout << is_image_ << is_pc2_ << endl;
    if ((is_image_ == true) && (is_pc2_ == true)){
      create_depth();
      // cout << "finish\n";
    }
    ros::spinOnce();
    r.sleep();
  }
}

int main(int argc, char* argv[])
{
  check_arguments(argc, argv);
  SaveDepth si;
  string path = argv[1];
  if (path[path.size() - 1] == '/'){
    path.erase(path.begin() + path.size()-1);
  }
  si.save_path_ = path + '/';
  si.calib_path_ = argv[2];
  si.image_topic_name_ = argv[3];
  si.pc2_topic_name_ = argv[4];
  si.is_image_ = false;
  si.is_pc2_ = false;
  si.read_CalibFile();
  si.depthConverter(argc, argv);
  cout << "finish\n";
  return 0;
}
