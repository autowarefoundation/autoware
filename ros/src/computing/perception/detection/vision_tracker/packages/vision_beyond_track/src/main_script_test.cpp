#include "read_data.h"
#include "vision_beyond_track.h"
#include "detection.h"
#include "visualizer.h"

#include <iostream>
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>

static int seqNo = 2;
static int num_frame = 25;
static string image_dir = "../../Data/images/image_02/test/%04d/";
static string calib_path = "../../Data/calib/calib_all_test.txt";
static string det_dir = "../../Data/RRC_Detections_txt/test/%02d/";
static string pose_path = "../../Data/ORBSLAM_pose/test/%04d/KITTITrajectoryComplete_new";

static double avg_car_sz[3] = {4.3, 2, 2};
static double sz_ub[3] = {34, 31.5, 31.5};
static double sz_lb[3] = {-34, -31.5, -31.5};
ObjectCuboid Detection::params_carCuboid = ObjectCuboid(avg_car_sz, sz_ub, sz_lb);

vector<Detection> convert_detection(vector<vector<double>> raw_dets) {
  vector<Detection> detections;
  for (auto&& e: raw_dets) {
    Detection det(e);
    detections.push_back(det);
  }
  return detections;
}

int main(int argc, const char** argv) {
  for (int i=0; i<argc; i++) {
    std::cout << argv[i] << '\t';
  }
  std::cout << '\n';

  if (argc == 1) {
    std::cout << "Default parameter are used for inference\n";
  } else if (argc == 2) {
    seqNo = atoi(argv[1]);
  }

  image_dir = (boost::format(image_dir) % seqNo).str();
  det_dir = (boost::format(det_dir) % seqNo).str();
  pose_path = (boost::format(pose_path) % seqNo).str();

  std::cout << "Image dir: " << image_dir << '\n';
  std::cout << "Detection dir: " << det_dir << '\n';
  std::cout << "Pose dir: " << pose_path << '\n';
  std::cout << "Calib file: " << calib_path << '\n';

  cv::Mat k_ = read_calib(calib_path, seqNo);
  vector<vector<vector<double>>> detection_list = read_detection(det_dir);
  vector<cv::Mat> pose_list = read_pose(pose_path);
  vector<string> img_list = read_img(image_dir);

  cv::Mat n = cv::Mat::zeros(1, 3, CV_64FC1);
  n.at<double>(0, 1) = 1;
  double h = 1.72;

  std::cout << "K_:\n" << k_ << '\n';
  std::cout << "n:\n" << n << '\n';
  std::cout << "h: " << h << '\n';
  std::cout << '\n';

  beyondtrack::BeyondTracker tracker = beyondtrack::BeyondTracker(k_);

  for (int sn=0; sn<num_frame; ++sn) {
    std::cout << "----------------------------\n";
    std::cout << boost::format("Seq<%02d> | frame: %04d\n") % seqNo % sn;
    vector<Detection> detections = convert_detection(detection_list[sn]);
    auto start = std::chrono::high_resolution_clock::now(); //std::chrono::system_clock::now();

    tracker.process(detections, pose_list[sn], n, h);

    auto end = std::chrono::high_resolution_clock::now();//std::chrono::system_clock::now();
    auto dur = end - start;
    auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    std::cout << "Tracking: " << msec << " milli sec \n";

    visualize_results(img_list[sn], tracker.get_results());
  }
  return 0;
}
