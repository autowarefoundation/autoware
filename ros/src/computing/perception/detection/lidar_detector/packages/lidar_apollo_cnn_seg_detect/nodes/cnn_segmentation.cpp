#include "cnn_segmentation.h"

bool CNNSegmentation::init() {
  // std::string config_file;
  std::string proto_file;
  std::string weight_file;

  // config_file = "/home/kosuke/codes/ros/catkin_ws/src/lidar_apollo_cnn_seg_detect/nodes/modules/perception/model/cnn_segmentation/cnnseg.conf";
  proto_file  = "/home/kosuke/apollo/modules/perception/model/cnn_segmentation/deploy.prototxt";
  weight_file = "/home/kosuke/apollo/modules/perception/model/cnn_segmentation/deploy.caffemodel";


  range_ = 60.0;

  width_ = 512;

  height_ = 512;

/// Instantiate Caffe net
#ifndef USE_CAFFE_GPU
  caffe::Caffe::set_mode(caffe::Caffe::CPU);
#else
  int gpu_id = 0;
  caffe::Caffe::SetDevice(gpu_id);
  caffe::Caffe::set_mode(caffe::Caffe::GPU);
  caffe::Caffe::DeviceQuery();
#endif

  caffe_net_.reset(new caffe::Net<float>(proto_file, caffe::TEST));
  caffe_net_->CopyTrainedLayersFrom(weight_file);


  std::string instance_pt_blob_name = "instance_pt";
  instance_pt_blob_ = caffe_net_->blob_by_name(instance_pt_blob_name);
  CHECK(instance_pt_blob_ != nullptr) << "`" << instance_pt_blob_name
                                      << "` not exists!";

  std::string category_pt_blob_name = "category_score";
  category_pt_blob_ = caffe_net_->blob_by_name(category_pt_blob_name);
  CHECK(category_pt_blob_ != nullptr) << "`" << category_pt_blob_name
                                      << "` not exists!";

  std::string confidence_pt_blob_name = "confidence_score";
  confidence_pt_blob_ = caffe_net_->blob_by_name(confidence_pt_blob_name);
  CHECK(confidence_pt_blob_ != nullptr) << "`" << confidence_pt_blob_name
                                        << "` not exists!";

  std::string height_pt_blob_name = "height_pt";
  height_pt_blob_ = caffe_net_->blob_by_name(height_pt_blob_name);
  CHECK(height_pt_blob_ != nullptr) << "`" << height_pt_blob_name
                                    << "` not exists!";

  std::string feature_blob_name = "data";
  feature_blob_ = caffe_net_->blob_by_name(feature_blob_name);
  CHECK(feature_blob_ != nullptr) << "`" << feature_blob_name
                                  << "` not exists!";

  std::string class_pt_blob_name = "class_score";
  class_pt_blob_ = caffe_net_->blob_by_name(class_pt_blob_name);
  CHECK(class_pt_blob_ != nullptr) << "`" << class_pt_blob_name
                                   << "` not exists!";

  cluster2d_.reset(new Cluster2D());
  if (!cluster2d_->init(height_, width_, range_)) {
    std::cout << "Fail to Init cluster2d for CNNSegmentation" << std::endl;
  }

  feature_generator_.reset(new FeatureGenerator());
  if (!feature_generator_->init(feature_blob_.get())) {
    std::cout << "Fail to Init feature generator for CNNSegmentation" << std::endl;
    return false;
  }

  return true;
}

bool CNNSegmentation::segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr,
                              const pcl::PointIndices& valid_idx,
                              autoware_msgs::DetectedObjectArray* objects) {
  int num_pts = static_cast<int>(pc_ptr->points.size());
  if (num_pts == 0) {
    std::cout << "None of input points, return directly." << std::endl;
    return true;
  }

  feature_generator_->generate(pc_ptr);

// network forward process
#ifdef USE_CAFFE_GPU
  caffe::Caffe::set_mode(caffe::Caffe::GPU);
#endif
  caffe_net_->Forward();
  // PERF_BLOCK_END("[CNNSeg] CNN forward");
//
  // clutser points and construct segments/objects
  float objectness_thresh = 0.5;
  bool use_all_grids_for_clustering = true;
  cluster2d_->cluster(*category_pt_blob_, *instance_pt_blob_, pc_ptr,
                      valid_idx, objectness_thresh,
                      use_all_grids_for_clustering);
  cluster2d_->filter(*confidence_pt_blob_, *height_pt_blob_);
  cluster2d_->classify(*class_pt_blob_);
  float confidence_thresh = 0.75;
  float height_thresh = 0.5;
  int min_pts_num = 3;
  cluster2d_->getObjects(confidence_thresh, height_thresh, min_pts_num,
                         objects);
  return true;
}

void CNNSegmentation::run() {
  std::string in_pcd_file = "/home/kosuke/apollo/modules/perception/data/cnnseg_test/uscar_12_1470770225_1470770492_1349.pcd";
  // apollo::PointCloudPtr in_pc;
  pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile(in_pcd_file, *in_pc_ptr);

  pcl::PointIndices valid_idx;
  auto& indices = valid_idx.indices;
  indices.resize(in_pc_ptr->size());
  std::iota(indices.begin(), indices.end(), 0);

  autoware_msgs::DetectedObjectArray objects;
  init();
  for (int i = 0; i < 10; ++i) {
    segment(in_pc_ptr, valid_idx, &objects);
    // EXPECT_TRUE(
    //     cnn_segmentor_->Segment(in_pc, valid_idx, options, &out_objects));
    // EXPECT_EQ(out_objects.size(), 15);
    std::cout << "size " << objects.objects.size() << std::endl;
  }
  // segment(in_pc_ptr, valid_idx, &objects);

}


// void CNNSegmentation::drawDetection(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr,
//                    const pcl::PointIndices& valid_idx,
//                    int rows, int cols, float range,
//                    const autoware_msgs::DetectedObjectArray& objects,
//                    const std::string &result_file) {
//   // create a new image for visualization
//   cv::Mat img(rows, cols, CV_8UC3, cv::Scalar(0.0));
//
//   // map points into bird-view grids
//   float inv_res_x = 0.5 * static_cast<float>(cols) / range;
//   float inv_res_y = 0.5 * static_cast<float>(rows) / range;
//   int grids = rows * cols;
//   std::vector<CellStat> view(grids);
//
//   const std::vector<int> *valid_indices_in_pc = &(valid_idx.indices);
//   CHECK_LE(valid_indices_in_pc->size(), pc_ptr->size());
//   unordered_set<int> unique_indices;
//   for (size_t i = 0; i < valid_indices_in_pc->size(); ++i) {
//     int point_id = valid_indices_in_pc->at(i);
//     CHECK(unique_indices.find(point_id) == unique_indices.end());
//     unique_indices.insert(point_id);
//   }
//
//   for (size_t i = 0; i < pc_ptr->size(); ++i) {
//     const auto &point = pc_ptr->points[i];
//     // * the coordinates of x and y have been exchanged in feature generation
//     // step,
//     // so they should be swapped back here.
//     int col = F2I(point.y, range, inv_res_x);  // col
//     int row = F2I(point.x, range, inv_res_y);  // row
//     if (IsValidRowCol(row, rows, col, cols)) {
//       // get grid index and count point number for corresponding node
//       int grid = RowCol2Grid(row, col, cols);
//       view[grid].point_num++;
//       if (unique_indices.find(i) != unique_indices.end()) {
//         view[grid].valid_point_num++;
//       }
//     }
//   }
//
//   // show grids with grey color
//   for (int row = 0; row < rows; ++row) {
//     for (int col = 0; col < cols; ++col) {
//       int grid = RowCol2Grid(row, col, cols);
//       if (view[grid].valid_point_num > 0) {
//         img.at<cv::Vec3b>(row, col) = cv::Vec3b(127, 127, 127);
//       } else if (view[grid].point_num > 0) {
//         img.at<cv::Vec3b>(row, col) = cv::Vec3b(63, 63, 63);
//       }
//     }
//   }
//
//   // show segment grids with tight bounding box
//   const cv::Vec3b segm_color(0, 0, 255);  // red
//
//   for (size_t i = 0; i < objects.objects.size(); ++i) {
//     const ObjectPtr &obj = objects.objects.objects[i];
//     CHECK_GT(obj->cloud->size(), 0);
//
//     int x_min = INT_MAX;
//     int y_min = INT_MAX;
//     int x_max = INT_MIN;
//     int y_max = INT_MIN;
//     float score = obj->score;
//     CHECK_GE(score, 0.0);
//     CHECK_LE(score, 1.0);
//     for (size_t j = 0; j < obj->cloud->size(); ++j) {
//       const auto &point = obj->cloud->points[j];
//       int col = F2I(point.y, range, inv_res_x);  // col
//       int row = F2I(point.x, range, inv_res_y);  // row
//       CHECK(IsValidRowCol(row, rows, col, cols));
//       img.at<cv::Vec3b>(row, col) = segm_color * score;
//       x_min = std::min(col, x_min);
//       y_min = std::min(row, y_min);
//       x_max = std::max(col, x_max);
//       y_max = std::max(row, y_max);
//     }
//
//     // fillConvexPoly(img, list.data(), list.size(), cv::Scalar(positive_prob *
//     // segm_color));
//     cv::Vec3b bbox_color = GetTypeColor(obj->type);
//     rectangle(img, cv::Point(x_min, y_min), cv::Point(x_max, y_max),
//               cv::Scalar(bbox_color));
//   }
//
//   // write image intensity values into file
//   FILE *f_res;
//   f_res = fopen(result_file.c_str(), "w");
//   fprintf(f_res, "%d %d\n", rows, cols);
//   for (int row = 0; row < rows; ++row) {
//     for (int col = 0; col < cols; ++col) {
//       fprintf(f_res, "%u %u %u\n", img.at<cv::Vec3b>(row, col)[0],
//               img.at<cv::Vec3b>(row, col)[1], img.at<cv::Vec3b>(row, col)[2]);
//     }
//   }
//   fclose(f_res);
// }
