#include "cluster2d.h"

bool Cluster2D::init(int rows, int cols, float range)
{
  rows_ = rows;
  cols_ = cols;
  grids_ = rows_ * cols_;
  range_ = range;
  scale_ = 0.5 * static_cast<float>(rows_) / range_;
  inv_res_x_ = 0.5 * static_cast<float>(cols_) / range_;
  inv_res_y_ = 0.5 * static_cast<float>(rows_) / range_;
  point2grid_.clear();
  // obstacles_.clear();
  id_img_.assign(grids_, -1);
  pc_ptr_.reset();
  valid_indices_in_pc_ = nullptr;
  return true;
}

void Cluster2D::traverse(Node* x) {
    std::vector<Node*> p;
    p.clear();
    while (x->traversed == 0) {
      p.push_back(x);
      x->traversed = 2;
      x = x->center_node;
    }
    if (x->traversed == 2) {
      for (int i = static_cast<int>(p.size()) - 1; i >= 0 && p[i] != x; i--) {
        p[i]->is_center = true;
      }
      x->is_center = true;
    }
    for (size_t i = 0; i < p.size(); i++) {
      Node* y = p[i];
      y->traversed = 1;
      y->parent = x->parent;
    }
  }

void Cluster2D::cluster(const caffe::Blob<float>& category_pt_blob,
                        const caffe::Blob<float>& instance_pt_blob,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr,
                        const pcl::PointIndices& valid_indices,
                        float objectness_thresh, bool use_all_grids_for_clustering)
{
  const float* category_pt_data = category_pt_blob.cpu_data();
  const float* instance_pt_x_data = instance_pt_blob.cpu_data();
  const float* instance_pt_y_data =
      instance_pt_blob.cpu_data() + instance_pt_blob.offset(0, 1);

  pc_ptr_ = pc_ptr;
  std::vector<std::vector<Node>> nodes(rows_,
                                       std::vector<Node>(cols_, Node()));

  // map points into grids
  size_t tot_point_num = pc_ptr_->size();
  valid_indices_in_pc_ = &(valid_indices.indices);
  CHECK_LE(valid_indices_in_pc_->size(), tot_point_num);
  point2grid_.assign(valid_indices_in_pc_->size(), -1);

  for (size_t i = 0; i < valid_indices_in_pc_->size(); ++i) {
    int point_id = valid_indices_in_pc_->at(i);
    CHECK_GE(point_id, 0);
    CHECK_LT(point_id, static_cast<int>(tot_point_num));
    const auto& point = pc_ptr_->points[point_id];
    // * the coordinates of x and y have been exchanged in feature generation
    // step,
    // so we swap them back here.
    int pos_x = F2I(point.y, range_, inv_res_x_);  // col
    int pos_y = F2I(point.x, range_, inv_res_y_);  // row
    if (IsValidRowCol(pos_y, pos_x)) {
      // get grid index and count point number for corresponding node
      point2grid_[i] = RowCol2Grid(pos_y, pos_x);
      nodes[pos_y][pos_x].point_num++;
    }
  }

  // construct graph with center offset prediction and objectness
  for (int row = 0; row < rows_; ++row) {
    for (int col = 0; col < cols_; ++col) {
      int grid = RowCol2Grid(row, col);
      Node* node = &nodes[row][col];
      DisjointSetMakeSet(node);
      node->is_object =
          (use_all_grids_for_clustering || nodes[row][col].point_num > 0) &&
          (*(category_pt_data + grid) >= objectness_thresh);
      int center_row = std::round(row + instance_pt_x_data[grid] * scale_);
      int center_col = std::round(col + instance_pt_y_data[grid] * scale_);
      center_row = std::min(std::max(center_row, 0), rows_ - 1);
      center_col = std::min(std::max(center_col, 0), cols_ - 1);
      node->center_node = &nodes[center_row][center_col];
    }
  }

  // traverse nodes
  for (int row = 0; row < rows_; ++row) {
    for (int col = 0; col < cols_; ++col) {
      Node* node = &nodes[row][col];
      if (node->is_object && node->traversed == 0) {
        traverse(node);
      }
    }
  }
  for (int row = 0; row < rows_; ++row) {
    for (int col = 0; col < cols_; ++col) {
      Node* node = &nodes[row][col];
      if (!node->is_center) {
        continue;
      }
      for (int row2 = row - 1; row2 <= row + 1; ++row2) {
        for (int col2 = col - 1; col2 <= col + 1; ++col2) {
          if ((row2 == row || col2 == col) && IsValidRowCol(row2, col2)) {
            Node* node2 = &nodes[row2][col2];
            if (node2->is_center) {
              DisjointSetUnion(node, node2);
            }
          }
        }
      }
    }
  }

  int count_obstacles = 0;
  obstacles_.clear();
  id_img_.assign(grids_, -1);
  for (int row = 0; row < rows_; ++row) {
    for (int col = 0; col < cols_; ++col) {
      Node* node = &nodes[row][col];
      if (!node->is_object) {
        continue;
      }
      Node* root = DisjointSetFind(node);
      if (root->obstacle_id < 0) {
        root->obstacle_id = count_obstacles++;
        CHECK_EQ(static_cast<int>(obstacles_.size()), count_obstacles - 1);
        obstacles_.push_back(Obstacle());
      }
      int grid = RowCol2Grid(row, col);
      CHECK_GE(root->obstacle_id, 0);
      id_img_[grid] = root->obstacle_id;
      obstacles_[root->obstacle_id].grids.push_back(grid);
    }
  }
  CHECK_EQ(static_cast<size_t>(count_obstacles), obstacles_.size());
}

void Cluster2D::filter(const caffe::Blob<float>& confidence_pt_blob,
              const caffe::Blob<float>& height_pt_blob)
{
  const float* confidence_pt_data = confidence_pt_blob.cpu_data();
  const float* height_pt_data = height_pt_blob.cpu_data();
  for (size_t obstacle_id = 0; obstacle_id < obstacles_.size();
       obstacle_id++) {
    Obstacle* obs = &obstacles_[obstacle_id];
    CHECK_GT(obs->grids.size(), 0);
    double score = 0.0;
    double height = 0.0;
    for (int grid : obs->grids) {
      score += static_cast<double>(confidence_pt_data[grid]);
      height += static_cast<double>(height_pt_data[grid]);
    }
    obs->score = score / static_cast<double>(obs->grids.size());
    obs->height = height / static_cast<double>(obs->grids.size());
    obs->cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }
}

void Cluster2D::classify(const caffe::Blob<float>& classify_pt_blob)
{
  const float* classify_pt_data = classify_pt_blob.cpu_data();
  int num_classes = classify_pt_blob.channels();
  CHECK_EQ(num_classes, MAX_META_TYPE);
  for (size_t obs_id = 0; obs_id < obstacles_.size(); obs_id++) {
    Obstacle* obs = &obstacles_[obs_id];
    for (size_t grid_id = 0; grid_id < obs->grids.size(); grid_id++) {
      int grid = obs->grids[grid_id];
      for (int k = 0; k < num_classes; k++) {
        obs->meta_type_probs[k] += classify_pt_data[k * grids_ + grid];
      }
    }
    int meta_type_id = 0;
    for (int k = 0; k < num_classes; k++) {
      obs->meta_type_probs[k] /= obs->grids.size();
      if (obs->meta_type_probs[k] > obs->meta_type_probs[meta_type_id]) {
        meta_type_id = k;
      }
    }
    obs->meta_type = static_cast<MetaType>(meta_type_id);
  }
}

void Cluster2D::getObjects(const float confidence_thresh, const float height_thresh,
                  const int min_pts_num, autoware_msgs::DetectedObjectArray* objects)
{
  CHECK(valid_indices_in_pc_ != nullptr);

  for (size_t i = 0; i < point2grid_.size(); ++i) {
    int grid = point2grid_[i];
    if (grid < 0) {
      continue;
    }

    CHECK_GE(grid, 0);
    CHECK_LT(grid, grids_);
    int obstacle_id = id_img_[grid];

    int point_id = valid_indices_in_pc_->at(i);
    CHECK_GE(point_id, 0);
    CHECK_LT(point_id, static_cast<int>(pc_ptr_->size()));

    if (obstacle_id >= 0 &&
        obstacles_[obstacle_id].score >= confidence_thresh) {
      if (height_thresh < 0 ||
          pc_ptr_->points[point_id].z <=
              obstacles_[obstacle_id].height + height_thresh) {
        obstacles_[obstacle_id].cloud_ptr->push_back(pc_ptr_->points[point_id]);
      }
    }
  }

  for (size_t obstacle_id = 0; obstacle_id < obstacles_.size();
       obstacle_id++) {
    Obstacle* obs = &obstacles_[obstacle_id];
    if (static_cast<int>(obs->cloud_ptr->size()) < min_pts_num) {
      continue;
    }
    autoware_msgs::DetectedObject out_obj;
    sensor_msgs::PointCloud2 ros_pc;
    pcl::toROSMsg(*obs->cloud_ptr, ros_pc);
    out_obj.pointcloud = ros_pc;
    out_obj.score = obs->score;
    // out_obj->score_type = SCORE_CNN;
    // out_obj->type = GetObjectType(obs->meta_type);
    // out_obj->type_probs = GetObjectTypeProbs(obs->meta_type_probs);
    objects->objects.push_back(out_obj);
  }
}

ObjectType GetObjectType(const MetaType meta_type_id)
{
  switch (meta_type_id) {
    case META_UNKNOWN:
      return UNKNOWN;
    case META_SMALLMOT:
      return VEHICLE;
    case META_BIGMOT:
      return VEHICLE;
    case META_NONMOT:
      return BICYCLE;
    case META_PEDESTRIAN:
      return PEDESTRIAN;
    default: {
      return UNKNOWN;
    }
  }
}
