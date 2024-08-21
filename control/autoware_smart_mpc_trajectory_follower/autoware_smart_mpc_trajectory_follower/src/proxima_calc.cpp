// Copyright 2024 Proxima Technology Inc, TIER IV
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// cSpell:ignore lstm

#include <Eigen/Core>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <iostream>
namespace py = pybind11;

Eigen::VectorXd tanh(const Eigen::VectorXd & v)
{
  return v.array().tanh();
}
Eigen::VectorXd sigmoid(const Eigen::VectorXd & v)
{
  return 0.5 * (0.5 * v).array().tanh() + 0.5;
}
Eigen::VectorXd relu(const Eigen::VectorXd & x)
{
  Eigen::VectorXd x_ = x;
  for (int i = 0; i < x.size(); i++) {
    if (x[i] < 0) {
      x_[i] = 0;
    }
  }
  return x_;
}
Eigen::MatrixXd d_relu_product(const Eigen::MatrixXd & m, const Eigen::VectorXd & x)
{
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(m.rows(), m.cols());
  for (int i = 0; i < m.cols(); i++) {
    if (x[i] >= 0) {
      result.col(i) = m.col(i);
    }
  }
  return result;
}
Eigen::MatrixXd d_tanh_product(const Eigen::MatrixXd & m, const Eigen::VectorXd & x)
{
  Eigen::MatrixXd result = Eigen::MatrixXd(m.rows(), m.cols());
  for (int i = 0; i < m.cols(); i++) {
    result.col(i) = m.col(i) / (std::cosh(x[i]) * std::cosh(x[i]));
  }
  return result;
}
Eigen::VectorXd d_tanh_product_vec(const Eigen::VectorXd & v, const Eigen::VectorXd & x)
{
  Eigen::VectorXd result = Eigen::VectorXd(v.size());
  for (int i = 0; i < v.size(); i++) {
    result[i] = v[i] / (std::cosh(x[i]) * std::cosh(x[i]));
  }
  return result;
}
Eigen::MatrixXd d_sigmoid_product(const Eigen::MatrixXd & m, const Eigen::VectorXd & x)
{
  Eigen::MatrixXd result = Eigen::MatrixXd(m.rows(), m.cols());
  for (int i = 0; i < m.cols(); i++) {
    result.col(i) = 0.25 * m.col(i) / (std::cosh(0.5 * x[i]) * std::cosh(0.5 * x[i]));
  }
  return result;
}
Eigen::VectorXd d_sigmoid_product_vec(const Eigen::VectorXd & v, const Eigen::VectorXd & x)
{
  Eigen::VectorXd result = Eigen::VectorXd(v.size());
  for (int i = 0; i < v.size(); i++) {
    result[i] = 0.25 * v[i] / (std::cosh(0.5 * x[i]) * std::cosh(0.5 * x[i]));
  }
  return result;
}

Eigen::VectorXd get_polynomial_features(const Eigen::VectorXd & x, const int deg, const int dim)
{
  const int n_features = x.size();
  Eigen::VectorXd result = Eigen::VectorXd(dim);
  result.head(n_features) = x;
  if (deg >= 2) {
    std::vector<int> index = {};
    for (int feature_idx = 0; feature_idx < n_features + 1; feature_idx++) {
      index.push_back(feature_idx);
    }
    int current_idx = n_features;
    for (int i = 0; i < deg - 1; i++) {
      std::vector<int> new_index = {};
      const int end = index[index.size() - 1];
      for (int feature_idx = 0; feature_idx < n_features; feature_idx++) {
        const int start = index[feature_idx];
        new_index.push_back(current_idx);
        const int next_idx = current_idx + end - start;
        result.segment(current_idx, end - start) =
          x[feature_idx] * result.segment(start, end - start);
        current_idx = next_idx;
      }
      new_index.push_back(current_idx);
      index = new_index;
    }
  }
  return result;
}
Eigen::MatrixXd get_polynomial_features_with_diff(
  const Eigen::VectorXd & x, const int deg, const int dim)
{
  const int n_features = x.size();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(dim, n_features + 1);
  result.block(0, 0, n_features, 1) = x;
  result.block(0, 1, n_features, n_features) = Eigen::MatrixXd::Identity(n_features, n_features);
  if (deg >= 2) {
    std::vector<int> index = {};
    for (int feature_idx = 0; feature_idx < n_features + 1; feature_idx++) {
      index.push_back(feature_idx);
    }
    int current_idx = n_features;
    for (int i = 0; i < deg - 1; i++) {
      std::vector<int> new_index = {};
      const int end = index[index.size() - 1];
      for (int feature_idx = 0; feature_idx < n_features; feature_idx++) {
        const int start = index[feature_idx];
        new_index.push_back(current_idx);
        const int next_idx = current_idx + end - start;
        result.block(current_idx, 0, end - start, n_features + 1) =
          x[feature_idx] * result.block(start, 0, end - start, n_features + 1);
        result.block(current_idx, feature_idx + 1, end - start, 1) +=
          result.block(start, 0, end - start, 1);
        current_idx = next_idx;
      }
      new_index.push_back(current_idx);
      index = new_index;
    }
  }
  return result;
}
class transform_model_to_eigen
{
private:
  Eigen::MatrixXd weight_acc_layer_1_;
  Eigen::MatrixXd weight_steer_layer_1_head_;
  Eigen::MatrixXd weight_steer_layer_1_tail_;
  Eigen::MatrixXd weight_acc_layer_2_;
  Eigen::MatrixXd weight_steer_layer_2_;
  Eigen::MatrixXd weight_linear_relu_1_;
  Eigen::MatrixXd weight_linear_relu_2_;
  Eigen::MatrixXd weight_finalize_;
  Eigen::VectorXd bias_acc_layer_1_;
  Eigen::VectorXd bias_steer_layer_1_head_;
  Eigen::VectorXd bias_steer_layer_1_tail_;
  Eigen::VectorXd bias_acc_layer_2_;
  Eigen::VectorXd bias_steer_layer_2_;
  Eigen::VectorXd bias_linear_relu_1_;
  Eigen::VectorXd bias_linear_relu_2_;
  Eigen::VectorXd bias_linear_finalize_;
  Eigen::MatrixXd A_linear_reg_;
  Eigen::VectorXd b_linear_reg_;
  int deg_{};
  int acc_delay_step_{};
  int steer_delay_step_{};
  int acc_ctrl_queue_size_{};
  int steer_ctrl_queue_size_{};
  int steer_ctrl_queue_size_core_{};
  double vel_normalize_{};
  double acc_normalize_{};
  double steer_normalize_{};
  static constexpr double max_acc_error_ = 20.0;
  static constexpr double max_steer_error_ = 20.0;

public:
  transform_model_to_eigen() {}
  void set_params(
    const Eigen::MatrixXd & weight_acc_layer_1, const Eigen::MatrixXd & weight_steer_layer_1_head,
    const Eigen::MatrixXd & weight_steer_layer_1_tail, const Eigen::MatrixXd & weight_acc_layer_2,
    const Eigen::MatrixXd & weight_steer_layer_2, const Eigen::MatrixXd & weight_linear_relu_1,
    const Eigen::MatrixXd & weight_linear_relu_2, const Eigen::MatrixXd & weight_finalize,
    const Eigen::VectorXd & bias_acc_layer_1, const Eigen::VectorXd & bias_steer_layer_1_head,
    const Eigen::VectorXd & bias_steer_layer_1_tail, const Eigen::VectorXd & bias_acc_layer_2,
    const Eigen::VectorXd & bias_steer_layer_2, const Eigen::VectorXd & bias_linear_relu_1,
    const Eigen::VectorXd & bias_linear_relu_2, const Eigen::VectorXd & bias_linear_finalize,
    const Eigen::MatrixXd & A_linear_reg, const Eigen::VectorXd & b_linear_reg, const int deg,
    const int acc_delay_step, const int steer_delay_step, const int acc_ctrl_queue_size,
    const int steer_ctrl_queue_size, const int steer_ctrl_queue_size_core,
    const double vel_normalize, const double acc_normalize, const double steer_normalize)
  {
    weight_acc_layer_1_ = weight_acc_layer_1;
    weight_steer_layer_1_head_ = weight_steer_layer_1_head;
    weight_steer_layer_1_tail_ = weight_steer_layer_1_tail;
    weight_acc_layer_2_ = weight_acc_layer_2;
    weight_steer_layer_2_ = weight_steer_layer_2;
    weight_linear_relu_1_ = weight_linear_relu_1;
    weight_linear_relu_2_ = weight_linear_relu_2;
    weight_finalize_ = weight_finalize;
    bias_acc_layer_1_ = bias_acc_layer_1;
    bias_steer_layer_1_head_ = bias_steer_layer_1_head;
    bias_steer_layer_1_tail_ = bias_steer_layer_1_tail;
    bias_acc_layer_2_ = bias_acc_layer_2;
    bias_steer_layer_2_ = bias_steer_layer_2;
    bias_linear_relu_1_ = bias_linear_relu_1;
    bias_linear_relu_2_ = bias_linear_relu_2;
    bias_linear_finalize_ = bias_linear_finalize;
    A_linear_reg_ = A_linear_reg;
    b_linear_reg_ = b_linear_reg;
    deg_ = deg;
    acc_delay_step_ = acc_delay_step;
    steer_delay_step_ = steer_delay_step;
    acc_ctrl_queue_size_ = acc_ctrl_queue_size;
    steer_ctrl_queue_size_ = steer_ctrl_queue_size;
    steer_ctrl_queue_size_core_ = steer_ctrl_queue_size_core;
    vel_normalize_ = vel_normalize;
    acc_normalize_ = acc_normalize;
    steer_normalize_ = steer_normalize;
  }
  Eigen::VectorXd error_prediction(const Eigen::VectorXd & x) const
  {
    Eigen::VectorXd acc_sub(acc_ctrl_queue_size_ + 1);
    acc_sub[0] = acc_normalize_ * x[1];
    acc_sub.tail(acc_ctrl_queue_size_) = acc_normalize_ * x.segment(3, acc_ctrl_queue_size_);

    Eigen::VectorXd steer_sub(steer_ctrl_queue_size_core_ + 1);
    steer_sub[0] = steer_normalize_ * x[2];
    steer_sub.tail(steer_ctrl_queue_size_core_) =
      steer_normalize_ * x.segment(3 + acc_ctrl_queue_size_, steer_ctrl_queue_size_core_);
    const Eigen::VectorXd acc_layer_1 = relu(weight_acc_layer_1_ * acc_sub + bias_acc_layer_1_);

    Eigen::VectorXd steer_layer_1(
      bias_steer_layer_1_head_.size() + bias_steer_layer_1_tail_.size());
    steer_layer_1.head(bias_steer_layer_1_head_.size()) =
      relu(weight_steer_layer_1_head_ * steer_sub + bias_steer_layer_1_head_);

    const Eigen::VectorXd steer_input_full =
      steer_normalize_ * x.segment(3 + acc_ctrl_queue_size_, steer_ctrl_queue_size_);
    steer_layer_1.tail(bias_steer_layer_1_tail_.size()) =
      relu(weight_steer_layer_1_tail_ * steer_input_full + bias_steer_layer_1_tail_);

    const Eigen::VectorXd acc_layer_2 = relu(weight_acc_layer_2_ * acc_layer_1 + bias_acc_layer_2_);

    const Eigen::VectorXd steer_layer_2 =
      relu(weight_steer_layer_2_ * steer_layer_1 + bias_steer_layer_2_);

    Eigen::VectorXd h1(1 + acc_layer_2.size() + steer_layer_2.size());
    h1[0] = vel_normalize_ * x[0];
    h1.segment(1, acc_layer_2.size()) = acc_layer_2;
    h1.tail(steer_layer_2.size()) = steer_layer_2;
    const Eigen::VectorXd h2 = relu(weight_linear_relu_1_ * h1 + bias_linear_relu_1_);
    const Eigen::VectorXd h3 = relu(weight_linear_relu_2_ * h2 + bias_linear_relu_2_);
    Eigen::VectorXd h4(h3.size() + acc_layer_2.size() + steer_layer_2.size());
    h4.head(h3.size()) = h3;
    h4.segment(h3.size(), acc_layer_2.size()) = acc_layer_2;
    h4.tail(steer_layer_2.size()) = steer_layer_2;
    Eigen::VectorXd x_for_polynomial_reg(9);
    x_for_polynomial_reg.head(3) = x.head(3);
    const int acc_start = 3 + std::max(acc_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(3, 3) = x.segment(acc_start, 3);
    const int steer_start = 3 + acc_ctrl_queue_size_ + std::max(steer_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(6, 3) = x.segment(steer_start, 3);

    Eigen::VectorXd y =
      weight_finalize_ * h4 + bias_linear_finalize_ +
      A_linear_reg_ * get_polynomial_features(x_for_polynomial_reg, deg_, A_linear_reg_.cols()) +
      b_linear_reg_;
    y[4] = std::min(std::max(y[4], -max_acc_error_), max_acc_error_);
    y[5] = std::min(std::max(y[5], -max_steer_error_), max_steer_error_);
    return y;
  }
  Eigen::VectorXd rot_and_d_rot_error_prediction(const Eigen::VectorXd & x) const
  {
    const int x_dim = x.size();
    const double theta = x[3];
    const double v = x[2];
    double coef = 2.0 * std::abs(v);
    coef = coef * coef * coef * coef * coef * coef * coef;
    if (coef > 1.0) {
      coef = 1.0;
    }
    /*
    In previous implementations, the training model was unreliable in the low speed range
    because data in the low speed range was excluded from the training data
    in order to exclude data not under control from them.
    However, now the topic /system/operation_mode/state is used to identify
    whether the data is under control or not.
    Therefore, it may be safe to always set coef = 1, and this variable may be eliminated
    once it is confirmed safe to do so.
    */
    const double cos = std::cos(theta);
    const double sin = std::sin(theta);
    Eigen::Matrix2d Rot;
    Rot << cos, -sin, sin, cos;

    Eigen::Matrix2d dRot;
    dRot << -sin, -cos, cos, -sin;
    Eigen::VectorXd vars(x_dim - 3);
    vars[0] = x[2];
    vars[1] = x[4];
    vars[2] = x[5];
    vars.tail(x_dim - 6) = x.tail(x_dim - 6);
    const Eigen::VectorXd pred = error_prediction(vars);
    Eigen::VectorXd rot_and_d_rot_pred(8);
    rot_and_d_rot_pred.head(2) = Rot * pred.head(2);
    rot_and_d_rot_pred.segment(2, 4) = pred.segment(2, 4);
    rot_and_d_rot_pred.tail(2) = dRot * pred.head(2);

    return coef * rot_and_d_rot_pred;
  }
  Eigen::MatrixXd error_prediction_with_diff(const Eigen::VectorXd & x) const
  {
    Eigen::VectorXd acc_sub(acc_ctrl_queue_size_ + 1);
    acc_sub[0] = acc_normalize_ * x[1];
    acc_sub.tail(acc_ctrl_queue_size_) = acc_normalize_ * x.segment(3, acc_ctrl_queue_size_);

    Eigen::VectorXd steer_sub(steer_ctrl_queue_size_core_ + 1);
    steer_sub[0] = steer_normalize_ * x[2];
    steer_sub.tail(steer_ctrl_queue_size_core_) =
      steer_normalize_ * x.segment(3 + acc_ctrl_queue_size_, steer_ctrl_queue_size_core_);
    const Eigen::VectorXd steer_input_full =
      steer_normalize_ * x.segment(3 + acc_ctrl_queue_size_, steer_ctrl_queue_size_);

    const Eigen::VectorXd u_acc_layer_1 = weight_acc_layer_1_ * acc_sub + bias_acc_layer_1_;
    const Eigen::VectorXd acc_layer_1 = relu(u_acc_layer_1);

    Eigen::VectorXd u_steer_layer_1(
      bias_steer_layer_1_head_.size() + bias_steer_layer_1_tail_.size());
    u_steer_layer_1.head(bias_steer_layer_1_head_.size()) =
      weight_steer_layer_1_head_ * steer_sub + bias_steer_layer_1_head_;
    u_steer_layer_1.tail(bias_steer_layer_1_tail_.size()) =
      weight_steer_layer_1_tail_ * steer_input_full + bias_steer_layer_1_tail_;
    const Eigen::VectorXd steer_layer_1 = relu(u_steer_layer_1);

    const Eigen::VectorXd u_acc_layer_2 = weight_acc_layer_2_ * acc_layer_1 + bias_acc_layer_2_;
    const Eigen::VectorXd acc_layer_2 = relu(u_acc_layer_2);

    const Eigen::VectorXd u_steer_layer_2 =
      weight_steer_layer_2_ * steer_layer_1 + bias_steer_layer_2_;
    const Eigen::VectorXd steer_layer_2 = relu(u_steer_layer_2);

    Eigen::VectorXd h1(1 + acc_layer_2.size() + steer_layer_2.size());
    h1[0] = vel_normalize_ * x[0];
    h1.segment(1, acc_layer_2.size()) = acc_layer_2;
    h1.tail(steer_layer_2.size()) = steer_layer_2;
    const Eigen::VectorXd u2 = weight_linear_relu_1_ * h1 + bias_linear_relu_1_;
    const Eigen::VectorXd h2 = relu(u2);
    const Eigen::VectorXd u3 = weight_linear_relu_2_ * h2 + bias_linear_relu_2_;
    const Eigen::VectorXd h3 = relu(u3);
    Eigen::VectorXd h4(h3.size() + acc_layer_2.size() + steer_layer_2.size());
    h4.head(h3.size()) = h3;
    h4.segment(h3.size(), acc_layer_2.size()) = acc_layer_2;
    h4.tail(steer_layer_2.size()) = steer_layer_2;

    Eigen::VectorXd x_for_polynomial_reg(9);
    x_for_polynomial_reg.head(3) = x.head(3);
    const int acc_start = 3 + std::max(acc_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(3, 3) = x.segment(acc_start, 3);
    const int steer_start = 3 + acc_ctrl_queue_size_ + std::max(steer_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(6, 3) = x.segment(steer_start, 3);
    const Eigen::MatrixXd polynomial_features_with_diff =
      get_polynomial_features_with_diff(x_for_polynomial_reg, deg_, A_linear_reg_.cols());

    Eigen::VectorXd y =
      weight_finalize_ * h4 + bias_linear_finalize_ +
      A_linear_reg_ * polynomial_features_with_diff.block(0, 0, A_linear_reg_.cols(), 1) +
      b_linear_reg_;
    y[4] = std::min(std::max(y[4], -max_acc_error_), max_acc_error_);
    y[5] = std::min(std::max(y[5], -max_steer_error_), max_steer_error_);

    const Eigen::MatrixXd dy_dh3 = weight_finalize_.block(0, 0, y.size(), h3.size());
    const Eigen::MatrixXd dy_dh2 = d_relu_product(dy_dh3, u3) * weight_linear_relu_2_;
    const Eigen::MatrixXd dy_dh1 = d_relu_product(dy_dh2, u2) * weight_linear_relu_1_;

    const Eigen::MatrixXd dy_da2 =
      dy_dh1.block(0, 1, y.size(), acc_layer_2.size()) +
      weight_finalize_.block(0, h3.size(), y.size(), acc_layer_2.size());
    const Eigen::MatrixXd dy_ds2 =
      dy_dh1.block(0, 1 + acc_layer_2.size(), y.size(), steer_layer_2.size()) +
      weight_finalize_.block(0, h3.size() + acc_layer_2.size(), y.size(), steer_layer_2.size());
    const Eigen::MatrixXd dy_da1 = d_relu_product(dy_da2, u_acc_layer_2) * weight_acc_layer_2_;
    const Eigen::MatrixXd dy_ds1 = d_relu_product(dy_ds2, u_steer_layer_2) * weight_steer_layer_2_;

    const Eigen::MatrixXd dy_d_acc = d_relu_product(dy_da1, u_acc_layer_1) * weight_acc_layer_1_;
    Eigen::MatrixXd dy_d_steer = Eigen::MatrixXd::Zero(y.size(), steer_input_full.size() + 1);
    dy_d_steer.block(0, 1, y.size(), steer_input_full.size()) +=
      d_relu_product(
        dy_ds1.block(0, bias_steer_layer_1_head_.size(), y.size(), bias_steer_layer_1_tail_.size()),
        u_steer_layer_1.tail(bias_steer_layer_1_tail_.size())) *
      weight_steer_layer_1_tail_;
    dy_d_steer.block(0, 0, y.size(), steer_sub.size()) +=
      d_relu_product(
        dy_ds1.block(0, 0, y.size(), bias_steer_layer_1_head_.size()),
        u_steer_layer_1.head(bias_steer_layer_1_head_.size())) *
      weight_steer_layer_1_head_;

    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(y.size(), x.size() + 1);

    result.col(0) = y;

    result.col(1) = vel_normalize_ * dy_dh1.col(0);
    result.col(2) = acc_normalize_ * dy_d_acc.col(0);
    result.col(3) = steer_normalize_ * dy_d_steer.col(0);
    result.block(0, 4, y.size(), acc_ctrl_queue_size_) =
      acc_normalize_ * dy_d_acc.block(0, 1, y.size(), acc_ctrl_queue_size_);
    result.block(0, 4 + acc_ctrl_queue_size_, y.size(), steer_ctrl_queue_size_) =
      steer_normalize_ * dy_d_steer.block(0, 1, y.size(), steer_ctrl_queue_size_);

    const Eigen::MatrixXd polynomial_reg_diff =
      A_linear_reg_ *
      polynomial_features_with_diff.block(0, 1, A_linear_reg_.cols(), x_for_polynomial_reg.size());
    result.block(0, 1, y.size(), 3) += polynomial_reg_diff.block(0, 0, y.size(), 3);
    result.block(0, 1 + acc_start, y.size(), 3) += polynomial_reg_diff.block(0, 3, y.size(), 3);
    result.block(0, 1 + steer_start, y.size(), 3) += polynomial_reg_diff.block(0, 6, y.size(), 3);
    return result;
  }
  Eigen::MatrixXd rot_and_d_rot_error_prediction_with_diff(const Eigen::VectorXd & x) const
  {
    const int x_dim = x.size();
    const double theta = x[3];
    const double v = x[2];
    double coef = 2.0 * std::abs(v);
    coef = coef * coef * coef * coef * coef * coef * coef;
    if (coef > 1.0) {
      coef = 1.0;
    }
    const double cos = std::cos(theta);
    const double sin = std::sin(theta);
    Eigen::Matrix2d Rot;
    Rot << cos, -sin, sin, cos;

    Eigen::Matrix2d dRot;
    dRot << -sin, -cos, cos, -sin;
    Eigen::VectorXd vars(x_dim - 3);
    vars[0] = x[2];
    vars[1] = x[4];
    vars[2] = x[5];
    vars.tail(x_dim - 6) = x.tail(x_dim - 6);

    const Eigen::MatrixXd pred_d_pred = error_prediction_with_diff(vars);
    const Eigen::VectorXd pred = pred_d_pred.col(0);
    const Eigen::MatrixXd d_pred = pred_d_pred.block(0, 1, 6, x_dim - 3);
    Eigen::MatrixXd rot_and_d_rot_pred_with_diff = Eigen::MatrixXd::Zero(6, x_dim + 2);
    Eigen::MatrixXd rot_pred_with_diff(6, x_dim - 3);
    rot_pred_with_diff.block(0, 0, 2, x_dim - 3) = Rot * d_pred.block(0, 0, 2, x_dim - 3);
    rot_pred_with_diff.block(2, 0, 4, x_dim - 3) = d_pred.block(2, 0, 4, x_dim - 3);

    rot_and_d_rot_pred_with_diff.block(0, 0, 2, 1) = Rot * pred.head(2);
    rot_and_d_rot_pred_with_diff.block(2, 0, 4, 1) = pred.segment(2, 4);
    rot_and_d_rot_pred_with_diff.block(0, 1, 2, 1) = dRot * pred.head(2);
    rot_and_d_rot_pred_with_diff.col(2 + 2) = rot_pred_with_diff.col(0);
    rot_and_d_rot_pred_with_diff.col(2 + 4) = rot_pred_with_diff.col(1);
    rot_and_d_rot_pred_with_diff.col(2 + 5) = rot_pred_with_diff.col(2);
    rot_and_d_rot_pred_with_diff.block(0, 2 + 6, 6, x_dim - 6) =
      rot_pred_with_diff.block(0, 3, 6, x_dim - 6);
    return coef * rot_and_d_rot_pred_with_diff;
  }
  Eigen::MatrixXd rot_and_d_rot_error_prediction_with_poly_diff(const Eigen::VectorXd & x) const
  {
    const int x_dim = x.size();
    const double theta = x[3];
    const double v = x[2];
    double coef = 2.0 * std::abs(v);
    coef = coef * coef * coef * coef * coef * coef * coef;
    if (coef > 1.0) {
      coef = 1.0;
    }
    const double cos = std::cos(theta);
    const double sin = std::sin(theta);
    Eigen::Matrix2d Rot;
    Rot << cos, -sin, sin, cos;

    Eigen::Matrix2d dRot;
    dRot << -sin, -cos, cos, -sin;
    Eigen::VectorXd vars(x_dim - 3);
    vars[0] = x[2];
    vars[1] = x[4];
    vars[2] = x[5];
    vars.tail(x_dim - 6) = x.tail(x_dim - 6);

    const Eigen::VectorXd pred = error_prediction(vars);
    Eigen::MatrixXd d_pred = Eigen::MatrixXd::Zero(6, x_dim - 3);

    Eigen::VectorXd x_for_polynomial_reg(9);
    x_for_polynomial_reg.head(3) = x.head(3);
    const int acc_start = 3 + std::max(acc_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(3, 3) = x.segment(acc_start, 3);
    const int steer_start = 3 + acc_ctrl_queue_size_ + std::max(steer_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(6, 3) = x.segment(steer_start, 3);

    const Eigen::MatrixXd polynomial_features_with_diff =
      get_polynomial_features_with_diff(x_for_polynomial_reg, deg_, A_linear_reg_.cols());
    const Eigen::MatrixXd polynomial_reg_diff =
      A_linear_reg_ *
      polynomial_features_with_diff.block(0, 1, A_linear_reg_.cols(), x_for_polynomial_reg.size());
    d_pred.block(0, 0, 6, 3) += polynomial_reg_diff.block(0, 0, 6, 3);
    d_pred.block(0, 1 + acc_start, 6, 3) += polynomial_reg_diff.block(0, 3, 6, 3);
    d_pred.block(0, 1 + steer_start, 6, 3) += polynomial_reg_diff.block(0, 6, 6, 3);

    Eigen::MatrixXd rot_and_d_rot_pred_with_diff = Eigen::MatrixXd::Zero(6, x_dim + 2);
    Eigen::MatrixXd rot_pred_with_diff(6, x_dim - 3);
    rot_pred_with_diff.block(0, 0, 2, x_dim - 3) = Rot * d_pred.block(0, 0, 2, x_dim - 3);
    rot_pred_with_diff.block(2, 0, 4, x_dim - 3) = d_pred.block(2, 0, 4, x_dim - 3);

    rot_and_d_rot_pred_with_diff.block(0, 0, 2, 1) = Rot * pred.head(2);
    rot_and_d_rot_pred_with_diff.block(2, 0, 4, 1) = pred.segment(2, 4);
    rot_and_d_rot_pred_with_diff.block(0, 1, 2, 1) = dRot * pred.head(2);
    rot_and_d_rot_pred_with_diff.col(2 + 2) = rot_pred_with_diff.col(0);
    rot_and_d_rot_pred_with_diff.col(2 + 4) = rot_pred_with_diff.col(1);
    rot_and_d_rot_pred_with_diff.col(2 + 5) = rot_pred_with_diff.col(2);
    rot_and_d_rot_pred_with_diff.block(0, 2 + 6, 6, x_dim - 6) =
      rot_pred_with_diff.block(0, 3, 6, x_dim - 6);
    return coef * rot_and_d_rot_pred_with_diff;
  }
  Eigen::VectorXd rotated_error_prediction(const Eigen::VectorXd & x) const
  {
    const int x_dim = x.size();
    const double theta = x[3];
    const double v = x[2];
    double coef = 2.0 * std::abs(v);
    coef = coef * coef * coef * coef * coef * coef * coef;
    if (coef > 1.0) {
      coef = 1.0;
    }
    const double cos = std::cos(theta);
    const double sin = std::sin(theta);
    Eigen::Matrix2d Rot;
    Rot << cos, -sin, sin, cos;
    Eigen::VectorXd vars(x_dim - 3);
    vars[0] = x[2];
    vars[1] = x[4];
    vars[2] = x[5];
    vars.tail(x_dim - 6) = x.tail(x_dim - 6);
    const Eigen::VectorXd pred = error_prediction(vars);
    Eigen::VectorXd rot_pred(6);
    rot_pred.head(2) = coef * Rot * pred.head(2);
    rot_pred.tail(4) = coef * pred.tail(4);
    return rot_pred;
  }
  Eigen::MatrixXd Rotated_error_prediction(const Eigen::MatrixXd & X) const
  {
    const int X_cols = X.cols();
    const int x_dim = X.rows();
    Eigen::MatrixXd Pred(6, X_cols);
    for (int i = 0; i < X_cols; i++) {
      const Eigen::VectorXd x = X.col(i);
      const double theta = x[3];
      const double v = x[2];
      double coef = 2.0 * std::abs(v);
      coef = coef * coef * coef * coef * coef * coef * coef;
      if (coef > 1.0) {
        coef = 1.0;
      }
      const double cos = std::cos(theta);
      const double sin = std::sin(theta);
      Eigen::Matrix2d Rot;
      Rot << cos, -sin, sin, cos;
      Eigen::VectorXd vars(x_dim - 3);

      vars[0] = x[2];

      vars[1] = x[4];
      vars[2] = x[5];
      vars.tail(x_dim - 6) = x.tail(x_dim - 6);
      const Eigen::VectorXd pred = error_prediction(vars);
      Pred.block(0, i, 2, 1) = coef * Rot * pred.head(2);
      Pred.block(2, i, 4, 1) = coef * pred.tail(4);
    }
    return Pred;
  }
};
class transform_model_with_memory_to_eigen
{
private:
  Eigen::MatrixXd weight_acc_layer_1_;
  Eigen::MatrixXd weight_steer_layer_1_head_;
  Eigen::MatrixXd weight_steer_layer_1_tail_;
  Eigen::MatrixXd weight_acc_layer_2_;
  Eigen::MatrixXd weight_steer_layer_2_;
  Eigen::MatrixXd weight_lstm_ih_;
  Eigen::MatrixXd weight_lstm_hh_;
  Eigen::MatrixXd weight_linear_relu_1_;
  Eigen::MatrixXd weight_linear_relu_2_;
  Eigen::MatrixXd weight_finalize_;
  Eigen::VectorXd bias_acc_layer_1_;
  Eigen::VectorXd bias_steer_layer_1_head_;
  Eigen::VectorXd bias_steer_layer_1_tail_;
  Eigen::VectorXd bias_acc_layer_2_;
  Eigen::VectorXd bias_steer_layer_2_;
  Eigen::VectorXd bias_lstm_ih_;
  Eigen::VectorXd bias_lstm_hh_;
  Eigen::VectorXd bias_linear_relu_1_;
  Eigen::VectorXd bias_linear_relu_2_;
  Eigen::VectorXd bias_linear_finalize_;
  Eigen::MatrixXd A_linear_reg_;
  Eigen::VectorXd b_linear_reg_;
  int deg_{};
  int acc_delay_step_{};
  int steer_delay_step_{};
  int acc_ctrl_queue_size_{};
  int steer_ctrl_queue_size_{};
  int steer_ctrl_queue_size_core_{};
  double vel_normalize_{};
  double acc_normalize_{};
  double steer_normalize_{};

  static constexpr double max_acc_error_ = 20.0;
  static constexpr double max_steer_error_ = 20.0;
  Eigen::VectorXd h_, c_;
  Eigen::MatrixXd H_, C_;
  Eigen::MatrixXd dy_dhc_, dhc_dhc_, dhc_dx_;
  Eigen::MatrixXd dy_dhc_pre_, dhc_dx_pre_;

public:
  transform_model_with_memory_to_eigen() {}
  void set_params(
    const Eigen::MatrixXd & weight_acc_layer_1, const Eigen::MatrixXd & weight_steer_layer_1_head,
    const Eigen::MatrixXd & weight_steer_layer_1_tail, const Eigen::MatrixXd & weight_acc_layer_2,
    const Eigen::MatrixXd & weight_steer_layer_2, const Eigen::MatrixXd & weight_lstm_ih,
    const Eigen::MatrixXd & weight_lstm_hh, const Eigen::MatrixXd & weight_linear_relu_1,
    const Eigen::MatrixXd & weight_linear_relu_2, const Eigen::MatrixXd & weight_finalize,
    const Eigen::VectorXd & bias_acc_layer_1, const Eigen::VectorXd & bias_steer_layer_1_head,
    const Eigen::VectorXd & bias_steer_layer_1_tail, const Eigen::VectorXd & bias_acc_layer_2,
    const Eigen::VectorXd & bias_steer_layer_2, const Eigen::VectorXd & bias_lstm_ih,
    const Eigen::VectorXd & bias_lstm_hh, const Eigen::VectorXd & bias_linear_relu_1,
    const Eigen::VectorXd & bias_linear_relu_2, const Eigen::VectorXd & bias_linear_finalize)
  {
    weight_acc_layer_1_ = weight_acc_layer_1;
    weight_steer_layer_1_head_ = weight_steer_layer_1_head;
    weight_steer_layer_1_tail_ = weight_steer_layer_1_tail;
    weight_acc_layer_2_ = weight_acc_layer_2;
    weight_steer_layer_2_ = weight_steer_layer_2;
    weight_lstm_ih_ = weight_lstm_ih;
    weight_lstm_hh_ = weight_lstm_hh;
    weight_linear_relu_1_ = weight_linear_relu_1;
    weight_linear_relu_2_ = weight_linear_relu_2;
    weight_finalize_ = weight_finalize;
    bias_acc_layer_1_ = bias_acc_layer_1;
    bias_steer_layer_1_head_ = bias_steer_layer_1_head;
    bias_steer_layer_1_tail_ = bias_steer_layer_1_tail;
    bias_acc_layer_2_ = bias_acc_layer_2;
    bias_steer_layer_2_ = bias_steer_layer_2;
    bias_lstm_ih_ = bias_lstm_ih;
    bias_lstm_hh_ = bias_lstm_hh;
    bias_linear_relu_1_ = bias_linear_relu_1;
    bias_linear_relu_2_ = bias_linear_relu_2;
    bias_linear_finalize_ = bias_linear_finalize;
  }
  void set_params_res(
    const Eigen::MatrixXd & A_linear_reg, const Eigen::VectorXd & b_linear_reg, const int deg,
    const int acc_delay_step, const int steer_delay_step, const int acc_ctrl_queue_size,
    const int steer_ctrl_queue_size, const int steer_ctrl_queue_size_core,
    const double vel_normalize, const double acc_normalize, const double steer_normalize)
  {
    A_linear_reg_ = A_linear_reg;
    b_linear_reg_ = b_linear_reg;
    deg_ = deg;
    acc_delay_step_ = acc_delay_step;
    steer_delay_step_ = steer_delay_step;
    acc_ctrl_queue_size_ = acc_ctrl_queue_size;
    steer_ctrl_queue_size_ = steer_ctrl_queue_size;
    steer_ctrl_queue_size_core_ = steer_ctrl_queue_size_core;
    vel_normalize_ = vel_normalize;
    acc_normalize_ = acc_normalize;
    steer_normalize_ = steer_normalize;
    const int h_dim = weight_lstm_hh_.cols();
    h_ = Eigen::VectorXd::Zero(h_dim);
    c_ = Eigen::VectorXd::Zero(h_dim);
    dy_dhc_pre_ = Eigen::MatrixXd::Zero(6, 2 * h_dim);
    dhc_dx_pre_ =
      Eigen::MatrixXd::Zero(2 * h_dim, 3 + acc_ctrl_queue_size_ + steer_ctrl_queue_size_);
    dy_dhc_ = Eigen::MatrixXd::Zero(6, 2 * h_dim);
    dhc_dhc_ = Eigen::MatrixXd::Zero(2 * h_dim, 2 * h_dim);
    dhc_dx_ = Eigen::MatrixXd::Zero(2 * h_dim, 6 + acc_ctrl_queue_size_ + steer_ctrl_queue_size_);
  }
  void set_lstm(const Eigen::VectorXd & h, const Eigen::VectorXd & c)
  {
    h_ = h;
    c_ = c;
  }
  void set_lstm_for_candidate(
    const Eigen::VectorXd & h, const Eigen::VectorXd & c, const int sample_size)
  {
    H_ = Eigen::MatrixXd::Zero(h.size(), sample_size);
    C_ = Eigen::MatrixXd::Zero(c.size(), sample_size);
    for (int i = 0; i < sample_size; i++) {
      H_.col(i) = h;
      C_.col(i) = c;
    }
  }
  Eigen::VectorXd get_h() const { return h_; }
  Eigen::VectorXd get_c() const { return c_; }
  Eigen::MatrixXd get_dy_dhc() const { return dy_dhc_; }
  Eigen::MatrixXd get_dhc_dhc() const { return dhc_dhc_; }
  Eigen::MatrixXd get_dhc_dx() const { return dhc_dx_; }
  Eigen::VectorXd error_prediction(const Eigen::VectorXd & x, const int cell_index)
  {
    Eigen::VectorXd acc_sub(acc_ctrl_queue_size_ + 1);
    acc_sub[0] = acc_normalize_ * x[1];
    acc_sub.tail(acc_ctrl_queue_size_) = acc_normalize_ * x.segment(3, acc_ctrl_queue_size_);
    Eigen::VectorXd steer_sub(steer_ctrl_queue_size_core_ + 1);
    steer_sub[0] = steer_normalize_ * x[2];
    steer_sub.tail(steer_ctrl_queue_size_core_) =
      steer_normalize_ * x.segment(3 + acc_ctrl_queue_size_, steer_ctrl_queue_size_core_);
    const Eigen::VectorXd acc_layer_1 = relu(weight_acc_layer_1_ * acc_sub + bias_acc_layer_1_);
    Eigen::VectorXd steer_layer_1(
      bias_steer_layer_1_head_.size() + bias_steer_layer_1_tail_.size());
    steer_layer_1.head(bias_steer_layer_1_head_.size()) =
      relu(weight_steer_layer_1_head_ * steer_sub + bias_steer_layer_1_head_);

    const Eigen::VectorXd steer_input_full =
      steer_normalize_ * x.segment(3 + acc_ctrl_queue_size_, steer_ctrl_queue_size_);
    steer_layer_1.tail(bias_steer_layer_1_tail_.size()) =
      relu(weight_steer_layer_1_tail_ * steer_input_full + bias_steer_layer_1_tail_);

    const Eigen::VectorXd acc_layer_2 = relu(weight_acc_layer_2_ * acc_layer_1 + bias_acc_layer_2_);

    const Eigen::VectorXd steer_layer_2 =
      relu(weight_steer_layer_2_ * steer_layer_1 + bias_steer_layer_2_);
    Eigen::VectorXd h1(1 + acc_layer_2.size() + steer_layer_2.size());
    h1[0] = vel_normalize_ * x[0];
    h1.segment(1, acc_layer_2.size()) = acc_layer_2;
    h1.tail(steer_layer_2.size()) = steer_layer_2;
    Eigen::VectorXd h, c;
    if (cell_index < 0) {
      h = h_;
      c = c_;
    } else {
      h = H_.col(cell_index);
      c = C_.col(cell_index);
    }

    const Eigen::VectorXd i_new = sigmoid(
      weight_lstm_ih_.block(0, 0, h_.size(), h1.size()) * h1 + bias_lstm_ih_.head(h_.size()) +
      weight_lstm_hh_.block(0, 0, h_.size(), h_.size()) * h + bias_lstm_hh_.head(h_.size()));
    const Eigen::VectorXd f_new = sigmoid(
      weight_lstm_ih_.block(h_.size(), 0, h_.size(), h1.size()) * h1 +
      bias_lstm_ih_.segment(h_.size(), h_.size()) +
      weight_lstm_hh_.block(h_.size(), 0, h_.size(), h_.size()) * h +
      bias_lstm_hh_.segment(h_.size(), h_.size()));
    const Eigen::VectorXd g_new = tanh(
      weight_lstm_ih_.block(2 * h_.size(), 0, h_.size(), h1.size()) * h1 +
      bias_lstm_ih_.segment(2 * h_.size(), h_.size()) +
      weight_lstm_hh_.block(2 * h_.size(), 0, h_.size(), h_.size()) * h +
      bias_lstm_hh_.segment(2 * h_.size(), h_.size()));
    const Eigen::VectorXd o_new = sigmoid(
      weight_lstm_ih_.block(3 * h_.size(), 0, h_.size(), h1.size()) * h1 +
      bias_lstm_ih_.segment(3 * h_.size(), h_.size()) +
      weight_lstm_hh_.block(3 * h_.size(), 0, h_.size(), h_.size()) * h +
      bias_lstm_hh_.segment(3 * h_.size(), h_.size()));
    const Eigen::VectorXd c_new = f_new.array() * c.array() + i_new.array() * g_new.array();
    const Eigen::VectorXd h_new = o_new.array() * tanh(c_new).array();

    Eigen::VectorXd h2(h_new.size() + bias_linear_relu_1_.size());
    h2.head(h_new.size()) = h_new;
    h2.tail(bias_linear_relu_1_.size()) = relu(weight_linear_relu_1_ * h1 + bias_linear_relu_1_);

    const Eigen::VectorXd h3 = relu(weight_linear_relu_2_ * h2 + bias_linear_relu_2_);
    Eigen::VectorXd h4(h3.size() + acc_layer_2.size() + steer_layer_2.size());
    h4.head(h3.size()) = h3;
    h4.segment(h3.size(), acc_layer_2.size()) = acc_layer_2;
    h4.tail(steer_layer_2.size()) = steer_layer_2;

    Eigen::VectorXd x_for_polynomial_reg(9);
    x_for_polynomial_reg.head(3) = x.head(3);
    const int acc_start = 3 + std::max(acc_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(3, 3) = x.segment(acc_start, 3);
    const int steer_start = 3 + acc_ctrl_queue_size_ + std::max(steer_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(6, 3) = x.segment(steer_start, 3);

    Eigen::VectorXd y =
      weight_finalize_ * h4 + bias_linear_finalize_ +
      A_linear_reg_ * get_polynomial_features(x_for_polynomial_reg, deg_, A_linear_reg_.cols()) +
      b_linear_reg_;

    y[4] = std::min(std::max(y[4], -max_acc_error_), max_acc_error_);
    y[5] = std::min(std::max(y[5], -max_steer_error_), max_steer_error_);

    if (cell_index < 0) {
      h_ = h_new;
      c_ = c_new;
    } else {
      H_.col(cell_index) = h_new;
      C_.col(cell_index) = c_new;
    }
    return y;
  }

  Eigen::MatrixXd error_prediction_with_diff(const Eigen::VectorXd & x)
  {
    Eigen::VectorXd acc_sub(acc_ctrl_queue_size_ + 1);
    acc_sub[0] = acc_normalize_ * x[1];
    acc_sub.tail(acc_ctrl_queue_size_) = acc_normalize_ * x.segment(3, acc_ctrl_queue_size_);
    Eigen::VectorXd steer_sub(steer_ctrl_queue_size_core_ + 1);
    steer_sub[0] = steer_normalize_ * x[2];
    steer_sub.tail(steer_ctrl_queue_size_core_) =
      steer_normalize_ * x.segment(3 + acc_ctrl_queue_size_, steer_ctrl_queue_size_core_);
    const Eigen::VectorXd steer_input_full =
      steer_normalize_ * x.segment(3 + acc_ctrl_queue_size_, steer_ctrl_queue_size_);

    const Eigen::VectorXd u_acc_layer_1 = weight_acc_layer_1_ * acc_sub + bias_acc_layer_1_;
    const Eigen::VectorXd acc_layer_1 = relu(u_acc_layer_1);

    Eigen::VectorXd u_steer_layer_1(
      bias_steer_layer_1_head_.size() + bias_steer_layer_1_tail_.size());
    u_steer_layer_1.head(bias_steer_layer_1_head_.size()) =
      weight_steer_layer_1_head_ * steer_sub + bias_steer_layer_1_head_;
    u_steer_layer_1.tail(bias_steer_layer_1_tail_.size()) =
      weight_steer_layer_1_tail_ * steer_input_full + bias_steer_layer_1_tail_;
    const Eigen::VectorXd steer_layer_1 = relu(u_steer_layer_1);

    const Eigen::VectorXd u_acc_layer_2 = weight_acc_layer_2_ * acc_layer_1 + bias_acc_layer_2_;
    const Eigen::VectorXd acc_layer_2 = relu(u_acc_layer_2);

    const Eigen::VectorXd u_steer_layer_2 =
      weight_steer_layer_2_ * steer_layer_1 + bias_steer_layer_2_;
    const Eigen::VectorXd steer_layer_2 = relu(u_steer_layer_2);

    Eigen::VectorXd h1(1 + acc_layer_2.size() + steer_layer_2.size());
    h1[0] = vel_normalize_ * x[0];
    h1.segment(1, acc_layer_2.size()) = acc_layer_2;
    h1.tail(steer_layer_2.size()) = steer_layer_2;

    const Eigen::VectorXd u_i_new =
      weight_lstm_ih_.block(0, 0, h_.size(), h1.size()) * h1 + bias_lstm_ih_.head(h_.size()) +
      weight_lstm_hh_.block(0, 0, h_.size(), h_.size()) * h_ + bias_lstm_hh_.head(h_.size());
    const Eigen::VectorXd u_f_new = weight_lstm_ih_.block(h_.size(), 0, h_.size(), h1.size()) * h1 +
                                    bias_lstm_ih_.segment(h_.size(), h_.size()) +
                                    weight_lstm_hh_.block(h_.size(), 0, h_.size(), h_.size()) * h_ +
                                    bias_lstm_hh_.segment(h_.size(), h_.size());
    const Eigen::VectorXd u_g_new =
      weight_lstm_ih_.block(2 * h_.size(), 0, h_.size(), h1.size()) * h1 +
      bias_lstm_ih_.segment(2 * h_.size(), h_.size()) +
      weight_lstm_hh_.block(2 * h_.size(), 0, h_.size(), h_.size()) * h_ +
      bias_lstm_hh_.segment(2 * h_.size(), h_.size());
    const Eigen::VectorXd u_o_new =
      weight_lstm_ih_.block(3 * h_.size(), 0, h_.size(), h1.size()) * h1 +
      bias_lstm_ih_.segment(3 * h_.size(), h_.size()) +
      weight_lstm_hh_.block(3 * h_.size(), 0, h_.size(), h_.size()) * h_ +
      bias_lstm_hh_.segment(3 * h_.size(), h_.size());
    const Eigen::VectorXd i_new = sigmoid(u_i_new);
    const Eigen::VectorXd f_new = sigmoid(u_f_new);
    const Eigen::VectorXd g_new = tanh(u_g_new);
    const Eigen::VectorXd o_new = sigmoid(u_o_new);

    const Eigen::VectorXd c_new = f_new.array() * c_.array() + i_new.array() * g_new.array();
    const Eigen::VectorXd h_new = o_new.array() * tanh(c_new).array();

    Eigen::VectorXd h2(h_new.size() + bias_linear_relu_1_.size());
    h2.head(h_new.size()) = h_new;
    const Eigen::VectorXd u2 = weight_linear_relu_1_ * h1 + bias_linear_relu_1_;
    h2.tail(bias_linear_relu_1_.size()) = relu(u2);

    const Eigen::VectorXd u3 = weight_linear_relu_2_ * h2 + bias_linear_relu_2_;
    const Eigen::VectorXd h3 = relu(u3);
    Eigen::VectorXd h4(h3.size() + acc_layer_2.size() + steer_layer_2.size());

    h4.head(h3.size()) = h3;
    h4.segment(h3.size(), acc_layer_2.size()) = acc_layer_2;
    h4.tail(steer_layer_2.size()) = steer_layer_2;

    Eigen::VectorXd x_for_polynomial_reg(9);
    x_for_polynomial_reg.head(3) = x.head(3);
    const int acc_start = 3 + std::max(acc_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(3, 3) = x.segment(acc_start, 3);
    const int steer_start = 3 + acc_ctrl_queue_size_ + std::max(steer_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(6, 3) = x.segment(steer_start, 3);

    const Eigen::MatrixXd polynomial_features_with_diff =
      get_polynomial_features_with_diff(x_for_polynomial_reg, deg_, A_linear_reg_.cols());

    Eigen::VectorXd y =
      weight_finalize_ * h4 + bias_linear_finalize_ +
      A_linear_reg_ * polynomial_features_with_diff.block(0, 0, A_linear_reg_.cols(), 1) +
      b_linear_reg_;

    y[4] = std::min(std::max(y[4], -max_acc_error_), max_acc_error_);
    y[5] = std::min(std::max(y[5], -max_steer_error_), max_steer_error_);

    const Eigen::MatrixXd dy_dh3 = weight_finalize_.block(0, 0, y.size(), h3.size());
    const Eigen::MatrixXd dy_dh2 = d_relu_product(dy_dh3, u3) * weight_linear_relu_2_;
    const Eigen::MatrixXd dy_dh2_head = dy_dh2.block(0, 0, y.size(), h_new.size());
    const Eigen::MatrixXd dy_dh2_tail =
      dy_dh2.block(0, h_new.size(), y.size(), bias_linear_relu_1_.size());

    const Eigen::MatrixXd dy_do = dy_dh2_head * tanh(c_new).asDiagonal();
    const Eigen::MatrixXd dy_dc_new = d_tanh_product(dy_dh2_head * o_new.asDiagonal(), c_new);
    Eigen::MatrixXd dy_dh1 = d_sigmoid_product(dy_do, u_o_new) *
                             weight_lstm_ih_.block(3 * h_.size(), 0, h_.size(), h1.size());

    dy_dh1 += d_sigmoid_product(dy_dc_new * c_.asDiagonal(), u_f_new) *
              weight_lstm_ih_.block(h_.size(), 0, h_.size(), h1.size());
    dy_dh1 += d_tanh_product(dy_dc_new * i_new.asDiagonal(), u_g_new) *
              weight_lstm_ih_.block(2 * h_.size(), 0, h_.size(), h1.size());
    dy_dh1 += d_sigmoid_product(dy_dc_new * g_new.asDiagonal(), u_i_new) *
              weight_lstm_ih_.block(0, 0, h_.size(), h1.size());

    dy_dh1 += d_relu_product(dy_dh2_tail, u2) * weight_linear_relu_1_;

    const Eigen::MatrixXd dy_da2 =
      dy_dh1.block(0, 1, y.size(), acc_layer_2.size()) +
      weight_finalize_.block(0, h3.size(), y.size(), acc_layer_2.size());
    const Eigen::MatrixXd dy_ds2 =
      dy_dh1.block(0, 1 + acc_layer_2.size(), y.size(), steer_layer_2.size()) +
      weight_finalize_.block(0, h3.size() + acc_layer_2.size(), y.size(), steer_layer_2.size());
    const Eigen::MatrixXd dy_da1 = d_relu_product(dy_da2, u_acc_layer_2) * weight_acc_layer_2_;
    const Eigen::MatrixXd dy_ds1 = d_relu_product(dy_ds2, u_steer_layer_2) * weight_steer_layer_2_;

    const Eigen::MatrixXd dy_d_acc = d_relu_product(dy_da1, u_acc_layer_1) * weight_acc_layer_1_;
    Eigen::MatrixXd dy_d_steer = Eigen::MatrixXd::Zero(y.size(), steer_input_full.size() + 1);
    dy_d_steer.block(0, 1, y.size(), steer_input_full.size()) +=
      d_relu_product(
        dy_ds1.block(0, bias_steer_layer_1_head_.size(), y.size(), bias_steer_layer_1_tail_.size()),
        u_steer_layer_1.tail(bias_steer_layer_1_tail_.size())) *
      weight_steer_layer_1_tail_;
    dy_d_steer.block(0, 0, y.size(), steer_sub.size()) +=
      d_relu_product(
        dy_ds1.block(0, 0, y.size(), bias_steer_layer_1_head_.size()),
        u_steer_layer_1.head(bias_steer_layer_1_head_.size())) *
      weight_steer_layer_1_head_;

    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(y.size(), x.size() + 1);

    result.col(0) = y;

    result.col(1) = vel_normalize_ * dy_dh1.col(0);
    result.col(2) = acc_normalize_ * dy_d_acc.col(0);
    result.col(3) = steer_normalize_ * dy_d_steer.col(0);
    result.block(0, 4, y.size(), acc_ctrl_queue_size_) =
      acc_normalize_ * dy_d_acc.block(0, 1, y.size(), acc_ctrl_queue_size_);
    result.block(0, 4 + acc_ctrl_queue_size_, y.size(), steer_ctrl_queue_size_) =
      steer_normalize_ * dy_d_steer.block(0, 1, y.size(), steer_ctrl_queue_size_);

    const Eigen::MatrixXd polynomial_reg_diff =
      A_linear_reg_ *
      polynomial_features_with_diff.block(0, 1, A_linear_reg_.cols(), x_for_polynomial_reg.size());
    result.block(0, 1, y.size(), 3) += polynomial_reg_diff.block(0, 0, y.size(), 3);
    result.block(0, 1 + acc_start, y.size(), 3) += polynomial_reg_diff.block(0, 3, y.size(), 3);
    result.block(0, 1 + steer_start, y.size(), 3) += polynomial_reg_diff.block(0, 6, y.size(), 3);

    h_ = h_new;
    c_ = c_new;
    return result;
  }
  Eigen::MatrixXd error_prediction_with_memory_diff(const Eigen::VectorXd & x)
  {
    Eigen::VectorXd acc_sub(acc_ctrl_queue_size_ + 1);
    acc_sub[0] = acc_normalize_ * x[1];
    acc_sub.tail(acc_ctrl_queue_size_) = acc_normalize_ * x.segment(3, acc_ctrl_queue_size_);
    Eigen::VectorXd steer_sub(steer_ctrl_queue_size_core_ + 1);
    steer_sub[0] = steer_normalize_ * x[2];
    steer_sub.tail(steer_ctrl_queue_size_core_) =
      steer_normalize_ * x.segment(3 + acc_ctrl_queue_size_, steer_ctrl_queue_size_core_);
    const Eigen::VectorXd steer_input_full =
      steer_normalize_ * x.segment(3 + acc_ctrl_queue_size_, steer_ctrl_queue_size_);

    const Eigen::VectorXd u_acc_layer_1 = weight_acc_layer_1_ * acc_sub + bias_acc_layer_1_;
    const Eigen::VectorXd acc_layer_1 = relu(u_acc_layer_1);

    Eigen::VectorXd u_steer_layer_1(
      bias_steer_layer_1_head_.size() + bias_steer_layer_1_tail_.size());
    u_steer_layer_1.head(bias_steer_layer_1_head_.size()) =
      weight_steer_layer_1_head_ * steer_sub + bias_steer_layer_1_head_;
    u_steer_layer_1.tail(bias_steer_layer_1_tail_.size()) =
      weight_steer_layer_1_tail_ * steer_input_full + bias_steer_layer_1_tail_;
    const Eigen::VectorXd steer_layer_1 = relu(u_steer_layer_1);

    const Eigen::VectorXd u_acc_layer_2 = weight_acc_layer_2_ * acc_layer_1 + bias_acc_layer_2_;
    const Eigen::VectorXd acc_layer_2 = relu(u_acc_layer_2);

    const Eigen::VectorXd u_steer_layer_2 =
      weight_steer_layer_2_ * steer_layer_1 + bias_steer_layer_2_;
    const Eigen::VectorXd steer_layer_2 = relu(u_steer_layer_2);

    Eigen::VectorXd h1(1 + acc_layer_2.size() + steer_layer_2.size());
    h1[0] = vel_normalize_ * x[0];
    h1.segment(1, acc_layer_2.size()) = acc_layer_2;
    h1.tail(steer_layer_2.size()) = steer_layer_2;

    const Eigen::VectorXd u_i_new =
      weight_lstm_ih_.block(0, 0, h_.size(), h1.size()) * h1 + bias_lstm_ih_.head(h_.size()) +
      weight_lstm_hh_.block(0, 0, h_.size(), h_.size()) * h_ + bias_lstm_hh_.head(h_.size());
    const Eigen::VectorXd u_f_new = weight_lstm_ih_.block(h_.size(), 0, h_.size(), h1.size()) * h1 +
                                    bias_lstm_ih_.segment(h_.size(), h_.size()) +
                                    weight_lstm_hh_.block(h_.size(), 0, h_.size(), h_.size()) * h_ +
                                    bias_lstm_hh_.segment(h_.size(), h_.size());
    const Eigen::VectorXd u_g_new =
      weight_lstm_ih_.block(2 * h_.size(), 0, h_.size(), h1.size()) * h1 +
      bias_lstm_ih_.segment(2 * h_.size(), h_.size()) +
      weight_lstm_hh_.block(2 * h_.size(), 0, h_.size(), h_.size()) * h_ +
      bias_lstm_hh_.segment(2 * h_.size(), h_.size());
    const Eigen::VectorXd u_o_new =
      weight_lstm_ih_.block(3 * h_.size(), 0, h_.size(), h1.size()) * h1 +
      bias_lstm_ih_.segment(3 * h_.size(), h_.size()) +
      weight_lstm_hh_.block(3 * h_.size(), 0, h_.size(), h_.size()) * h_ +
      bias_lstm_hh_.segment(3 * h_.size(), h_.size());
    const Eigen::VectorXd i_new = sigmoid(u_i_new);
    const Eigen::VectorXd f_new = sigmoid(u_f_new);
    const Eigen::VectorXd g_new = tanh(u_g_new);
    const Eigen::VectorXd o_new = sigmoid(u_o_new);

    const Eigen::VectorXd c_new = f_new.array() * c_.array() + i_new.array() * g_new.array();
    const Eigen::VectorXd h_new = o_new.array() * tanh(c_new).array();

    Eigen::VectorXd h2(h_new.size() + bias_linear_relu_1_.size());
    h2.head(h_new.size()) = h_new;
    const Eigen::VectorXd u2 = weight_linear_relu_1_ * h1 + bias_linear_relu_1_;
    h2.tail(bias_linear_relu_1_.size()) = relu(u2);

    const Eigen::VectorXd u3 = weight_linear_relu_2_ * h2 + bias_linear_relu_2_;
    const Eigen::VectorXd h3 = relu(u3);
    Eigen::VectorXd h4(h3.size() + acc_layer_2.size() + steer_layer_2.size());

    h4.head(h3.size()) = h3;
    h4.segment(h3.size(), acc_layer_2.size()) = acc_layer_2;
    h4.tail(steer_layer_2.size()) = steer_layer_2;

    Eigen::VectorXd x_for_polynomial_reg(9);
    x_for_polynomial_reg.head(3) = x.head(3);
    const int acc_start = 3 + std::max(acc_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(3, 3) = x.segment(acc_start, 3);
    const int steer_start = 3 + acc_ctrl_queue_size_ + std::max(steer_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(6, 3) = x.segment(steer_start, 3);

    const Eigen::MatrixXd polynomial_features_with_diff =
      get_polynomial_features_with_diff(x_for_polynomial_reg, deg_, A_linear_reg_.cols());

    Eigen::VectorXd y =
      weight_finalize_ * h4 + bias_linear_finalize_ +
      A_linear_reg_ * polynomial_features_with_diff.block(0, 0, A_linear_reg_.cols(), 1) +
      b_linear_reg_;

    y[4] = std::min(std::max(y[4], -max_acc_error_), max_acc_error_);
    y[5] = std::min(std::max(y[5], -max_steer_error_), max_steer_error_);

    const Eigen::MatrixXd dy_dh3 = weight_finalize_.block(0, 0, y.size(), h3.size());
    const Eigen::MatrixXd dy_dh2 = d_relu_product(dy_dh3, u3) * weight_linear_relu_2_;
    const Eigen::MatrixXd dy_dh2_head = dy_dh2.block(0, 0, y.size(), h_new.size());
    const Eigen::MatrixXd dy_dh2_tail =
      dy_dh2.block(0, h_new.size(), y.size(), bias_linear_relu_1_.size());

    const Eigen::MatrixXd dy_do = dy_dh2_head * tanh(c_new).asDiagonal();
    const Eigen::MatrixXd dy_dc_new = d_tanh_product(dy_dh2_head * o_new.asDiagonal(), c_new);

    // calc dy_dhc_pre_, dhc_dhc_, dhc_dx_pre_
    const Eigen::VectorXd dc_du_f = d_sigmoid_product_vec(c_, u_f_new);
    const Eigen::VectorXd dc_du_g = d_tanh_product_vec(i_new, u_g_new);
    const Eigen::VectorXd dc_du_i = d_sigmoid_product_vec(g_new, u_i_new);
    const Eigen::VectorXd dh_dc_new = d_tanh_product_vec(o_new, c_new);
    const Eigen::VectorXd dh_du_o = d_sigmoid_product_vec(tanh(c_new), u_o_new);

    const Eigen::MatrixXd dc_dc = f_new.asDiagonal();
    const Eigen::MatrixXd dy_dc = dy_dc_new * dc_dc;

    Eigen::MatrixXd dc_dh =
      dc_du_f.asDiagonal() * weight_lstm_hh_.block(h_.size(), 0, h_.size(), h_.size());
    dc_dh += dc_du_g.asDiagonal() * weight_lstm_hh_.block(2 * h_.size(), 0, h_.size(), h_.size());
    dc_dh += dc_du_i.asDiagonal() * weight_lstm_hh_.block(0, 0, h_.size(), h_.size());
    const Eigen::VectorXd dh_dc = dh_dc_new.array() * f_new.array();

    Eigen::MatrixXd dh_dh =
      dh_du_o.asDiagonal() * weight_lstm_hh_.block(3 * h_.size(), 0, h_.size(), h_.size());

    dh_dh += dh_dc_new.asDiagonal() * dc_dh;

    const Eigen::MatrixXd dy_dh = dy_dh2_head * dh_dh;
    Eigen::MatrixXd dc_dh1 =
      dc_du_f.asDiagonal() * weight_lstm_ih_.block(h_.size(), 0, h_.size(), h1.size());
    dc_dh1 += dc_du_g.asDiagonal() * weight_lstm_ih_.block(2 * h_.size(), 0, h_.size(), h1.size());
    dc_dh1 += dc_du_i.asDiagonal() * weight_lstm_ih_.block(0, 0, h_.size(), h1.size());

    Eigen::MatrixXd dh_dh1 =
      dh_du_o.asDiagonal() * weight_lstm_ih_.block(3 * h_.size(), 0, h_.size(), h1.size());
    dh_dh1 += dh_dc_new.asDiagonal() * dc_dh1;

    const Eigen::MatrixXd dc_da2 = dc_dh1.block(0, 1, h_.size(), acc_layer_2.size());
    const Eigen::MatrixXd dc_ds2 =
      dc_dh1.block(0, 1 + acc_layer_2.size(), h_.size(), steer_layer_2.size());

    const Eigen::MatrixXd dh_da2 = dh_dh1.block(0, 1, h_.size(), acc_layer_2.size());
    const Eigen::MatrixXd dh_ds2 =
      dh_dh1.block(0, 1 + acc_layer_2.size(), h_.size(), steer_layer_2.size());

    const Eigen::MatrixXd dc_da1 = d_relu_product(dc_da2, u_acc_layer_2) * weight_acc_layer_2_;
    const Eigen::MatrixXd dc_ds1 = d_relu_product(dc_ds2, u_steer_layer_2) * weight_steer_layer_2_;

    const Eigen::MatrixXd dh_da1 = d_relu_product(dh_da2, u_acc_layer_2) * weight_acc_layer_2_;
    const Eigen::MatrixXd dh_ds1 = d_relu_product(dh_ds2, u_steer_layer_2) * weight_steer_layer_2_;

    const Eigen::MatrixXd dc_d_acc = d_relu_product(dc_da1, u_acc_layer_1) * weight_acc_layer_1_;

    Eigen::MatrixXd dc_d_steer = Eigen::MatrixXd::Zero(h_.size(), steer_input_full.size() + 1);
    dc_d_steer.block(0, 1, h_.size(), steer_input_full.size()) +=
      d_relu_product(
        dc_ds1.block(
          0, bias_steer_layer_1_head_.size(), h_.size(), bias_steer_layer_1_tail_.size()),
        u_steer_layer_1.tail(bias_steer_layer_1_tail_.size())) *
      weight_steer_layer_1_tail_;
    dc_d_steer.block(0, 0, h_.size(), steer_sub.size()) +=
      d_relu_product(
        dc_ds1.block(0, 0, h_.size(), bias_steer_layer_1_head_.size()),
        u_steer_layer_1.head(bias_steer_layer_1_head_.size())) *
      weight_steer_layer_1_head_;

    const Eigen::MatrixXd dh_d_acc = d_relu_product(dh_da1, u_acc_layer_1) * weight_acc_layer_1_;

    Eigen::MatrixXd dh_d_steer = Eigen::MatrixXd::Zero(h_.size(), steer_input_full.size() + 1);
    dh_d_steer.block(0, 1, h_.size(), steer_input_full.size()) +=
      d_relu_product(
        dh_ds1.block(
          0, bias_steer_layer_1_head_.size(), h_.size(), bias_steer_layer_1_tail_.size()),
        u_steer_layer_1.tail(bias_steer_layer_1_tail_.size())) *
      weight_steer_layer_1_tail_;
    dh_d_steer.block(0, 0, h_.size(), steer_sub.size()) +=
      d_relu_product(
        dh_ds1.block(0, 0, h_.size(), bias_steer_layer_1_head_.size()),
        u_steer_layer_1.head(bias_steer_layer_1_head_.size())) *
      weight_steer_layer_1_head_;

    Eigen::MatrixXd dc_dx = Eigen::MatrixXd(h_.size(), x.size());
    Eigen::MatrixXd dh_dx = Eigen::MatrixXd(h_.size(), x.size());
    dc_dx.col(0) = vel_normalize_ * dc_dh1.col(0);
    dc_dx.col(1) = acc_normalize_ * dc_d_acc.col(0);
    dc_dx.col(2) = steer_normalize_ * dc_d_steer.col(0);
    dc_dx.block(0, 3, h_.size(), acc_ctrl_queue_size_) =
      acc_normalize_ * dc_d_acc.block(0, 1, h_.size(), acc_ctrl_queue_size_);
    dc_dx.block(0, 3 + acc_ctrl_queue_size_, h_.size(), steer_ctrl_queue_size_) =
      steer_normalize_ * dc_d_steer.block(0, 1, h_.size(), steer_ctrl_queue_size_);

    dh_dx.col(0) = vel_normalize_ * dh_dh1.col(0);
    dh_dx.col(1) = acc_normalize_ * dh_d_acc.col(0);
    dh_dx.col(2) = steer_normalize_ * dh_d_steer.col(0);
    dh_dx.block(0, 3, h_.size(), acc_ctrl_queue_size_) =
      acc_normalize_ * dh_d_acc.block(0, 1, h_.size(), acc_ctrl_queue_size_);
    dh_dx.block(0, 3 + acc_ctrl_queue_size_, h_.size(), steer_ctrl_queue_size_) =
      steer_normalize_ * dh_d_steer.block(0, 1, h_.size(), steer_ctrl_queue_size_);

    dy_dhc_pre_.block(0, 0, y.size(), h_.size()) = dy_dh;
    dy_dhc_pre_.block(0, h_.size(), y.size(), h_.size()) = dy_dc;
    dhc_dhc_.block(0, 0, h_.size(), h_.size()) = dh_dh;
    dhc_dhc_.block(h_.size(), 0, h_.size(), h_.size()) = dc_dh;
    dhc_dhc_.block(0, h_.size(), h_.size(), h_.size()) = dh_dc.asDiagonal();
    dhc_dhc_.block(h_.size(), h_.size(), h_.size(), h_.size()) = dc_dc;
    dhc_dx_pre_.block(0, 0, h_.size(), x.size()) = dh_dx;
    dhc_dx_pre_.block(h_.size(), 0, h_.size(), x.size()) = dc_dx;

    // finished calc dy_dhc_pre_, dhc_dhc_, dhc_dx_pre_

    Eigen::MatrixXd dy_dh1 = d_sigmoid_product(dy_do, u_o_new) *
                             weight_lstm_ih_.block(3 * h_.size(), 0, h_.size(), h1.size());

    dy_dh1 += d_sigmoid_product(dy_dc_new * c_.asDiagonal(), u_f_new) *
              weight_lstm_ih_.block(h_.size(), 0, h_.size(), h1.size());
    dy_dh1 += d_tanh_product(dy_dc_new * i_new.asDiagonal(), u_g_new) *
              weight_lstm_ih_.block(2 * h_.size(), 0, h_.size(), h1.size());
    dy_dh1 += d_sigmoid_product(dy_dc_new * g_new.asDiagonal(), u_i_new) *
              weight_lstm_ih_.block(0, 0, h_.size(), h1.size());

    dy_dh1 += d_relu_product(dy_dh2_tail, u2) * weight_linear_relu_1_;

    const Eigen::MatrixXd dy_da2 =
      dy_dh1.block(0, 1, y.size(), acc_layer_2.size()) +
      weight_finalize_.block(0, h3.size(), y.size(), acc_layer_2.size());
    const Eigen::MatrixXd dy_ds2 =
      dy_dh1.block(0, 1 + acc_layer_2.size(), y.size(), steer_layer_2.size()) +
      weight_finalize_.block(0, h3.size() + acc_layer_2.size(), y.size(), steer_layer_2.size());
    const Eigen::MatrixXd dy_da1 = d_relu_product(dy_da2, u_acc_layer_2) * weight_acc_layer_2_;
    const Eigen::MatrixXd dy_ds1 = d_relu_product(dy_ds2, u_steer_layer_2) * weight_steer_layer_2_;

    const Eigen::MatrixXd dy_d_acc = d_relu_product(dy_da1, u_acc_layer_1) * weight_acc_layer_1_;
    Eigen::MatrixXd dy_d_steer = Eigen::MatrixXd::Zero(y.size(), steer_input_full.size() + 1);
    dy_d_steer.block(0, 1, y.size(), steer_input_full.size()) +=
      d_relu_product(
        dy_ds1.block(0, bias_steer_layer_1_head_.size(), y.size(), bias_steer_layer_1_tail_.size()),
        u_steer_layer_1.tail(bias_steer_layer_1_tail_.size())) *
      weight_steer_layer_1_tail_;
    dy_d_steer.block(0, 0, y.size(), steer_sub.size()) +=
      d_relu_product(
        dy_ds1.block(0, 0, y.size(), bias_steer_layer_1_head_.size()),
        u_steer_layer_1.head(bias_steer_layer_1_head_.size())) *
      weight_steer_layer_1_head_;

    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(y.size(), x.size() + 1);

    result.col(0) = y;

    result.col(1) = vel_normalize_ * dy_dh1.col(0);
    result.col(2) = acc_normalize_ * dy_d_acc.col(0);
    result.col(3) = steer_normalize_ * dy_d_steer.col(0);
    result.block(0, 4, y.size(), acc_ctrl_queue_size_) =
      acc_normalize_ * dy_d_acc.block(0, 1, y.size(), acc_ctrl_queue_size_);
    result.block(0, 4 + acc_ctrl_queue_size_, y.size(), steer_ctrl_queue_size_) =
      steer_normalize_ * dy_d_steer.block(0, 1, y.size(), steer_ctrl_queue_size_);

    const Eigen::MatrixXd polynomial_reg_diff =
      A_linear_reg_ *
      polynomial_features_with_diff.block(0, 1, A_linear_reg_.cols(), x_for_polynomial_reg.size());
    result.block(0, 1, y.size(), 3) += polynomial_reg_diff.block(0, 0, y.size(), 3);
    result.block(0, 1 + acc_start, y.size(), 3) += polynomial_reg_diff.block(0, 3, y.size(), 3);
    result.block(0, 1 + steer_start, y.size(), 3) += polynomial_reg_diff.block(0, 6, y.size(), 3);

    h_ = h_new;
    c_ = c_new;
    return result;
  }

  void update_memory(const Eigen::VectorXd & x)
  {
    Eigen::VectorXd acc_sub(acc_ctrl_queue_size_ + 1);
    acc_sub[0] = acc_normalize_ * x[1];
    acc_sub.tail(acc_ctrl_queue_size_) = acc_normalize_ * x.segment(3, acc_ctrl_queue_size_);
    Eigen::VectorXd steer_sub(steer_ctrl_queue_size_core_ + 1);
    steer_sub[0] = steer_normalize_ * x[2];
    steer_sub.tail(steer_ctrl_queue_size_core_) =
      steer_normalize_ * x.segment(3 + acc_ctrl_queue_size_, steer_ctrl_queue_size_core_);
    const Eigen::VectorXd acc_layer_1 = relu(weight_acc_layer_1_ * acc_sub + bias_acc_layer_1_);
    Eigen::VectorXd steer_layer_1(
      bias_steer_layer_1_head_.size() + bias_steer_layer_1_tail_.size());
    steer_layer_1.head(bias_steer_layer_1_head_.size()) =
      relu(weight_steer_layer_1_head_ * steer_sub + bias_steer_layer_1_head_);

    const Eigen::VectorXd steer_input_full =
      steer_normalize_ * x.segment(3 + acc_ctrl_queue_size_, steer_ctrl_queue_size_);
    steer_layer_1.tail(bias_steer_layer_1_tail_.size()) =
      relu(weight_steer_layer_1_tail_ * steer_input_full + bias_steer_layer_1_tail_);

    const Eigen::VectorXd acc_layer_2 = relu(weight_acc_layer_2_ * acc_layer_1 + bias_acc_layer_2_);

    const Eigen::VectorXd steer_layer_2 =
      relu(weight_steer_layer_2_ * steer_layer_1 + bias_steer_layer_2_);
    Eigen::VectorXd h1(1 + acc_layer_2.size() + steer_layer_2.size());
    h1[0] = vel_normalize_ * x[0];
    h1.segment(1, acc_layer_2.size()) = acc_layer_2;
    h1.tail(steer_layer_2.size()) = steer_layer_2;

    const Eigen::VectorXd i_new = sigmoid(
      weight_lstm_ih_.block(0, 0, h_.size(), h1.size()) * h1 + bias_lstm_ih_.head(h_.size()) +
      weight_lstm_hh_.block(0, 0, h_.size(), h_.size()) * h_ + bias_lstm_hh_.head(h_.size()));
    const Eigen::VectorXd f_new = sigmoid(
      weight_lstm_ih_.block(h_.size(), 0, h_.size(), h1.size()) * h1 +
      bias_lstm_ih_.segment(h_.size(), h_.size()) +
      weight_lstm_hh_.block(h_.size(), 0, h_.size(), h_.size()) * h_ +
      bias_lstm_hh_.segment(h_.size(), h_.size()));
    const Eigen::VectorXd g_new = tanh(
      weight_lstm_ih_.block(2 * h_.size(), 0, h_.size(), h1.size()) * h1 +
      bias_lstm_ih_.segment(2 * h_.size(), h_.size()) +
      weight_lstm_hh_.block(2 * h_.size(), 0, h_.size(), h_.size()) * h_ +
      bias_lstm_hh_.segment(2 * h_.size(), h_.size()));
    const Eigen::VectorXd o_new = sigmoid(
      weight_lstm_ih_.block(3 * h_.size(), 0, h_.size(), h1.size()) * h1 +
      bias_lstm_ih_.segment(3 * h_.size(), h_.size()) +
      weight_lstm_hh_.block(3 * h_.size(), 0, h_.size(), h_.size()) * h_ +
      bias_lstm_hh_.segment(3 * h_.size(), h_.size()));
    const Eigen::VectorXd c_new = f_new.array() * c_.array() + i_new.array() * g_new.array();
    const Eigen::VectorXd h_new = o_new.array() * tanh(c_new).array();
    h_ = h_new;
    c_ = c_new;
  }
  Eigen::VectorXd rot_and_d_rot_error_prediction(const Eigen::VectorXd & x)
  {
    const int x_dim = x.size();
    const double theta = x[3];
    const double v = x[2];
    double coef = 2.0 * std::abs(v);
    coef = coef * coef * coef * coef * coef * coef * coef;
    if (coef > 1.0) {
      coef = 1.0;
    }
    const double cos = std::cos(theta);
    const double sin = std::sin(theta);
    Eigen::Matrix2d Rot;
    Rot << cos, -sin, sin, cos;

    Eigen::Matrix2d dRot;
    dRot << -sin, -cos, cos, -sin;
    Eigen::VectorXd vars(x_dim - 3);
    vars[0] = x[2];
    vars[1] = x[4];
    vars[2] = x[5];
    vars.tail(x_dim - 6) = x.tail(x_dim - 6);
    const Eigen::VectorXd pred = error_prediction(vars, -1);
    Eigen::VectorXd rot_and_d_rot_pred(8);
    rot_and_d_rot_pred.head(2) = Rot * pred.head(2);
    rot_and_d_rot_pred.segment(2, 4) = pred.segment(2, 4);
    rot_and_d_rot_pred.tail(2) = dRot * pred.head(2);

    return coef * rot_and_d_rot_pred;
  }
  Eigen::MatrixXd rot_and_d_rot_error_prediction_with_diff(const Eigen::VectorXd & x)
  {
    const int x_dim = x.size();
    const double theta = x[3];
    const double v = x[2];
    double coef = 2.0 * std::abs(v);
    coef = coef * coef * coef * coef * coef * coef * coef;
    if (coef > 1.0) {
      coef = 1.0;
    }
    const double cos = std::cos(theta);
    const double sin = std::sin(theta);
    Eigen::Matrix2d Rot;
    Rot << cos, -sin, sin, cos;

    Eigen::Matrix2d dRot;
    dRot << -sin, -cos, cos, -sin;
    Eigen::VectorXd vars(x_dim - 3);
    vars[0] = x[2];
    vars[1] = x[4];
    vars[2] = x[5];
    vars.tail(x_dim - 6) = x.tail(x_dim - 6);

    const Eigen::MatrixXd pred_d_pred = error_prediction_with_diff(vars);
    const Eigen::VectorXd pred = pred_d_pred.col(0);
    const Eigen::MatrixXd d_pred = pred_d_pred.block(0, 1, 6, x_dim - 3);
    Eigen::MatrixXd rot_and_d_rot_pred_with_diff = Eigen::MatrixXd::Zero(6, x_dim + 2);
    Eigen::MatrixXd rot_pred_with_diff(6, x_dim - 3);
    rot_pred_with_diff.block(0, 0, 2, x_dim - 3) = Rot * d_pred.block(0, 0, 2, x_dim - 3);
    rot_pred_with_diff.block(2, 0, 4, x_dim - 3) = d_pred.block(2, 0, 4, x_dim - 3);

    rot_and_d_rot_pred_with_diff.block(0, 0, 2, 1) = Rot * pred.head(2);
    rot_and_d_rot_pred_with_diff.block(2, 0, 4, 1) = pred.segment(2, 4);
    rot_and_d_rot_pred_with_diff.block(0, 1, 2, 1) = dRot * pred.head(2);
    rot_and_d_rot_pred_with_diff.col(2 + 2) = rot_pred_with_diff.col(0);
    rot_and_d_rot_pred_with_diff.col(2 + 4) = rot_pred_with_diff.col(1);
    rot_and_d_rot_pred_with_diff.col(2 + 5) = rot_pred_with_diff.col(2);
    rot_and_d_rot_pred_with_diff.block(0, 2 + 6, 6, x_dim - 6) =
      rot_pred_with_diff.block(0, 3, 6, x_dim - 6);

    return coef * rot_and_d_rot_pred_with_diff;
  }
  Eigen::MatrixXd rot_and_d_rot_error_prediction_with_memory_diff(const Eigen::VectorXd & x)
  {
    const int x_dim = x.size();
    const double theta = x[3];
    const double v = x[2];
    double coef = 2.0 * std::abs(v);
    coef = coef * coef * coef * coef * coef * coef * coef;
    if (coef > 1.0) {
      coef = 1.0;
    }
    const double cos = std::cos(theta);
    const double sin = std::sin(theta);
    Eigen::Matrix2d Rot;
    Rot << cos, -sin, sin, cos;

    Eigen::Matrix2d dRot;
    dRot << -sin, -cos, cos, -sin;
    Eigen::VectorXd vars(x_dim - 3);
    vars[0] = x[2];
    vars[1] = x[4];
    vars[2] = x[5];
    vars.tail(x_dim - 6) = x.tail(x_dim - 6);

    const Eigen::MatrixXd pred_d_pred = error_prediction_with_memory_diff(vars);
    const Eigen::VectorXd pred = pred_d_pred.col(0);
    const Eigen::MatrixXd d_pred = pred_d_pred.block(0, 1, 6, x_dim - 3);
    Eigen::MatrixXd rot_and_d_rot_pred_with_diff = Eigen::MatrixXd::Zero(6, x_dim + 2);
    Eigen::MatrixXd rot_pred_with_diff(6, x_dim - 3);
    rot_pred_with_diff.block(0, 0, 2, x_dim - 3) = Rot * d_pred.block(0, 0, 2, x_dim - 3);
    rot_pred_with_diff.block(2, 0, 4, x_dim - 3) = d_pred.block(2, 0, 4, x_dim - 3);

    rot_and_d_rot_pred_with_diff.block(0, 0, 2, 1) = Rot * pred.head(2);
    rot_and_d_rot_pred_with_diff.block(2, 0, 4, 1) = pred.segment(2, 4);
    rot_and_d_rot_pred_with_diff.block(0, 1, 2, 1) = dRot * pred.head(2);
    rot_and_d_rot_pred_with_diff.col(2 + 2) = rot_pred_with_diff.col(0);
    rot_and_d_rot_pred_with_diff.col(2 + 4) = rot_pred_with_diff.col(1);
    rot_and_d_rot_pred_with_diff.col(2 + 5) = rot_pred_with_diff.col(2);
    rot_and_d_rot_pred_with_diff.block(0, 2 + 6, 6, x_dim - 6) =
      rot_pred_with_diff.block(0, 3, 6, x_dim - 6);

    dy_dhc_.block(0, 0, 2, 2 * h_.size()) = Rot * dy_dhc_pre_.block(0, 0, 2, 2 * h_.size());
    dy_dhc_.block(2, 0, 4, 2 * h_.size()) = dy_dhc_pre_.block(2, 0, 4, 2 * h_.size());

    dhc_dx_.col(2) = dhc_dx_pre_.col(0);
    dhc_dx_.col(4) = dhc_dx_pre_.col(1);
    dhc_dx_.col(5) = dhc_dx_pre_.col(2);
    dhc_dx_.block(0, 6, 2 * h_.size(), x_dim - 6) =
      dhc_dx_pre_.block(0, 3, 2 * h_.size(), x_dim - 6);

    return coef * rot_and_d_rot_pred_with_diff;
  }
  Eigen::MatrixXd rot_and_d_rot_error_prediction_with_poly_diff(const Eigen::VectorXd & x)
  {
    const int x_dim = x.size();
    const double theta = x[3];
    const double v = x[2];
    double coef = 2.0 * std::abs(v);
    coef = coef * coef * coef * coef * coef * coef * coef;
    if (coef > 1.0) {
      coef = 1.0;
    }
    const double cos = std::cos(theta);
    const double sin = std::sin(theta);
    Eigen::Matrix2d Rot;
    Rot << cos, -sin, sin, cos;

    Eigen::Matrix2d dRot;
    dRot << -sin, -cos, cos, -sin;
    Eigen::VectorXd vars(x_dim - 3);
    vars[0] = x[2];
    vars[1] = x[4];
    vars[2] = x[5];
    vars.tail(x_dim - 6) = x.tail(x_dim - 6);

    const Eigen::VectorXd pred = error_prediction(vars, -1);
    Eigen::MatrixXd d_pred = Eigen::MatrixXd::Zero(6, x_dim - 3);

    Eigen::VectorXd x_for_polynomial_reg(9);
    x_for_polynomial_reg.head(3) = x.head(3);
    const int acc_start = 3 + std::max(acc_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(3, 3) = x.segment(acc_start, 3);
    const int steer_start = 3 + acc_ctrl_queue_size_ + std::max(steer_delay_step_ - 3, 0);
    x_for_polynomial_reg.segment(6, 3) = x.segment(steer_start, 3);

    const Eigen::MatrixXd polynomial_features_with_diff =
      get_polynomial_features_with_diff(x_for_polynomial_reg, deg_, A_linear_reg_.cols());
    const Eigen::MatrixXd polynomial_reg_diff =
      A_linear_reg_ *
      polynomial_features_with_diff.block(0, 1, A_linear_reg_.cols(), x_for_polynomial_reg.size());
    d_pred.block(0, 0, 6, 3) += polynomial_reg_diff.block(0, 0, 6, 3);
    d_pred.block(0, 1 + acc_start, 6, 3) += polynomial_reg_diff.block(0, 3, 6, 3);
    d_pred.block(0, 1 + steer_start, 6, 3) += polynomial_reg_diff.block(0, 6, 6, 3);

    Eigen::MatrixXd rot_and_d_rot_pred_with_diff = Eigen::MatrixXd::Zero(6, x_dim + 2);
    Eigen::MatrixXd rot_pred_with_diff(6, x_dim - 3);
    rot_pred_with_diff.block(0, 0, 2, x_dim - 3) = Rot * d_pred.block(0, 0, 2, x_dim - 3);
    rot_pred_with_diff.block(2, 0, 4, x_dim - 3) = d_pred.block(2, 0, 4, x_dim - 3);

    rot_and_d_rot_pred_with_diff.block(0, 0, 2, 1) = Rot * pred.head(2);
    rot_and_d_rot_pred_with_diff.block(2, 0, 4, 1) = pred.segment(2, 4);
    rot_and_d_rot_pred_with_diff.block(0, 1, 2, 1) = dRot * pred.head(2);
    rot_and_d_rot_pred_with_diff.col(2 + 2) = rot_pred_with_diff.col(0);
    rot_and_d_rot_pred_with_diff.col(2 + 4) = rot_pred_with_diff.col(1);
    rot_and_d_rot_pred_with_diff.col(2 + 5) = rot_pred_with_diff.col(2);
    rot_and_d_rot_pred_with_diff.block(0, 2 + 6, 6, x_dim - 6) =
      rot_pred_with_diff.block(0, 3, 6, x_dim - 6);

    return coef * rot_and_d_rot_pred_with_diff;
  }
  Eigen::VectorXd rotated_error_prediction(const Eigen::VectorXd & x)
  {
    const int x_dim = x.size();
    const double theta = x[3];
    const double v = x[2];
    double coef = 2.0 * std::abs(v);
    coef = coef * coef * coef * coef * coef * coef * coef;
    if (coef > 1.0) {
      coef = 1.0;
    }
    const double cos = std::cos(theta);
    const double sin = std::sin(theta);
    Eigen::Matrix2d Rot;
    Rot << cos, -sin, sin, cos;
    Eigen::VectorXd vars(x_dim - 3);
    vars[0] = x[2];
    vars[1] = x[4];
    vars[2] = x[5];
    vars.tail(x_dim - 6) = x.tail(x_dim - 6);
    const Eigen::VectorXd pred = error_prediction(vars, -1);
    Eigen::VectorXd rot_pred(6);
    rot_pred.head(2) = coef * Rot * pred.head(2);
    rot_pred.tail(4) = coef * pred.tail(4);
    return rot_pred;
  }
  Eigen::MatrixXd Rotated_error_prediction(const Eigen::MatrixXd & X)
  {
    const int X_cols = X.cols();
    const int x_dim = X.rows();
    Eigen::MatrixXd Pred(6, X_cols);
    for (int i = 0; i < X_cols; i++) {
      const Eigen::VectorXd x = X.col(i);
      const double theta = x[3];
      const double v = x[2];
      double coef = 2.0 * std::abs(v);
      coef = coef * coef * coef * coef * coef * coef * coef;
      if (coef > 1.0) {
        coef = 1.0;
      }
      const double cos = std::cos(theta);
      const double sin = std::sin(theta);
      Eigen::Matrix2d Rot;
      Rot << cos, -sin, sin, cos;
      Eigen::VectorXd vars(x_dim - 3);

      vars[0] = x[2];

      vars[1] = x[4];
      vars[2] = x[5];
      vars.tail(x_dim - 6) = x.tail(x_dim - 6);
      const Eigen::VectorXd pred = error_prediction(vars, i);
      Pred.block(0, i, 2, 1) = coef * Rot * pred.head(2);
      Pred.block(2, i, 4, 1) = coef * pred.tail(4);
    }
    return Pred;
  }
  void update_memory_by_state_history(const Eigen::MatrixXd & X)
  {
    const int X_cols = X.cols();
    const int x_dim = X.rows();
    for (int i = 0; i < X_cols; i++) {
      const Eigen::VectorXd x = X.col(i);
      Eigen::VectorXd vars(x_dim - 3);
      vars[0] = x[2];
      vars[1] = x[4];
      vars[2] = x[5];
      vars.tail(x_dim - 6) = x.tail(x_dim - 6);
      update_memory(vars);
    }
  }
};
PYBIND11_MODULE(proxima_calc, m)
{
  py::class_<transform_model_to_eigen>(m, "transform_model_to_eigen")
    .def(py::init())
    .def("set_params", &transform_model_to_eigen::set_params)
    .def(
      "rot_and_d_rot_error_prediction", &transform_model_to_eigen::rot_and_d_rot_error_prediction)
    .def("error_prediction_with_diff", &transform_model_to_eigen::error_prediction_with_diff)
    .def(
      "rot_and_d_rot_error_prediction_with_diff",
      &transform_model_to_eigen::rot_and_d_rot_error_prediction_with_diff)
    .def(
      "rot_and_d_rot_error_prediction_with_poly_diff",
      &transform_model_to_eigen::rot_and_d_rot_error_prediction_with_poly_diff)
    .def("rotated_error_prediction", &transform_model_to_eigen::rotated_error_prediction)
    .def("Rotated_error_prediction", &transform_model_to_eigen::Rotated_error_prediction);
  py::class_<transform_model_with_memory_to_eigen>(m, "transform_model_with_memory_to_eigen")
    .def(py::init())
    .def("set_params", &transform_model_with_memory_to_eigen::set_params)
    .def("set_params_res", &transform_model_with_memory_to_eigen::set_params_res)
    .def("set_lstm", &transform_model_with_memory_to_eigen::set_lstm)
    .def("set_lstm_for_candidate", &transform_model_with_memory_to_eigen::set_lstm_for_candidate)
    .def("get_h", &transform_model_with_memory_to_eigen::get_h)
    .def("get_c", &transform_model_with_memory_to_eigen::get_c)
    .def("get_dy_dhc", &transform_model_with_memory_to_eigen::get_dy_dhc)
    .def("get_dhc_dx", &transform_model_with_memory_to_eigen::get_dhc_dx)
    .def("get_dhc_dhc", &transform_model_with_memory_to_eigen::get_dhc_dhc)
    .def("error_prediction", &transform_model_with_memory_to_eigen::error_prediction)
    .def(
      "rot_and_d_rot_error_prediction",
      &transform_model_with_memory_to_eigen::rot_and_d_rot_error_prediction)
    .def(
      "rot_and_d_rot_error_prediction_with_diff",
      &transform_model_with_memory_to_eigen::rot_and_d_rot_error_prediction_with_diff)
    .def(
      "rot_and_d_rot_error_prediction_with_memory_diff",
      &transform_model_with_memory_to_eigen::rot_and_d_rot_error_prediction_with_memory_diff)
    .def(
      "rot_and_d_rot_error_prediction_with_poly_diff",
      &transform_model_with_memory_to_eigen::rot_and_d_rot_error_prediction_with_poly_diff)
    .def(
      "update_memory_by_state_history",
      &transform_model_with_memory_to_eigen::update_memory_by_state_history)
    .def(
      "rotated_error_prediction", &transform_model_with_memory_to_eigen::rotated_error_prediction)
    .def(
      "Rotated_error_prediction", &transform_model_with_memory_to_eigen::Rotated_error_prediction);
}
