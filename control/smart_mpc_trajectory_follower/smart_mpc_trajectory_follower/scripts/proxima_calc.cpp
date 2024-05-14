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

#include <Eigen/Core>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <iostream>
namespace py = pybind11;

double nominal_model_input(Eigen::VectorXd var, double lam, int step)
{
  double nominal_pred = var[0];
  nominal_pred += (var[step] - nominal_pred) * 0.03333 / lam;
  nominal_pred += (var[step - 1] - nominal_pred) * 0.03333 / lam;
  nominal_pred += (var[step - 2] - nominal_pred) * 0.03333 / lam;
  return nominal_pred;
}
Eigen::RowVectorXd Nominal_model_input(Eigen::MatrixXd Var, double lam, int step)
{
  Eigen::RowVectorXd nominal_pred = Var.row(0);
  nominal_pred += (Var.row(step) - nominal_pred) * 0.03333 / lam;
  nominal_pred += (Var.row(step - 1) - nominal_pred) * 0.03333 / lam;
  nominal_pred += (Var.row(step - 2) - nominal_pred) * 0.03333 / lam;
  return nominal_pred;
}

Eigen::VectorXd d_tanh(Eigen::VectorXd v)
{
  Eigen::VectorXd result = 1 / (v.array().cosh() * v.array().cosh());
  return result;
}
Eigen::VectorXd relu(Eigen::VectorXd x)
{
  Eigen::VectorXd x_ = x;
  for (int i = 0; i < x.size(); i++) {
    if (x[i] < 0) {
      x_[i] = 0;
    }
  }
  return x_;
}
Eigen::VectorXd d_relu(Eigen::VectorXd x)
{
  Eigen::VectorXd result = Eigen::VectorXd::Ones(x.size());
  for (int i = 0; i < x.size(); i++) {
    if (x[i] < 0) {
      result[i] = 0;
    }
  }
  return result;
}

Eigen::MatrixXd d_relu_product(Eigen::MatrixXd m, Eigen::VectorXd x)
{
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(m.rows(), m.cols());
  for (int i = 0; i < m.cols(); i++) {
    if (x[i] >= 0) {
      result.col(i) = m.col(i);
    }
  }
  return result;
}
Eigen::MatrixXd d_tanh_product(Eigen::MatrixXd m, Eigen::VectorXd x)
{
  Eigen::MatrixXd result = Eigen::MatrixXd(m.rows(), m.cols());
  for (int i = 0; i < m.cols(); i++) {
    result.col(i) = m.col(i) / (std::cosh(x[i]) * std::cosh(x[i]));
  }
  return result;
}
Eigen::MatrixXd test_product(Eigen::VectorXd v, Eigen::MatrixXd m)
{
  return v.asDiagonal() * m;
}
Eigen::VectorXd get_polynomial_features(Eigen::VectorXd x, int deg, int dim)
{
  int n_features = x.size();
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
      int end = index[index.size() - 1];
      for (int feature_idx = 0; feature_idx < n_features; feature_idx++) {
        int start = index[feature_idx];
        new_index.push_back(current_idx);
        int next_idx = current_idx + end - start;
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
Eigen::MatrixXd get_polynomial_features_with_diff(Eigen::VectorXd x, int deg, int dim)
{
  int n_features = x.size();
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
      int end = index[index.size() - 1];
      for (int feature_idx = 0; feature_idx < n_features; feature_idx++) {
        int start = index[feature_idx];
        new_index.push_back(current_idx);
        int next_idx = current_idx + end - start;
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
  Eigen::MatrixXd weight_0;
  Eigen::MatrixXd weight_1;
  Eigen::MatrixXd weight_2;
  Eigen::MatrixXd weight_3;
  Eigen::MatrixXd weight_4;
  Eigen::MatrixXd weight_5;
  Eigen::MatrixXd weight_6;
  Eigen::MatrixXd weight_7;
  Eigen::VectorXd bias_0;
  Eigen::VectorXd bias_1;
  Eigen::VectorXd bias_2;
  Eigen::VectorXd bias_3;
  Eigen::VectorXd bias_4;
  Eigen::VectorXd bias_5;
  Eigen::VectorXd bias_6;
  Eigen::VectorXd bias_7;
  Eigen::MatrixXd A_linear_reg;
  Eigen::VectorXd b_linear_reg;
  int deg;
  double acc_time_constant;
  int acc_delay_step;
  double steer_time_constant;
  int steer_delay_step;
  int acc_ctrl_queue_size;
  int steer_ctrl_queue_size;
  int steer_ctrl_queue_size_core;
  double max_acc_error = 20.0;
  double max_steer_error = 20.0;

public:
  transform_model_to_eigen() {}
  void set_params(
    Eigen::MatrixXd weight_0_, Eigen::MatrixXd weight_1_, Eigen::MatrixXd weight_2_,
    Eigen::MatrixXd weight_3_, Eigen::MatrixXd weight_4_, Eigen::MatrixXd weight_5_,
    Eigen::MatrixXd weight_6_, Eigen::MatrixXd weight_7_, Eigen::VectorXd bias_0_,
    Eigen::VectorXd bias_1_, Eigen::VectorXd bias_2_, Eigen::VectorXd bias_3_,
    Eigen::VectorXd bias_4_, Eigen::VectorXd bias_5_, Eigen::VectorXd bias_6_,
    Eigen::VectorXd bias_7_, Eigen::MatrixXd A_linear_reg_, Eigen::VectorXd b_linear_reg_, int deg_,
    double acc_time_constant_, int acc_delay_step_, double steer_time_constant_,
    int steer_delay_step_, int acc_ctrl_queue_size_, int steer_ctrl_queue_size_,
    int steer_ctrl_queue_size_core_)
  {
    weight_0 = weight_0_;
    weight_1 = weight_1_;
    weight_2 = weight_2_;
    weight_3 = weight_3_;
    weight_4 = weight_4_;
    weight_5 = weight_5_;
    weight_6 = weight_6_;
    weight_7 = weight_7_;
    bias_0 = bias_0_;
    bias_1 = bias_1_;
    bias_2 = bias_2_;
    bias_3 = bias_3_;
    bias_4 = bias_4_;
    bias_5 = bias_5_;
    bias_6 = bias_6_;
    bias_7 = bias_7_;
    A_linear_reg = A_linear_reg_;
    b_linear_reg = b_linear_reg_;
    deg = deg_;
    acc_time_constant = acc_time_constant_;
    acc_delay_step = acc_delay_step_;
    steer_time_constant = steer_time_constant_;
    steer_delay_step = steer_delay_step_;
    acc_ctrl_queue_size = acc_ctrl_queue_size_;
    steer_ctrl_queue_size = steer_ctrl_queue_size_;
    steer_ctrl_queue_size_core = steer_ctrl_queue_size_core_;
  }
  Eigen::VectorXd error_prediction(Eigen::VectorXd x)
  {
    Eigen::VectorXd acc_sub(acc_ctrl_queue_size + 1);
    acc_sub[0] = x[1];
    acc_sub.tail(acc_ctrl_queue_size) = x.segment(3, acc_ctrl_queue_size);

    Eigen::VectorXd steer_sub(steer_ctrl_queue_size_core + 1);
    steer_sub[0] = x[2];
    steer_sub.tail(steer_ctrl_queue_size_core) =
      x.segment(3 + acc_ctrl_queue_size, steer_ctrl_queue_size_core);
    Eigen::VectorXd acc_layer_1 = relu(weight_0 * acc_sub + bias_0);

    Eigen::VectorXd steer_layer_1(bias_1.size() + bias_2.size());
    steer_layer_1.head(bias_1.size()) = relu(weight_1 * steer_sub + bias_1);

    Eigen::VectorXd steer_input_full = x.segment(3 + acc_ctrl_queue_size, steer_ctrl_queue_size);
    steer_layer_1.tail(bias_2.size()) = relu(weight_2 * steer_input_full + bias_2);

    Eigen::VectorXd acc_layer_2 = (weight_3 * acc_layer_1 + bias_3).array().tanh();

    Eigen::VectorXd steer_layer_2 = (weight_4 * steer_layer_1 + bias_4).array().tanh();

    Eigen::VectorXd h1(1 + acc_layer_2.size() + steer_layer_2.size());
    h1[0] = x[0];
    h1.segment(1, acc_layer_2.size()) = acc_layer_2;
    h1.tail(steer_layer_2.size()) = steer_layer_2;
    Eigen::VectorXd h2 = relu(weight_5 * h1 + bias_5);
    Eigen::VectorXd h3 = relu(weight_6 * h2 + bias_6);
    Eigen::VectorXd h4(h3.size() + acc_layer_2.size() + steer_layer_2.size());
    h4.head(h3.size()) = h3;
    h4.segment(h3.size(), acc_layer_2.size()) = acc_layer_2;
    h4.tail(steer_layer_2.size()) = steer_layer_2;
    Eigen::VectorXd x_for_polynomial_reg(5);
    x_for_polynomial_reg.head(3) = x.head(3);
    x_for_polynomial_reg[3] = x[3 + acc_delay_step];
    x_for_polynomial_reg[4] = x[3 + acc_ctrl_queue_size + steer_delay_step];
    Eigen::VectorXd y =
      weight_7 * h4 + bias_7 +
      A_linear_reg * get_polynomial_features(x_for_polynomial_reg, deg, A_linear_reg.cols()) +
      b_linear_reg;
    y[4] = std::min(std::max(y[4], -max_acc_error), max_acc_error);
    y[5] = std::min(std::max(y[5], -max_steer_error), max_steer_error);
    return y;
  }
  Eigen::VectorXd rot_and_d_rot_error_prediction(Eigen::VectorXd x)
  {
    int x_dim = x.size();
    double theta = x[3];
    double v = x[2];
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
    double cos = std::cos(theta);
    double sin = std::sin(theta);
    Eigen::Matrix2d Rot;
    Rot << cos, -sin, sin, cos;

    Eigen::Matrix2d dRot;
    dRot << -sin, -cos, cos, -sin;
    Eigen::VectorXd vars(x_dim - 3);
    vars[0] = x[2];
    vars[1] = x[4];
    vars[2] = x[5];
    vars.tail(x_dim - 6) = x.tail(x_dim - 6);
    Eigen::VectorXd pred = error_prediction(vars);
    Eigen::VectorXd rot_and_d_rot_pred(8);
    rot_and_d_rot_pred.head(2) = Rot * pred.head(2);
    rot_and_d_rot_pred.segment(2, 4) = pred.segment(2, 4);
    rot_and_d_rot_pred.tail(2) = dRot * pred.head(2);

    return coef * rot_and_d_rot_pred;
  }
  Eigen::MatrixXd error_prediction_with_diff(Eigen::VectorXd x)
  {
    Eigen::VectorXd acc_sub(acc_ctrl_queue_size + 1);
    acc_sub[0] = x[1];
    acc_sub.tail(acc_ctrl_queue_size) = x.segment(3, acc_ctrl_queue_size);

    Eigen::VectorXd steer_sub(steer_ctrl_queue_size_core + 1);
    steer_sub[0] = x[2];
    steer_sub.tail(steer_ctrl_queue_size_core) =
      x.segment(3 + acc_ctrl_queue_size, steer_ctrl_queue_size_core);
    Eigen::VectorXd steer_input_full = x.segment(3 + acc_ctrl_queue_size, steer_ctrl_queue_size);

    Eigen::VectorXd u_acc_layer_1 = weight_0 * acc_sub + bias_0;
    Eigen::VectorXd acc_layer_1 = relu(u_acc_layer_1);

    Eigen::VectorXd u_steer_layer_1(bias_1.size() + bias_2.size());
    u_steer_layer_1.head(bias_1.size()) = weight_1 * steer_sub + bias_1;
    u_steer_layer_1.tail(bias_2.size()) = weight_2 * steer_input_full + bias_2;
    Eigen::VectorXd steer_layer_1 = relu(u_steer_layer_1);

    Eigen::VectorXd u_acc_layer_2 = weight_3 * acc_layer_1 + bias_3;
    Eigen::VectorXd acc_layer_2 = u_acc_layer_2.array().tanh();

    Eigen::VectorXd u_steer_layer_2 = weight_4 * steer_layer_1 + bias_4;
    Eigen::VectorXd steer_layer_2 = u_steer_layer_2.array().tanh();

    Eigen::VectorXd h1(1 + acc_layer_2.size() + steer_layer_2.size());
    h1[0] = x[0];
    h1.segment(1, acc_layer_2.size()) = acc_layer_2;
    h1.tail(steer_layer_2.size()) = steer_layer_2;
    Eigen::VectorXd u2 = weight_5 * h1 + bias_5;
    Eigen::VectorXd h2 = relu(u2);
    Eigen::VectorXd u3 = weight_6 * h2 + bias_6;
    Eigen::VectorXd h3 = relu(u3);
    Eigen::VectorXd h4(h3.size() + acc_layer_2.size() + steer_layer_2.size());
    h4.head(h3.size()) = h3;
    h4.segment(h3.size(), acc_layer_2.size()) = acc_layer_2;
    h4.tail(steer_layer_2.size()) = steer_layer_2;
    Eigen::VectorXd x_for_polynomial_reg(5);
    x_for_polynomial_reg.head(3) = x.head(3);
    x_for_polynomial_reg[3] = x[3 + acc_delay_step];
    x_for_polynomial_reg[4] = x[3 + acc_ctrl_queue_size + steer_delay_step];
    Eigen::MatrixXd polynomial_features_with_diff =
      get_polynomial_features_with_diff(x_for_polynomial_reg, deg, A_linear_reg.cols());

    Eigen::VectorXd y =
      weight_7 * h4 + bias_7 +
      A_linear_reg * polynomial_features_with_diff.block(0, 0, A_linear_reg.cols(), 1) +
      b_linear_reg;
    y[4] = std::min(std::max(y[4], -max_acc_error), max_acc_error);
    y[5] = std::min(std::max(y[5], -max_steer_error), max_steer_error);
    // Eigen::MatrixXd dy_dh4 = weight_7;

    Eigen::MatrixXd dy_dh3 = weight_7.block(0, 0, y.size(), h3.size());
    Eigen::MatrixXd dy_dh2 = d_relu_product(dy_dh3, u3) * weight_6;
    Eigen::MatrixXd dy_dh1 = d_relu_product(dy_dh2, u2) * weight_5;

    Eigen::MatrixXd dy_da2 = dy_dh1.block(0, 1, y.size(), acc_layer_2.size()) +
                             weight_7.block(0, h3.size(), y.size(), acc_layer_2.size());
    Eigen::MatrixXd dy_ds2 =
      dy_dh1.block(0, 1 + acc_layer_2.size(), y.size(), steer_layer_2.size()) +
      weight_7.block(0, h3.size() + acc_layer_2.size(), y.size(), steer_layer_2.size());
    Eigen::MatrixXd dy_da1 = d_tanh_product(dy_da2, u_acc_layer_2) * weight_3;
    Eigen::MatrixXd dy_ds1 = d_tanh_product(dy_ds2, u_steer_layer_2) * weight_4;

    Eigen::MatrixXd dy_d_acc = d_relu_product(dy_da1, u_acc_layer_1) * weight_0;
    Eigen::MatrixXd dy_d_steer = Eigen::MatrixXd::Zero(y.size(), steer_input_full.size() + 1);
    dy_d_steer.block(0, 1, y.size(), steer_input_full.size()) +=
      d_relu_product(
        dy_ds1.block(0, bias_1.size(), y.size(), bias_2.size()),
        u_steer_layer_1.tail(bias_2.size())) *
      weight_2;
    dy_d_steer.block(0, 0, y.size(), steer_sub.size()) +=
      d_relu_product(
        dy_ds1.block(0, 0, y.size(), bias_1.size()), u_steer_layer_1.head(bias_1.size())) *
      weight_1;

    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(y.size(), x.size() + 1);

    result.col(0) = y;

    result.col(1) = dy_dh1.col(0);
    result.col(2) = dy_d_acc.col(0);
    result.col(3) = dy_d_steer.col(0);
    result.block(0, 4, y.size(), acc_ctrl_queue_size) =
      dy_d_acc.block(0, 1, y.size(), acc_ctrl_queue_size);
    result.block(0, 4 + acc_ctrl_queue_size, y.size(), steer_ctrl_queue_size) =
      dy_d_steer.block(0, 1, y.size(), steer_ctrl_queue_size);

    Eigen::MatrixXd polynomial_reg_diff =
      A_linear_reg *
      polynomial_features_with_diff.block(0, 1, A_linear_reg.cols(), x_for_polynomial_reg.size());
    result.block(0, 1, y.size(), 3) += polynomial_reg_diff.block(0, 0, y.size(), 3);
    result.block(0, 4 + acc_delay_step, y.size(), 1) +=
      polynomial_reg_diff.block(0, 3, y.size(), 1);
    result.block(0, 4 + acc_ctrl_queue_size + steer_delay_step, y.size(), 1) +=
      polynomial_reg_diff.block(0, 4, y.size(), 1);
    return result;
  }
  Eigen::MatrixXd rot_and_d_rot_error_prediction_with_diff(Eigen::VectorXd x)
  {
    int x_dim = x.size();
    double theta = x[3];
    double v = x[2];
    double coef = 2.0 * std::abs(v);
    coef = coef * coef * coef * coef * coef * coef * coef;
    if (coef > 1.0) {
      coef = 1.0;
    }
    double cos = std::cos(theta);
    double sin = std::sin(theta);
    Eigen::Matrix2d Rot;
    Rot << cos, -sin, sin, cos;

    Eigen::Matrix2d dRot;
    dRot << -sin, -cos, cos, -sin;
    Eigen::VectorXd vars(x_dim - 3);
    vars[0] = x[2];
    vars[1] = x[4];
    vars[2] = x[5];
    vars.tail(x_dim - 6) = x.tail(x_dim - 6);

    Eigen::MatrixXd pred_d_pred = error_prediction_with_diff(vars);
    Eigen::VectorXd pred = pred_d_pred.col(0);
    Eigen::MatrixXd d_pred = pred_d_pred.block(0, 1, 6, x_dim - 3);
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
  Eigen::MatrixXd Rotated_error_prediction(Eigen::MatrixXd X)
  {
    int X_cols = X.cols();
    int x_dim = X.rows();
    Eigen::MatrixXd Pred(6, X_cols);
    for (int i = 0; i < X_cols; i++) {
      Eigen::VectorXd x = X.col(i);
      double theta = x[3];
      double v = x[2];
      double coef = 2.0 * std::abs(v);
      coef = coef * coef * coef * coef * coef * coef * coef;
      if (coef > 1.0) {
        coef = 1.0;
      }
      double cos = std::cos(theta);
      double sin = std::sin(theta);
      Eigen::Matrix2d Rot;
      Rot << cos, -sin, sin, cos;
      Eigen::VectorXd vars(x_dim - 3);

      vars[0] = x[2];

      vars[1] = x[4];
      vars[2] = x[5];
      vars.tail(x_dim - 6) = x.tail(x_dim - 6);
      Eigen::VectorXd pred = error_prediction(vars);
      Pred.block(0, i, 2, 1) = coef * Rot * pred.head(2);
      Pred.block(2, i, 4, 1) = coef * pred.tail(4);
    }
    return Pred;
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
    .def("Rotated_error_prediction", &transform_model_to_eigen::Rotated_error_prediction);
}
