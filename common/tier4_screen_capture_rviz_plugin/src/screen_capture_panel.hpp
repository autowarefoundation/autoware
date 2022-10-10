// Copyright 2021 Tier IV, Inc.
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

#ifndef SCREEN_CAPTURE_PANEL_HPP_
#define SCREEN_CAPTURE_PANEL_HPP_

// Qt
#include <QApplication>
#include <QDesktopWidget>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QScreen>
#include <QSpinBox>
#include <QTimer>

// rviz
#include <opencv2/opencv.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>

// ros
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <string>
#include <vector>

class QLineEdit;

class AutowareScreenCapturePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AutowareScreenCapturePanel(QWidget * parent = nullptr);
  ~AutowareScreenCapturePanel() override;
  void update();
  void onInitialize() override;
  void createWallTimer();
  void onTimer();
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;
  void onCaptureTrigger(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    const std_srvs::srv::Trigger::Response::SharedPtr res);

public Q_SLOTS:
  void onClickScreenCapture();
  void onClickVideoCapture();
  void onPrefixChanged();
  void onRateChanged();

private:
  QLabel * ros_time_label_;
  QPushButton * screen_capture_button_ptr_;
  QPushButton * capture_to_mp4_button_ptr_;
  QLineEdit * file_name_prefix_;
  QSpinBox * capture_hz_;
  QTimer * capture_timer_;
  QMainWindow * main_window_{nullptr};
  enum class State { WAITING_FOR_CAPTURE, CAPTURING };
  State state_;
  std::string capture_file_name_;
  bool is_capture_{false};
  cv::VideoWriter writer_;
  cv::Size current_movie_size_;
  std::vector<cv::Mat> image_vec_;

  static std::string stateToString(const State & state)
  {
    if (state == State::WAITING_FOR_CAPTURE) {
      return "waiting for capture";
    }
    if (state == State::CAPTURING) {
      return "capturing";
    }
    return "";
  }

protected:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  rclcpp::Node::SharedPtr raw_node_;
};

#endif  // SCREEN_CAPTURE_PANEL_HPP_
