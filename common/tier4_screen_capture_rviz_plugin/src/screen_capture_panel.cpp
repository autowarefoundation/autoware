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

#include "screen_capture_panel.hpp"

#include <rclcpp/rclcpp.hpp>

#include <ctime>
#include <filesystem>
#include <iostream>

using std::placeholders::_1;
using std::placeholders::_2;

void setFormatDate(QLabel * line, double time)
{
  char buffer[128];
  auto seconds = static_cast<time_t>(time);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H-%M-%S", localtime(&seconds));
  line->setText(QString("- ") + QString(buffer));
}

AutowareScreenCapturePanel::AutowareScreenCapturePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  std::filesystem::create_directory("capture");
  auto * v_layout = new QVBoxLayout;
  // screen capture
  auto * cap_layout = new QHBoxLayout;
  {
    ros_time_label_ = new QLabel;
    screen_capture_button_ptr_ = new QPushButton("Capture Screen Shot");
    connect(screen_capture_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickScreenCapture()));
    file_name_prefix_ = new QLineEdit("cap");
    connect(file_name_prefix_, SIGNAL(valueChanged(std::string)), this, SLOT(onPrefixChanged()));
    cap_layout->addWidget(screen_capture_button_ptr_);
    cap_layout->addWidget(file_name_prefix_);
    cap_layout->addWidget(ros_time_label_);
    // initialize file name system clock is better for identification.
    setFormatDate(ros_time_label_, rclcpp::Clock().now().seconds());
  }

  // video capture
  auto * video_cap_layout = new QHBoxLayout;
  {
    capture_to_mp4_button_ptr_ = new QPushButton("Capture Screen");
    connect(capture_to_mp4_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickVideoCapture()));
    capture_hz_ = new QSpinBox();
    capture_hz_->setRange(1, 10);
    capture_hz_->setValue(10);
    capture_hz_->setSingleStep(1);
    connect(capture_hz_, SIGNAL(valueChanged(int)), this, SLOT(onRateChanged()));
    // video cap layout
    video_cap_layout->addWidget(capture_to_mp4_button_ptr_);
    video_cap_layout->addWidget(capture_hz_);
    video_cap_layout->addWidget(new QLabel(" [Hz]"));
  }

  // consider layout
  {
    v_layout->addLayout(cap_layout);
    v_layout->addLayout(video_cap_layout);
    setLayout(v_layout);
  }
  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &AutowareScreenCapturePanel::update);
  timer->start(1000);
  capture_timer_ = new QTimer(this);
  connect(capture_timer_, &QTimer::timeout, this, &AutowareScreenCapturePanel::onTimer);
  state_ = State::WAITING_FOR_CAPTURE;
}

void AutowareScreenCapturePanel::onCaptureTrigger(
  [[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr req,
  const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  onClickVideoCapture();
  res->success = true;
  res->message = stateToString(state_);
}

void AutowareScreenCapturePanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  capture_service_ = raw_node_->create_service<std_srvs::srv::Trigger>(
    "/debug/service/capture_screen",
    std::bind(&AutowareScreenCapturePanel::onCaptureTrigger, this, _1, _2));
}

void onPrefixChanged()
{
}

void AutowareScreenCapturePanel::onRateChanged()
{
}

void AutowareScreenCapturePanel::onClickScreenCapture()
{
  const std::string time_text =
    "capture/" + file_name_prefix_->text().toStdString() + ros_time_label_->text().toStdString();
  getDisplayContext()->getViewManager()->getRenderPanel()->getRenderWindow()->captureScreenShot(
    time_text + ".png");
}

void AutowareScreenCapturePanel::onClickVideoCapture()
{
  const int clock = static_cast<int>(1e3 / capture_hz_->value());
  try {
    const QWidgetList top_level_widgets = QApplication::topLevelWidgets();
    for (QWidget * widget : top_level_widgets) {
      auto * main_window_candidate = qobject_cast<QMainWindow *>(widget);
      if (main_window_candidate) {
        main_window_ = main_window_candidate;
      }
    }
  } catch (...) {
    return;
  }
  if (!main_window_) return;
  switch (state_) {
    case State::WAITING_FOR_CAPTURE:
      // initialize setting
      {
        capture_file_name_ = ros_time_label_->text().toStdString();
      }
      capture_to_mp4_button_ptr_->setText("capturing rviz screen");
      capture_to_mp4_button_ptr_->setStyleSheet("background-color: #FF0000;");
      {
        int fourcc = cv::VideoWriter::fourcc('h', '2', '6', '4');  // mp4
        QScreen * screen = QGuiApplication::primaryScreen();
        const auto q_size = screen->grabWindow(main_window_->winId())
                              .toImage()
                              .convertToFormat(QImage::Format_RGB888)
                              .rgbSwapped()
                              .size();
        current_movie_size_ = cv::Size(q_size.width(), q_size.height());
        writer_.open(
          "capture/" + file_name_prefix_->text().toStdString() + capture_file_name_ + ".mp4",
          fourcc, capture_hz_->value(), current_movie_size_);
      }
      capture_timer_->start(clock);
      state_ = State::CAPTURING;
      break;
    case State::CAPTURING:
      writer_.release();
      capture_timer_->stop();
      capture_to_mp4_button_ptr_->setText("waiting for capture");
      capture_to_mp4_button_ptr_->setStyleSheet("background-color: #00FF00;");
      state_ = State::WAITING_FOR_CAPTURE;
      break;
  }
}

void AutowareScreenCapturePanel::onTimer()
{
  if (!main_window_) return;
  // this is deprecated but only way to capture nicely
  QScreen * screen = QGuiApplication::primaryScreen();
  QPixmap original_pixmap = screen->grabWindow(main_window_->winId());
  const auto q_image =
    original_pixmap.toImage().convertToFormat(QImage::Format_RGB888).rgbSwapped();
  const int h = q_image.height();
  const int w = q_image.width();
  cv::Size size = cv::Size(w, h);
  cv::Mat image(
    size, CV_8UC3, const_cast<uchar *>(q_image.bits()),
    static_cast<size_t>(q_image.bytesPerLine()));
  if (size != current_movie_size_) {
    cv::Mat new_image;
    cv::resize(image, new_image, current_movie_size_);
    writer_.write(new_image);
  } else {
    writer_.write(image);
  }
  cv::waitKey(0);
}

void AutowareScreenCapturePanel::update()
{
  setFormatDate(ros_time_label_, rclcpp::Clock().now().seconds());
}

void AutowareScreenCapturePanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void AutowareScreenCapturePanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

AutowareScreenCapturePanel::~AutowareScreenCapturePanel() = default;

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(AutowareScreenCapturePanel, rviz_common::Panel)
