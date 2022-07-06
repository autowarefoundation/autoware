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
  time_t seconds = static_cast<time_t>(time);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H-%M-%S", localtime(&seconds));
  line->setText(QString(buffer));
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
    cap_layout->addWidget(screen_capture_button_ptr_);
    cap_layout->addWidget(ros_time_label_);
    cap_layout->addWidget(new QLabel(" [.mp4] "));
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
  QTimer * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &AutowareScreenCapturePanel::update);
  timer->start(1000);
  capture_timer_ = new QTimer(this);
  connect(capture_timer_, &QTimer::timeout, this, &AutowareScreenCapturePanel::onTimer);
  state_ = State::WAITING_FOR_CAPTURE;
}

void AutowareScreenCapturePanel::onCaptureTrigger(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
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

void AutowareScreenCapturePanel::onRateChanged() {}

void AutowareScreenCapturePanel::onClickScreenCapture()
{
  const std::string time_text = "capture/" + ros_time_label_->text().toStdString();
  getDisplayContext()->getViewManager()->getRenderPanel()->getRenderWindow()->captureScreenShot(
    time_text + ".png");
  return;
}

void AutowareScreenCapturePanel::convertPNGImagesToMP4()
{
  cv::VideoCapture capture(root_folder_ + "/%06d.png", cv::CAP_IMAGES);
  if (!capture.isOpened()) {
    return;
  }
  int fourcc = cv::VideoWriter::fourcc('h', '2', '6', '4');  // mp4
  cv::VideoWriter writer;
  cv::Size size = cv::Size(width_, height_);
  writer.open("capture/" + root_folder_ + ".mp4", fourcc, capture_hz_->value(), size);
  cv::Mat image;
  while (true) {
    capture >> image;
    if (image.empty()) {
      break;
    }
    // need to resize for fixed frame video
    writer << image;
    cv::waitKey(0);
  }
  capture.release();
  writer.release();
  // remove temporary created folder
  std::filesystem::remove_all(root_folder_);
}

void AutowareScreenCapturePanel::onClickVideoCapture()
{
  const int clock = static_cast<int>(1e3 / capture_hz_->value());
  try {
    const QWidgetList top_level_widgets = QApplication::topLevelWidgets();
    for (QWidget * widget : top_level_widgets) {
      QMainWindow * main_window_candidate = qobject_cast<QMainWindow *>(widget);
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
        counter_ = 0;
        root_folder_ = ros_time_label_->text().toStdString();
        std::filesystem::create_directory(root_folder_);
      }
      capture_to_mp4_button_ptr_->setText("capturing rviz screen");
      capture_to_mp4_button_ptr_->setStyleSheet("background-color: #FF0000;");
      capture_timer_->start(clock);
      state_ = State::CAPTURING;
      break;
    case State::CAPTURING: {
      capture_timer_->stop();
    }
      capture_to_mp4_button_ptr_->setText("writing to video");
      capture_to_mp4_button_ptr_->setStyleSheet("background-color: #FFFF00;");
      convertPNGImagesToMP4();
      capture_to_mp4_button_ptr_->setText("waiting for capture");
      capture_to_mp4_button_ptr_->setStyleSheet("background-color: #00FF00;");
      state_ = State::WAITING_FOR_CAPTURE;
      break;
  }
  return;
}

void AutowareScreenCapturePanel::onTimer()
{
  std::stringstream count_text;
  count_text << std::setw(6) << std::setfill('0') << counter_;
  const std::string file = root_folder_ + "/" + count_text.str() + ".png";
  if (!main_window_) return;
  try {
    // this is deprecated but only way to capture nicely
    QScreen * screen = QGuiApplication::primaryScreen();
    QPixmap original_pixmap = screen->grabWindow(main_window_->winId());
    QString format = "png";
    QString file_name = QString::fromStdString(file);
    if (!file_name.isEmpty())
      original_pixmap.scaled(width_, height_).save(file_name, format.toLatin1().constData());
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  counter_++;
}

void AutowareScreenCapturePanel::update()
{
  setFormatDate(ros_time_label_, raw_node_->get_clock()->now().seconds());
}

void AutowareScreenCapturePanel::save(rviz_common::Config config) const
{
  Panel::save(config);
  config.mapSetValue("width", width_);
  config.mapSetValue("height", height_);
}

void AutowareScreenCapturePanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
  if (!config.mapGetFloat("width", &width_)) width_ = 1280;
  if (!config.mapGetFloat("height", &height_)) height_ = 720;
}

AutowareScreenCapturePanel::~AutowareScreenCapturePanel()
{
  std::filesystem::remove_all(root_folder_);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(AutowareScreenCapturePanel, rviz_common::Panel)
