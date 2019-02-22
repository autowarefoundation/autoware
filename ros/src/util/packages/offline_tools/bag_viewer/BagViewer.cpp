/*
 *  Copyright (c) 2019, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *  THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <thread>

#include <QFileDialog>
#include <QString>

#include "BagViewer.h"
#include "ui_BagViewer.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

using namespace std;

using ptime = boost::posix_time::ptime;
using tduration = boost::posix_time::time_duration;

inline ptime getCurrentTime() {
  return boost::posix_time::microsec_clock::local_time();
}

BagViewer::BagViewer(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::BagViewer) {
  ui->setupUi(this);

  timeOffsetIndicator = new ClickableLabel("0.00", this);
  ui->statusbar->addPermanentWidget(timeOffsetIndicator);
  this->connect(timeOffsetIndicator, SIGNAL(clicked()), this,
                SLOT(timeOffsetIndicator_clicked()));
}

BagViewer::~BagViewer() { delete ui; }

void BagViewer::setBagFile(const std::string &bagfilename) {
  bagFdPtr = std::shared_ptr<rosbag::Bag>(
      new rosbag::Bag(bagfilename, rosbag::BagMode::Read));

  ui->topicSelector->clear();

  auto vTopicList = RandomAccessBag::getTopicList(*bagFdPtr);
  for (auto vi : vTopicList) {

    if (vi.second == "sensor_msgs/Image" or
        vi.second == "sensor_msgs/CompressedImage") {
      RandomAccessBag::Ptr smImg(new RandomAccessBag(*bagFdPtr, vi.first));
      imageBagList.push_back(smImg);
      ui->topicSelector->addItem(QString(vi.first.c_str()));
    }
  }

  setTopic(0);
}

void BagViewer::setTopic(int n) {
  if (n >= imageBagList.size())
    return;

  currentActiveTopic = imageBagList.at(n);
  ui->playProgress->setRange(0, currentActiveTopic->size() - 1);
  ui->playProgress->setValue(0);

  updateImage(0);
}

void BagViewer::on_playButton_clicked(bool checked) {
  static bool playStarted = false;
  static std::thread *playerThread = NULL;

  std::function<void()> playThreadFn = [&]() {
    const int startPos = ui->playProgress->sliderPosition();
    disableControlsOnPlaying(true);
    for (int p = startPos; p <= ui->playProgress->maximum(); p++) {

      ptime t1x = getCurrentTime();
      ui->playProgress->setSliderPosition(p);
      updateImage(p);
      if (playStarted == false)
        break;

      if (p < ui->playProgress->maximum()) {
        ros::Time t1 = currentActiveTopic->timeAt(p),
                  t2 = currentActiveTopic->timeAt(p + 1);
        ptime t2x = getCurrentTime();
        tduration tdx = t2x - t1x; // processing overhead
        tduration td = (t2 - t1).toBoost() - tdx;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(td.total_milliseconds()));
      }
    }
    disableControlsOnPlaying(false);
  };

  if (checked == true) {
    playStarted = true;
    playerThread = new std::thread(playThreadFn);
  }

  else {
    playStarted = false;
    playerThread->join();
    delete (playerThread);
  }

  return;
}

void BagViewer::on_playProgress_sliderMoved(int i) { updateImage(i); }

void BagViewer::on_topicSelector_currentIndexChanged(int i) { setTopic(i); }

void BagViewer::on_saveButton_clicked(bool checked) {
  QString fname = QFileDialog::getSaveFileName(this, tr("Save Image"));
  if (fname.length() == 0)
    return;

  cv::Mat rgb;
  cv::cvtColor(currentImage, rgb, CV_RGB2BGR);

  cv::imwrite(fname.toStdString(), rgb);
}

void BagViewer::updateImage(int n) {
  if (n >= currentActiveTopic->size())
    return;
  currentPosition = n;

  if (currentActiveTopic->messageType() == "sensor_msgs/Image") {
    sensor_msgs::Image::ConstPtr imageMsg =
        currentActiveTopic->at<sensor_msgs::Image>(n);
    currentImage =
        cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::RGB8)
            ->image;

  }

  else if (currentActiveTopic->messageType() == "sensor_msgs/CompressedImage") {
    sensor_msgs::CompressedImage::ConstPtr imageMsg =
        currentActiveTopic->at<sensor_msgs::CompressedImage>(n);
    currentImage =
        cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::RGB8)
            ->image;
  }

  QImage curImage(currentImage.data, currentImage.cols, currentImage.rows,
                  currentImage.step[0], QImage::Format_RGB888);
  ui->imageFrame->setImage(curImage);
  updateTimeOffsetIndicator();
}

void BagViewer::disableControlsOnPlaying(bool state) {
  ui->topicSelector->setDisabled(state);
  ui->saveButton->setDisabled(state);
  ui->playProgress->setDisabled(state);
}

void BagViewer::timeOffsetIndicator_clicked() {
  if (timeOffsetIndicatorMode == OFFSET_TIME)
    timeOffsetIndicatorMode = OFFSET_INTEGER;
  else if (timeOffsetIndicatorMode == OFFSET_INTEGER)
    timeOffsetIndicatorMode = OFFSET_TIME;

  updateTimeOffsetIndicator();
}

void BagViewer::updateTimeOffsetIndicator() {
  string toi;

  if (timeOffsetIndicatorMode == OFFSET_INTEGER) {
    auto td = currentActiveTopic->timeAt(currentPosition) -
              currentActiveTopic->timeAt(0);
    stringstream ss;
    ss << fixed << setprecision(2) << td.toSec();
    toi = ss.str();
  }

  else if (timeOffsetIndicatorMode == OFFSET_TIME) {
    toi = std::to_string(currentPosition);
  }

  timeOffsetIndicator->setText(QString::fromStdString(toi));
}
