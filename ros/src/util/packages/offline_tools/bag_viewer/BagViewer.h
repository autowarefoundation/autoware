/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef BAGVIEWER_H
#define BAGVIEWER_H

#include <map>
#include <memory>
#include <vector>

#include "ClickableLabel.h"
#include <QMainWindow>

#include <rosbag/bag.h>

#include <opencv2/opencv.hpp>

#include "RandomAccessBag.h"

namespace Ui {
class BagViewer;
}

class BagViewer : public QMainWindow {
  Q_OBJECT

public:
  explicit BagViewer(QWidget *parent = 0);
  ~BagViewer();

  void setBagFile(const std::string &bagfilename);

public slots:
  void on_playButton_clicked(bool checked);
  void on_playProgress_sliderMoved(int i);
  void on_topicSelector_currentIndexChanged(int i);
  void on_saveButton_clicked(bool checked);
  void timeOffsetIndicator_clicked();

private:
  Ui::BagViewer *ui;

protected:
  // Store bag file descriptor object
  std::shared_ptr<rosbag::Bag> bagFdPtr = nullptr;
  // Currently active topic's bag view
  RandomAccessBag::Ptr currentActiveTopic = nullptr;

  std::vector<RandomAccessBag::Ptr> imageBagList;

  cv::Mat currentImage;

  void setTopic(int n);

  void updateImage(int n);

  void disableControlsOnPlaying(bool state);

  // Identifies current playing position in integer
  int currentPosition;

  ClickableLabel *timeOffsetIndicator;
  enum { OFFSET_INTEGER, OFFSET_TIME } timeOffsetIndicatorMode = OFFSET_TIME;
  void updateTimeOffsetIndicator();
};

#endif // BAGVIEWER_H
