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
