#ifndef CONVERT_IMAGE_H_
#define CONVERT_IMAGE_H_

#include <QImage>
#include <QPixmap>

#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace convert_image {
  inline QImage CvMatToQImage(const cv::Mat &input) {
    switch (input.type()) {
    case CV_8UC4: {  // 8-bit, 4 channel
      QImage image(input.data, input.cols, input.rows, input.step, QImage::Format_RGB32);
      return image;
    }
    case CV_8UC3: {  // 8-bit, 3 channel
      QImage image(input.data, input.cols, input.rows, input.step, QImage::Format_RGB888);
      return image.rgbSwapped();
    }
    case CV_8UC1: {  // 8-bit, 1 channel
      static QVector<QRgb> color_table;

      // only create color table once
      if (color_table.isEmpty()) {
        for (int i = 0; i < 256; i++) {
          color_table.push_back(qRgb(i, i, i));
        }
      }

      QImage image(input.data, input.cols, input.rows, input.step, QImage::Format_Indexed8);
      image.setColorTable(color_table);
      return image;
    }
    }

    return QImage();
  }


  inline QPixmap CvMatToQPixmap(const cv::Mat &input) {
    return QPixmap::fromImage(CvMatToQImage(input));
  }
}

#endif
