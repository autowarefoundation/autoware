/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
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
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CALIBRATIONTOOLKIT_H
#define CALIBRATIONTOOLKIT_H

#include<qwidget.h>
#include<QHBoxLayout>
#include<QVBoxLayout>
#include<qlabel.h>
#include<qpushbutton.h>
#include<qtablewidget.h>
#include<qfiledialog.h>
#include<qimage.h>
#include<qdatetime.h>
#include<qtabwidget.h>
#include<qscrollarea.h>
#include<qdebug.h>
#include<qsplitter.h>
#include<qframe.h>
#include<qmessagebox.h>

#include<opencv2/opencv.hpp>

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include<sensor_msgs/Image.h>
#include<sensor_msgs/PointCloud2.h>

#include<nlopt.hpp>

#include<rosinterface.h>
#include<glviewer.h>

#include"selectionwidget.h"

#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"
#define CHESSBOARDPOSE "ChessboardPose"
#define CHESSBOARDIMAGE "ChessboardImage"
#define CHESSBOARDVIEWNUM "ChessboardViewNUM"
#define GRID3DPOINTS "Grid3DPoints"
#define GRID2DPOINTS "Grid2DPoints"
#define REPROJECTIONERROR "ReprojectionError"
#define VELODYNEPOINTS "VelodynePoints"
#define VELODYNENORMALS "VelodyneNormals"
#define VELODYNEVIEWERPOSE "VelodyneViewerPose"
#define CALIBRATIONERROR "CalibrationError"

class CalibrationToolkitBase : public QWidget
{
    Q_OBJECT
public:
    CalibrationToolkitBase(QWidget * parent=0);
protected:
    QSplitter * splitter;
    QSplitter * calibsplitter;
public slots:
    void grabCalibDataSlot();
    void removeCalibDataSlot();
    void calibrateSensorSlot();
    void loadCalibResultSlot();
    void saveCalibResultSlot();
signals:
    void calibDataGrabbedSignal();
    void calibDataGrabbedErrorSignal();
    void calibDataRemovedSignal();
    void calibDataRemovedErrorSignal();
    void sensorCalibratedSignal();
    void sensorCalibratedErrorSignal();
    void calibResultLoadedSignal();
    void calibResultLoadedErrorSignal();
    void calibResultSavedSignal();
    void calibResultSavedErrorSignal();
protected:
    virtual bool grabCalibData()=0;
    virtual bool removeCalibData()=0;
    virtual bool calibrateSensor()=0;
    virtual bool loadCalibResult(cv::FileStorage & fs)=0;
    virtual bool saveCalibResult(cv::FileStorage & fs)=0;
protected:
    void setResultShow(cv::Mat result, QTableWidget * show);
    void readResultShow(cv::Mat &result, QTableWidget * show);
    QVector<double> convertMatrix2Euler(cv::Mat mat);
};

class CalibrateCameraBase : public CalibrationToolkitBase
{
    Q_OBJECT
public:
    CalibrateCameraBase(QWidget * parent=0);
protected:
    QTabWidget * cameracalibtab;
    cv::Mat cameraextrinsicmat;
    QTableWidget * cameraextrinsicshow;
    cv::Mat cameramat;
    QTableWidget * cameramatshow;
    cv::Mat distcoeff;
    QTableWidget * distcoeffshow;
    cv::Size2i imagesize;
    QLabel * imagesizeshow;

    QSplitter * imagesplitter;
    QTabWidget * cameraimagetab;
    QTime cameratimestamp;
    cv::Mat calibimage;
    QLabel * calibimageshow;
    QVector<QRgb> colorTable;
protected slots:
    void refreshImageSlot();
public slots:
    void refreshParametersSlot();
signals:
    void imageRefreshedSignal();
    void imageRefreshedErrorSignal();
protected:
    virtual bool refreshImage();
    bool loadCalibResult(cv::FileStorage & fs);
    bool saveCalibResult(cv::FileStorage & fs);
public:
    cv::Mat getCameraExtrinsicMat();
    cv::Mat getCameraMat();
    cv::Mat getDistCoeff();
    cv::Size2i getImageSize();
};

class CalibrateCameraChessboardBase : public CalibrateCameraBase
{
    Q_OBJECT
public:
    CalibrateCameraChessboardBase(cv::Size2f patternSize, cv::Size2i patternNum, QWidget * parent=0);
protected:
    cv::Size2i patternnum;
    cv::Size2f patternsize;

    cv::vector<cv::Point3f> grid3dpoint;
    cv::vector<cv::vector<cv::Point3f> > grid3dpoints;
    cv::vector<cv::vector<cv::Point2f> > grid2dpoints;

    QTabWidget * chessboardtab;
    cv::vector<cv::Mat> chessboardposes;
    QTabWidget * chessboardposeshow;
    double reprojectionerror;
    QLabel * reprojectionerrorshow;

    cv::vector<cv::Mat> calibimages;
    QTabWidget * calibimagesshow;
protected:
    bool removeCalibData();
    bool calibrateSensor();
    bool loadCalibResult(cv::FileStorage & fs);
    bool saveCalibResult(cv::FileStorage & fs);
public:
    int getChessboardNum();
    cv::vector<cv::Mat> getChessboardPoses();
    cv::Mat getChessboardPose(int id);
    cv::Mat getCalibImage(int id);
};

class CalibrateCameraChessboardROS : public CalibrateCameraChessboardBase
{
    Q_OBJECT
public:
    CalibrateCameraChessboardROS(QString topic, u_int32_t queueSize, int interval, cv::Size2f patternSize, cv::Size2i patternNum, QWidget * parent=0);
    ~CalibrateCameraChessboardROS();
protected:
    ROSSub<sensor_msgs::ImageConstPtr> * camerasub;
protected:
    bool refreshImage();
    bool grabCalibData();
};

class CalibrateCameraVelodyneChessboardBase : public CalibrateCameraChessboardBase
{
    Q_OBJECT
public:
    CalibrateCameraVelodyneChessboardBase(float maxRange, cv::Size2f patternSize, cv::Size2i patternNum, QWidget * parent=0);
public:
    struct CameraVelodyneCalibrationData
    {
        cv::Mat chessboardnormals;   //n*3
        cv::Mat chessboardpoints;    //n*3
        cv::vector<cv::Mat> velodynepoints; //n*m*3
        cv::Mat velodynenormals;
        cv::Mat rotationresult;
    };
protected:
    float maxrange;

    QTabWidget * calibvelodynetab;

    QVector<pcl::PointCloud<pcl::PointXYZI>::Ptr> calibvelodynespoints;
    QVector<cv::Mat> calibvelodynesnormals;
    QTabWidget * calibvelodynepointstab;
    QTabWidget * calibvelodynenormalstab;

    double calibrationrotationalerror;
    double calibrationtranslationalerror;
    QLabel * calibrationerrorshow;

    QSplitter * velodynesplitter;
    QTabWidget * velodynetab;
    QTime velodynetimestamp;
    sensor_msgs::PointCloud2ConstPtr calibvelodyne;
    GLViewer * calibvelodyneviewer;
    GLuint calibvelodynedisplaylist;

    QTabWidget * calibvelodynesshow;
protected slots:
    void refreshVelodyneSlot();
public slots:
    void extractionResultSlot(pcl::PointCloud<pcl::PointXYZI>::Ptr extraction, cv::Mat normal, int id);
    void projectVelodynePointsSlot();
signals:
    void velodyneRefreshedSignal();
    void velodyneRefreshedErrorSignal();
protected:
    virtual bool refreshVelodyne();
    bool removeCalibData();
    bool calibrateSensor();
    bool loadCalibResult(cv::FileStorage &fs);
    bool saveCalibResult(cv::FileStorage &fs);
};

class CalibrateCameraVelodyneChessboardROS : public CalibrateCameraVelodyneChessboardBase
{
    Q_OBJECT
public:
    CalibrateCameraVelodyneChessboardROS(QString cameraTopic, u_int32_t cameraQueueSize, int cameraInterval, QString velodyneTopic, u_int32_t velodyneQueueSize, int velodyneInterval, float maxRange, cv::Size2f patternSize, cv::Size2i patternNum, QWidget * parent=0);
    ~CalibrateCameraVelodyneChessboardROS();
protected:
    ROSSub<sensor_msgs::ImageConstPtr> * camerasub;
    ROSSub<sensor_msgs::PointCloud2ConstPtr> * velodynesub;
protected:
    bool refreshImage();
    bool refreshVelodyne();
    bool grabCalibData();
};

#endif // CALIBRATIONTOOLKIT_H
