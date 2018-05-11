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
#include<qcombobox.h>

#include<opencv2/opencv.hpp>

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include<sensor_msgs/Image.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/LaserScan.h>

#include<nlopt.hpp>

#include<rosinterface/rosinterface.h>
#include<glviewer/glviewer.h>

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
#define LIDARPOINTS "LidarPoints"

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

    std::vector<cv::Point3f> grid3dpoint;
    std::vector<std::vector<cv::Point3f> > grid3dpoints;
    std::vector<std::vector<cv::Point2f> > grid2dpoints;

    enum CHESSBOARDTYPE
    {
        BoxGrid,
        CircleGrid
    };

    QComboBox * chessboardtype;

    QTabWidget * chessboardtab;
    std::vector<cv::Mat> chessboardposes;
    QTabWidget * chessboardposeshow;
    double reprojectionerror;
    QLabel * reprojectionerrorshow;

    std::vector<cv::Mat> calibimages;
    QTabWidget * calibimagesshow;
protected:
    bool removeCalibData();
    bool calibrateSensor();
    bool loadCalibResult(cv::FileStorage & fs);
    bool saveCalibResult(cv::FileStorage & fs);
public:
    int getChessboardNum();
    std::vector<cv::Mat> getChessboardPoses();
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
        std::vector<cv::Mat> velodynepoints; //n*m*3
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
    void projectPointsSlot();
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

class CalibrateCameraLidarChessboardBase : public CalibrateCameraChessboardBase
{
    Q_OBJECT
public:
    CalibrateCameraLidarChessboardBase(float maxRange, cv::Size2f patternSize, cv::Size2i patternNum, QWidget * parent=0);
public:
    struct CameraLidarCalibrationData
    {
        cv::Mat chessboardnormals;   //n*3
        cv::Mat chessboardpoints;    //n*3
        QVector<cv::Mat> lidarpoints;
    };
protected:
    float maxrange;

    QTabWidget * caliblidartab;

    QVector<QVector<QPointF> > caliblidarspoints;
    QTabWidget * caliblidarpointstab;

    double calibrationerror;
    QLabel * calibrationerrorshow;

    QSplitter * lidarsplitter;
    QTabWidget * lidartab;
    QTime lidartimestamp;
    sensor_msgs::LaserScanConstPtr caliblidar;
    PointsExtractor * caliblidarviewer;

    QTabWidget * caliblidarsshow;
protected slots:
    void refreshLidarSlot();
public slots:
    void extractionResultSlot(QVector<QPointF> extraction, int id);
    void projectPointsSlot();
signals:
    void lidarRefreshedSignal();
    void lidarRefreshedErrorSignal();
protected:
    virtual bool refreshLidar();
    bool removeCalibData();
    bool calibrateSensor();
    bool loadCalibResult(cv::FileStorage &fs);
    bool saveCalibResult(cv::FileStorage &fs);
};

class CalibrateCameraLidarChessboardROS : public CalibrateCameraLidarChessboardBase
{
    Q_OBJECT
public:
    CalibrateCameraLidarChessboardROS(QString cameraTopic, u_int32_t cameraQueueSize, int cameraInterval, QString lidarTopic, u_int32_t lidarQueueSize, int lidarInterval, float maxRange, cv::Size2f patternSize, cv::Size2i patternNum, QWidget * parent=0);
    ~CalibrateCameraLidarChessboardROS();
protected:
    ROSSub<sensor_msgs::ImageConstPtr> * camerasub;
    ROSSub<sensor_msgs::LaserScanConstPtr> * lidarsub;
protected:
    bool refreshImage();
    bool refreshLidar();
    bool grabCalibData();
};

#endif // CALIBRATIONTOOLKIT_H
