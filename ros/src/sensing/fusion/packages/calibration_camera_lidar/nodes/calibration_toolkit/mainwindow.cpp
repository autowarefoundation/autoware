#include "mainwindow.h"
#include "ui_mainwindow.h"

#define CALIBCAMERA QString("Camera Only")
#define CALIBCAMERAVELODYNE QString("Camera -> Velodyne")
#define CALIBCAMERA2DLIDAR QString("Camera -> 2D LIDAR")

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QString selection=QInputDialog::getItem(this,"Choose Calibration Type", "Calibration Type",QStringList()<<CALIBCAMERA<<CALIBCAMERAVELODYNE<<CALIBCAMERA2DLIDAR,0,0);

    QSettings settings("RobotSDK","CalibrationToolkit");

    QSizeF patternsize=settings.value("PatternSize",QSizeF(0.035,0.035)).toSizeF();
    ui->patternwidth->setText(QString("%1").arg(patternsize.width()));
    ui->patternheight->setText(QString("%1").arg(patternsize.height()));
    cv::Size2f cvpatternsize=cv::Size2f(patternsize.width(),patternsize.height());

    QSize patternnum=settings.value("PatternNum",QSize(8,6)).toSize();
    ui->patterncolumn->setText(QString("%1").arg(patternnum.width()));
    ui->patternrow->setText(QString("%1").arg(patternnum.height()));
    cv::Size cvpatternnum=cv::Size(patternnum.width(),patternnum.height());


    if(selection==CALIBCAMERA)
    {
        QString cameratopic="/image_raw";
        CalibrateCameraChessboardROS * calibration=new CalibrateCameraChessboardROS(cameratopic,1000,10,cvpatternsize,cvpatternnum);
        ui->tabWidget->addTab(calibration,CALIBCAMERA);
        connect(ui->grab,SIGNAL(clicked()),calibration,SLOT(grabCalibDataSlot()));
        connect(ui->remove,SIGNAL(clicked()),calibration,SLOT(removeCalibDataSlot()));
        connect(ui->calibrate,SIGNAL(clicked()),calibration,SLOT(calibrateSensorSlot()));
        connect(ui->load,SIGNAL(clicked()),calibration,SLOT(loadCalibResultSlot()));
        connect(ui->save,SIGNAL(clicked()),calibration,SLOT(saveCalibResultSlot()));
        connect(ui->refresh,SIGNAL(clicked()),calibration,SLOT(refreshParametersSlot()));
        ui->Project->setEnabled(0);
    }
    else if(selection==CALIBCAMERAVELODYNE)
    {
        QString cameratopic="/image_raw";
        QString velodynetopic="/points_raw";
        CalibrateCameraVelodyneChessboardROS * calibration=new CalibrateCameraVelodyneChessboardROS(cameratopic,1000,10,velodynetopic,1000,10,100,cvpatternsize,cvpatternnum);
        ui->tabWidget->addTab(calibration,CALIBCAMERAVELODYNE);
        connect(ui->grab,SIGNAL(clicked()),calibration,SLOT(grabCalibDataSlot()));
        connect(ui->remove,SIGNAL(clicked()),calibration,SLOT(removeCalibDataSlot()));
        connect(ui->calibrate,SIGNAL(clicked()),calibration,SLOT(calibrateSensorSlot()));
        connect(ui->load,SIGNAL(clicked()),calibration,SLOT(loadCalibResultSlot()));
        connect(ui->save,SIGNAL(clicked()),calibration,SLOT(saveCalibResultSlot()));
        connect(ui->refresh,SIGNAL(clicked()),calibration,SLOT(refreshParametersSlot()));
        connect(ui->Project,SIGNAL(clicked()),calibration,SLOT(projectPointsSlot()));
    }
    else if(selection==CALIBCAMERA2DLIDAR)
    {
        QString cameratopic="/image_raw";
        QString lidartopic="/scan";
        CalibrateCameraLidarChessboardROS * calibration=new CalibrateCameraLidarChessboardROS(cameratopic,1000,10,lidartopic,1000,10,100,cvpatternsize,cvpatternnum);
        ui->tabWidget->addTab(calibration,CALIBCAMERA2DLIDAR);
        connect(ui->grab,SIGNAL(clicked()),calibration,SLOT(grabCalibDataSlot()));
        connect(ui->remove,SIGNAL(clicked()),calibration,SLOT(removeCalibDataSlot()));
        connect(ui->calibrate,SIGNAL(clicked()),calibration,SLOT(calibrateSensorSlot()));
        connect(ui->load,SIGNAL(clicked()),calibration,SLOT(loadCalibResultSlot()));
        connect(ui->save,SIGNAL(clicked()),calibration,SLOT(saveCalibResultSlot()));
        connect(ui->refresh,SIGNAL(clicked()),calibration,SLOT(refreshParametersSlot()));
        connect(ui->Project,SIGNAL(clicked()),calibration,SLOT(projectPointsSlot()));
    }
}

MainWindow::~MainWindow()
{
    QSettings settings("RobotSDK","CalibrationToolkit");
    settings.setValue("PatternSize",QSizeF(ui->patternwidth->text().toFloat(),ui->patternheight->text().toFloat()));
    settings.setValue("PatternNum",QSize(ui->patterncolumn->text().toInt(),ui->patternrow->text().toInt()));
    delete ui;
}
