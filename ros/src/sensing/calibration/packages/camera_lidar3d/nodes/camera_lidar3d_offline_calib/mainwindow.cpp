#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    CalibrateCameraVelodyneChessboardROS * calibration=new CalibrateCameraVelodyneChessboardROS("/camera/image_raw",1000,10,"/velodyne_points",1000,10,30,cv::Size2f(0.108,0.108),cv::Size2i(8,6));
    ui->tabWidget->addTab(calibration,"Calibration");

    connect(ui->grab,SIGNAL(clicked()),calibration,SLOT(grabCalibDataSlot()));
    connect(ui->calibrate,SIGNAL(clicked()),calibration,SLOT(calibrateSensorSlot()));
    connect(ui->load,SIGNAL(clicked()),calibration,SLOT(loadCalibResultSlot()));
    connect(ui->save,SIGNAL(clicked()),calibration,SLOT(saveCalibResultSlot()));
    connect(ui->Project,SIGNAL(clicked()),calibration,SLOT(projectVelodynePointsSlot()));
    connect(ui->refresh,SIGNAL(clicked()),calibration,SLOT(refreshParametersSlot()));
}

MainWindow::~MainWindow()
{
    delete ui;
}
