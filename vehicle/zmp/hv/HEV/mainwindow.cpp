#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;
using namespace zmp::hev;

#include "../hev_base/activate_hev.cpp" 

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    _selectLog.start = false;

    hev = new HevCnt();
    hev->Init();
    hev->Start();

    readTimer = new QTimer(this);
    drvTimer = new QTimer(this);

    ui->setupUi(this);
//    ui->graphicsView->setScene(&scene);
    pixmapItem = NULL;
//    captureInterval = 10;
    scaleValue      = 1.0;
//    _videoSave = false;
    _drvTargetVeloc = 0;
    _drvTargetStroke = 0;

    for(int i=0; i<21; i++)
    {
        _config.data[i] = 0;
    }

//    capture = cvCaptureFromCAM(0);
//    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 320);
//    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 240);
//    cvQueryFrame(capture);
//    _cap_size = new cv::Size(320,240);
//    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
//    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
//    cvQueryFrame(capture);
//    _cap_size = new cv::Size(640,480);

/*    _cap = new cv::VideoCapture(0);
    _cap_size = new cv::Size(320,240);
    _cap->set(CV_CAP_PROP_FRAME_WIDTH, _cap_size->width);
    _cap->set(CV_CAP_PROP_FRAME_HEIGHT, _cap_size->height);
    int fps = 15;
    _frame  = new cv::Mat;
    _writer = new cv::VideoWriter("capture.mjpg", CV_FOURCC('X','V','I','D'), fps, *_cap_size, true);
    cv::namedWindow("video", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
*/
//    this->addToolBar(getToolBar(this));

    connect(ui->horizontalSlider_drvStroke, SIGNAL(valueChanged(int)), this, SLOT(setDrvStrokeSBox()));
    connect(ui->horizontalSlider_drvVeloc, SIGNAL(valueChanged(int)), this, SLOT(setDrvVelocSBox()));
    connect(ui->spinBox_drvStroke, SIGNAL(valueChanged(int)), this, SLOT(setDrvStrokeSlider()));
    connect(ui->spinBox_drvVeloc, SIGNAL(valueChanged(int)), this, SLOT(setDrvVelocSlider()));
    connect(ui->pushButton_drvStroke_zero, SIGNAL(clicked()), this, SLOT(setDrvStrokeZero()));
    connect(ui->pushButton_drvVeloc_zero, SIGNAL(clicked()), this, SLOT(setDrvVelocZero()));
    connect(ui->pushButton_drvManual, SIGNAL(clicked()), this, SLOT(sndDrvModeM()));
    connect(ui->pushButton_drvProgram, SIGNAL(clicked()), this, SLOT(sndDrvModeP()));
    connect(ui->pushButton_drvStroke, SIGNAL(clicked()), this, SLOT(sndDrvCModeS()));
    connect(ui->pushButton_drvVelocity, SIGNAL(clicked()), this, SLOT(sndDrvCModeV()));
    connect(ui->pushButton_drvOverON, SIGNAL(clicked()), this, SLOT(sndDrvOModeON()));
    connect(ui->pushButton_drvOverOFF, SIGNAL(clicked()), this, SLOT(sndDrvOModeOFF()));
    connect(ui->pushButton_drvServoON, SIGNAL(clicked()), this, SLOT(sndDrvServoON()));
    connect(ui->pushButton_drvServoOFF, SIGNAL(clicked()), this, SLOT(sndDrvServoOFF()));
    connect(ui->pushButton_drvShiftSet_D, SIGNAL(clicked()), this, SLOT(sndDrvShiftD()));
    connect(ui->pushButton_drvShiftSet_N, SIGNAL(clicked()), this, SLOT(sndDrvShiftN()));
    connect(ui->pushButton_drvShiftSet_R, SIGNAL(clicked()), this, SLOT(sndDrvShiftR()));
    connect(ui->pushButton_drvShiftSet_B, SIGNAL(clicked()), this, SLOT(sndDrvShiftB()));
    connect(ui->pushButton_drvShiftSet_S, SIGNAL(clicked()), this, SLOT(sndDrvShiftS()));


    connect(ui->horizontalSlider_strAngle, SIGNAL(valueChanged(int)), this, SLOT(setStrAngleSBox()));
    connect(ui->horizontalSlider_strTorque, SIGNAL(valueChanged(int)), this, SLOT(setStrTorqueSBox()));
    connect(ui->spinBox_strAngle, SIGNAL(valueChanged(int)), this, SLOT(setStrAngleSlider()));
    connect(ui->spinBox_strTorque, SIGNAL(valueChanged(int)), this, SLOT(setStrTorqueSlider()));
    connect(ui->pushButton_strAngle_zero, SIGNAL(clicked()), this, SLOT(setStrAngleZero()));
    connect(ui->pushButton_strTorque_zero, SIGNAL(clicked()), this, SLOT(setStrTorqueZero()));
    connect(ui->pushButton_strManual, SIGNAL(clicked()), this, SLOT(sndStrModeM()));
    connect(ui->pushButton_strProgram, SIGNAL(clicked()), this, SLOT(sndStrModeP()));
    connect(ui->pushButton_strAngle, SIGNAL(clicked()),this,SLOT(sndStrCModeA()));
    connect(ui->pushButton_strTorque, SIGNAL(clicked()), this, SLOT(sndStrCModeT()));
    connect(ui->pushButton_strOverON, SIGNAL(clicked()), this, SLOT(sndStrOModeON()));
    connect(ui->pushButton_strOverOFF, SIGNAL(clicked()), this, SLOT(sndStrOModeOFF()));
    connect(ui->pushButton_strServoON, SIGNAL(clicked()), this, SLOT(sndStrServoON()));
    connect(ui->pushButton_strServoOFF, SIGNAL(clicked()), this, SLOT(sndStrServoOFF()));

    connect(ui->horizontalSlider_brkStroke, SIGNAL(valueChanged(int)), this, SLOT(setBrkStrokeSBox()));
    connect(ui->spinBox_brkStroke, SIGNAL(valueChanged(int)), this, SLOT(setBrkStrokeSlider()));
    connect(ui->pushButton_brkStroke_zero, SIGNAL(clicked()), this, SLOT(setBrkStrokeZero()));

    connect(ui->pushButton_diagEngineGet, SIGNAL(clicked()), this, SLOT(SndDiagEngineReq()));
    connect(ui->pushButton_diagEngineClear, SIGNAL(clicked()), this, SLOT(SndDiagEngineClear()));
    connect(ui->pushButton_diagHVGet, SIGNAL(clicked()), this, SLOT(SndDiagHVReq()));
    connect(ui->pushButton_diagHVClear, SIGNAL(clicked()), this, SLOT(SndDiagHVClear()));
    connect(ui->pushButton_diagCruiseGet, SIGNAL(clicked()), this, SLOT(SndDiagCruiseReq()));
    connect(ui->pushButton_diagBattGet, SIGNAL(clicked()), this, SLOT(SndDiagBattReq()));
    connect(ui->pushButton_diagBattClear, SIGNAL(clicked()), this, SLOT(SndDiagBattClear()));
    connect(ui->pushButton_errGet, SIGNAL(clicked()), this, SLOT(SndErrReq()));

    connect(ui->pushButton_configBrkOverrideTh_Get, SIGNAL(clicked()), this, SLOT(GetConfBrkOverrideTh()));
    connect(ui->pushButton_configBrkOverrideTh_Set, SIGNAL(clicked()), this, SLOT(SetConfBrkOverrideTh()));
    connect(ui->pushButton_configDrvOverrideTh_Get, SIGNAL(clicked()), this, SLOT(GetConfAccOverrideTh()));
    connect(ui->pushButton_configDrvOverrideTh_Set, SIGNAL(clicked()), this, SLOT(SetConfAccOverrideTh()));
    connect(ui->pushButton_configSteerCCWFactor_Get, SIGNAL(clicked()), this, SLOT(GetConfStrCCWFactor()));
    connect(ui->pushButton_configSteerCCWFactor_Set, SIGNAL(clicked()), this, SLOT(SetConfStrCCWFactor()));
    connect(ui->pushButton_configSteerDecTime_Get, SIGNAL(clicked()), this, SLOT(GetConfStrDecTime()));
    connect(ui->pushButton_configSteerDecTime_Set, SIGNAL(clicked()), this, SLOT(SetConfStrDecTime()));
    connect(ui->pushButton_configSteerGainKD_Get, SIGNAL(clicked()), this, SLOT(GetConfStrGainKD()));
    connect(ui->pushButton_configSteerGainKD_Set, SIGNAL(clicked()), this, SLOT(SetConfStrGainKD()));
    connect(ui->pushButton_configSteerGainKI_Get, SIGNAL(clicked()), this, SLOT(GetConfStrGainKI()));
    connect(ui->pushButton_configSteerGainKI_Set, SIGNAL(clicked()), this, SLOT(SetConfStrGainKI()));
    connect(ui->pushButton_configSteerGainKP_Get, SIGNAL(clicked()), this, SLOT(GetConfStrGainKP()));
    connect(ui->pushButton_configSteerGainKP_Set, SIGNAL(clicked()), this, SLOT(SetConfStrGainKP()));
    connect(ui->pushButton_configSteerPunch_Get, SIGNAL(clicked()), this, SLOT(GetConfStrPunch()));
    connect(ui->pushButton_configSteerPunch_Set, SIGNAL(clicked()), this, SLOT(SetConfStrPunch()));
    connect(ui->pushButton_configSteerTrqMax_Get, SIGNAL(clicked()), this, SLOT(GetConfStrMaxTrq()));
    connect(ui->pushButton_configSteerTrqMax_Set, SIGNAL(clicked()), this, SLOT(SetConfStrMaxTrq()));
    connect(ui->pushButton_configSteerWindow_Get, SIGNAL(clicked()), this, SLOT(GetConfStrWindow()));
    connect(ui->pushButton_configSteerWindow_Set, SIGNAL(clicked()), this, SLOT(SetConfStrWindow()));
    connect(ui->pushButton_configStrOverrideTh_Get, SIGNAL(clicked()), this, SLOT(GetConfStrOverrideTh()));
    connect(ui->pushButton_configStrOverrideTh_Set, SIGNAL(clicked()), this, SLOT(SetConfStrOverrideTh()));
    connect(ui->pushButton_configTestFlags_Get, SIGNAL(clicked()), this, SLOT(GetConfTestFlags()));
    connect(ui->pushButton_configTestFlags_Set, SIGNAL(clicked()), this, SLOT(SetConfTestFlags()));
    connect(ui->pushButton_configVelocGainKPM_Get, SIGNAL(clicked()), this, SLOT(GetConfVelocGainKPM()));
    connect(ui->pushButton_configVelocGainKPM_Set, SIGNAL(clicked()), this, SLOT(SetConfVelocGainKPM()));
    connect(ui->pushButton_configVelocGainKPP_Get, SIGNAL(clicked()), this, SLOT(GetConfVelocGainKPP()));
    connect(ui->pushButton_configVelocGainKPP_Set, SIGNAL(clicked()), this, SLOT(SetConfVelocGainKPP()));
    connect(ui->pushButton_configVelocMaxOutputM_Get, SIGNAL(clicked()), this, SLOT(GetConfVelocMaxOutputM()));
    connect(ui->pushButton_configVelocMaxOutputM_Set, SIGNAL(clicked()), this, SLOT(SetConfVelocMaxOutputM()));
    connect(ui->pushButton_configVelocMaxOutputP_Get, SIGNAL(clicked()), this, SLOT(GetConfVelocMaxOutputP()));
    connect(ui->pushButton_configVelocMaxOutputP_Set, SIGNAL(clicked()), this, SLOT(SetConfVelocMaxOutputP()));
    connect(ui->pushButton_configVelocPunchM_Get, SIGNAL(clicked()), this, SLOT(GetConfVelocPunchM()));
    connect(ui->pushButton_configVelocPunchM_Set, SIGNAL(clicked()), this, SLOT(SetConfVelocPunchM()));
    connect(ui->pushButton_configVelocPunchP_Get, SIGNAL(clicked()), this, SLOT(GetConfVelocPunchP()));
    connect(ui->pushButton_configVelocPunchP_Set, SIGNAL(clicked()), this, SLOT(SetConfVelocPunchP()));
    connect(ui->pushButton_configVelocSpeedMax_Get, SIGNAL(clicked()), this, SLOT(GetConfVelocMaxSpeed()));
    connect(ui->pushButton_configVelocSpeedMax_Set, SIGNAL(clicked()), this, SLOT(SetConfVelocMaxSpeed()));
    connect(ui->pushButton_configVelocWindowM_Get, SIGNAL(clicked()), this, SLOT(GetConfVelocWindowM()));
    connect(ui->pushButton_configVelocWindowM_Set, SIGNAL(clicked()), this, SLOT(SetConfVelocWindowM()));
    connect(ui->pushButton_configVelocWindowP_Get, SIGNAL(clicked()), this, SLOT(GetConfVelocWindowP()));
    connect(ui->pushButton_configVelocWindowP_Set, SIGNAL(clicked()), this, SLOT(SetConfVelocWindowP()));
    connect(ui->pushButton_configSave, SIGNAL(clicked()), this, SLOT(SaveConfig()));

    connect(readTimer, SIGNAL(timeout()), this, SLOT(readLoop()));
    connect(drvTimer, SIGNAL(timeout()),this, SLOT(sndDrvTarget()));

//    connect(ui->pushButton_camStat, SIGNAL(clicked()), this, SLOT(startVideo()));
//    connect(ui->pushButton_camStop, SIGNAL(clicked()), this, SLOT(stopVideo()));
//    connect(ui->pushButton_camCapture, SIGNAL(clicked()), this, SLOT(takeCapture()));
//    connect(ui->pushButton_camCaptureSave, SIGNAL(clicked()), this, SLOT(saveCapture()));
//    connect(ui->pushButton_camCaptureLoad, SIGNAL(clicked()), this, SLOT(loadCapture()));
//    connect(ui->pushButton_videoStart2, SIGNAL(clicked()), this, SLOT(startVideo2()));
//    connect(ui->pushButton_videoStop2, SIGNAL(clicked()), this, SLOT(stopVideo2()));

    connect(ui->pushButton_startLog, SIGNAL(clicked()), this, SLOT(startLog()));
    connect(ui->pushButton_stopLog, SIGNAL(clicked()), this, SLOT(stopLog()));

    connect(ui->checkBox_gameEnable, SIGNAL(clicked()), this, SLOT(setGameEnable()));

    QFont font;
    font.setPointSize(9);
    ui->textEdit_drvMode->setFont(font);
    ui->textEdit_drvCntMode->setFont(font);
    ui->textEdit_drvMode->setFont(font);
    ui->textEdit_drvOverMode->setFont(font);
    ui->textEdit_drvServo->setFont(font);
    ui->textEdit_drvStrokeActual->setFont(font);
//    ui->textEdit_drvStrokeRawVpa1->setFont(font);
//    ui->textEdit_drvStrokeRawVpa2->setFont(font);
    ui->textEdit_drvStrokeTarget->setFont(font);
    ui->textEdit_drvVelocActual->setFont(font);
    ui->textEdit_drvVelocTarget->setFont(font);

    ui->textEdit_strAngleActual->setFont(font);
    ui->textEdit_strAngleTarget->setFont(font);
    ui->textEdit_strContMode->setFont(font);
    ui->textEdit_strMode->setFont(font);
    ui->textEdit_strOverMode->setFont(font);
    ui->textEdit_strServo->setFont(font);
//    ui->textEdit_strTorqueRawTrq1->setFont(font);
//    ui->textEdit_strTorqueRawTrq2->setFont(font);

    ui->textEdit_brkPressState->setFont(font);
//    ui->textEdit_brkRawPFL->setFont(font);
//    ui->textEdit_brkRawPFR->setFont(font);
//    ui->textEdit_brkRawPRL->setFont(font);
//    ui->textEdit_brkRawPRR->setFont(font);
    ui->textEdit_brkStrokeActual->setFont(font);
//    ui->textEdit_brkStrokeRawPMC1->setFont(font);
//    ui->textEdit_brkStrokeRawPMC2->setFont(font);
//    ui->textEdit_brkStrokeRawSKS1->setFont(font);
//    ui->textEdit_brkStrokeRawSKS2->setFont(font);
    ui->textEdit_brkStrokeTarget->setFont(font);

//    ui->pushButton_camStop->setEnabled(false);
//    ui->pushButton_camCaptureSave->setEnabled(true);
//    ui->pushButton_videoStart2->setEnabled(true);
//    ui->pushButton_videoStop2->setEnabled(false);

    ui->pushButton_startLog->setEnabled(true);
    ui->pushButton_stopLog->setEnabled(false);

    pthread_create(&_logThread, NULL, LogThreadEntry, this);
    readTimer->start(100);
    drvTimer->start(200);

    hev->SetDrvMode(MODE_MANUAL);
    hev->SetStrMode(MODE_MANUAL);

    _gameData.angle = 0;
    _gameData.brake = 0;
    _gameData.button = 0;
    _gameData.drive = 0;

    _gameEnable = false;
    _gameRes = _Game.GameInit();
//    if(_gameRes == true)
//    {
        _Game.SetGameReceiveHandler(this);
//    }

    _Game.GameStart();
    hev->SetConfigCallback(this);
}

MainWindow::~MainWindow()
{

    hev->Stop();
    hev->Close();
    delete ui;
}

bool MainWindow::GameInit()
{
    if(_gameRes == true)
        return true;

    _gameRes = _Game.GameInit();
    if(_gameRes == true){
        _Game.SetGameReceiveHandler(this);
    } else {
        return false;
    }
    _Game.GameStart();
    return true;
}

bool MainWindow::GameStart()
{
    if(_gameRes == true)
        _Game.GameStart();
    return true;
}

void* MainWindow::LogThreadEntry(void* arg)
{
    MainWindow* main = (MainWindow*)arg;
    main->logThread();
    return NULL;
}

void MainWindow::logThread()
{
  //activate hev_base program
  HevBaseActivate();
//    readTimer->start(10);
    while(1){
        updateTime();
        if(_selectLog.start == true){
            writeLog();
        }
        usleep(10*1000);
    }
}

void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch(e->type()){
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

/*void MainWindow::takeCapture()
{
    QImage* qimage = grab2();
    updateTime();
    ui->textEdit_updateTime->setText(_update);

    if((_selectLog.start == true) && (_selectLog.cameraInf == true)){
        mode_t mode;
        mode = S_IRUSR | S_IREAD | S_IWUSR | S_IWRITE | S_IXUSR | S_IEXEC |
                S_IRGRP | S_IWGRP | S_IXGRP | S_IROTH | S_IWOTH | S_IXOTH;
        char dirName[32];
        sprintf(dirName, "img/%d-%d-%d-%d-%d",
                _s_time->tm_year+1900, _s_time->tm_mon+1, _s_time->tm_mday,
                _s_time->tm_hour, _s_time->tm_min);
        mkdir(dirName, mode);

        QString fileName = "img/" + QString::number(_s_time->tm_year+1900 ,10);
        fileName += "-" + QString::number(_s_time->tm_mon+1 ,10);
        fileName += "-" + QString::number(_s_time->tm_mday ,10);
        fileName += "-" + QString::number(_s_time->tm_hour ,10);
        fileName += "-" + QString::number(_s_time->tm_min ,10);
        fileName += "/" + _update;
        fileName += ".jpg";
        if(qimage == NULL)
            return;

        qimage->save(fileName, "jpg", -1);
    }
    _update = "";
    showQImage(qimage);
    delete qimage;
}*/

/*void MainWindow::takeCapture2()
{
//    QImage* qimage = grab2();
    IplImage* frame = cvQueryFrame(capture);
    QImage* qimage = iplToQ(frame);

    cvWriteFrame(_writer, frame);
    showQImage(qimage);
    delete qimage;
}*/

/*void MainWindow::showQImage(QImage* qimage)
{
    if(qimage==NULL)
        return;

    image = *qimage;
    QPixmap pixmap = QPixmap::fromImage(image);
    if(pixmapItem==NULL)
        pixmapItem = scene.addPixmap(pixmap);
    else {
        pixmapItem->setPixmap(pixmap);
    }
    return;
}*/

/*void MainWindow::startVideo()
{
    ui->pushButton_camStat->setEnabled(false);
    ui->pushButton_camCaptureLoad->setEnabled(false);
    ui->pushButton_camCaptureSave->setEnabled(false);
    ui->pushButton_camCapture->setEnabled(false);
    ui->pushButton_camStop->setEnabled(true);
    ui->pushButton_videoStart2->setEnabled(false);
    ui->pushButton_videoStop2->setEnabled(false);
    timer.start((int)(captureInterval*1));
    connect(&timer, SIGNAL(timeout()), this, SLOT(takeCapture()));
}*/

/*void MainWindow::startVideo2()
{
    updateTime();
    char logFile[32];
    sprintf(logFile, "%d_%d_%d_%d_%d_%d.avi",
            _s_time->tm_year+1900, _s_time->tm_mon+1, _s_time->tm_mday,
            _s_time->tm_hour,_s_time->tm_min, _s_time->tm_sec);
//    double fps = 15.0f;

    _writer = cvCreateVideoWriter(logFile, CV_FOURCC('X','V','I','D'), 25, *_cap_size);
    ui->pushButton_camStat->setEnabled(false);
    ui->pushButton_camCaptureLoad->setEnabled(false);
    ui->pushButton_camCaptureSave->setEnabled(false);
    ui->pushButton_camCapture->setEnabled(false);
    ui->pushButton_camStop->setEnabled(false);
    ui->pushButton_videoStart2->setEnabled(false);
    ui->pushButton_videoStop2->setEnabled(true);

    timer.start((int)(captureInterval*1));
    connect(&timer, SIGNAL(timeout()), this, SLOT(takeCapture2()));
}*/

/*void MainWindow::stopVideo()
{
    ui->pushButton_camStat->setEnabled(true);
    ui->pushButton_camStop->setEnabled(false);
    ui->pushButton_camCaptureLoad->setEnabled(true);
    ui->pushButton_camCaptureSave->setEnabled(true);
    ui->pushButton_camCapture->setEnabled(true);
    ui->pushButton_videoStart2->setEnabled(true);
    ui->pushButton_videoStop2->setEnabled(false);
    timer.stop();
    disconnect(this, SLOT(takeCapture()));
}*/

/*void MainWindow::stopVideo2()
{
    timer.stop();
    disconnect(this, SLOT(takeCapture2()));

    ui->pushButton_camStat->setEnabled(true);
    ui->pushButton_camStop->setEnabled(false);
    ui->pushButton_camCaptureLoad->setEnabled(true);
    ui->pushButton_camCaptureSave->setEnabled(true);
    ui->pushButton_camCapture->setEnabled(true);
    ui->pushButton_videoStart2->setEnabled(true);
    ui->pushButton_videoStop2->setEnabled(false);
}*/

/*void MainWindow::loadCapture()
{
    ui->pushButton_camStat->setEnabled(true);
    ui->pushButton_camStop->setEnabled(false);
    ui->pushButton_camCaptureLoad->setEnabled(true);
    ui->pushButton_camCaptureSave->setEnabled(true);
    ui->pushButton_camCapture->setEnabled(true);
    QString fileName = QFileDialog::getOpenFileName(this, "Open Image", "./", "Image Files (*.jpg *.bmp)");
    if(fileName=="")
        return;
    IplImage* iplImage = cvLoadImage(fileName.toAscii());
    showQImage(iplToQ(iplImage));
}*/

/*void MainWindow::saveCapture()
{
    ui->pushButton_camStat->setEnabled(true);
    ui->pushButton_camStop->setEnabled(false);
    ui->pushButton_camCaptureLoad->setEnabled(true);
    ui->pushButton_camCaptureSave->setEnabled(true);
    ui->pushButton_camCapture->setEnabled(true);
    QString fileName = QFileDialog::getSaveFileName(this, "Save Image", "./", "Image Files (*.jpg *.bmp)");
    if(fileName=="")
        return;
    image.save(fileName);
}*/

void MainWindow::startLog()
{
    updateTime();
    char logFile[32];
    sprintf(logFile, "%d_%d_%d_%d_%d_%d.txt",
            _s_time->tm_year+1900, _s_time->tm_mon+1, _s_time->tm_mday,
            _s_time->tm_hour,_s_time->tm_min, _s_time->tm_sec);

    _logSave = fopen(logFile, "w+");
    ui->pushButton_startLog->setEnabled(false);
    ui->pushButton_stopLog->setEnabled(true);

    _selectLog.start = true;
    if(ui->checkBox_battInf->isChecked() == true)
        _selectLog.battInf = true;
    else
        _selectLog.battInf = false;
    if(ui->checkBox_brkInf->isChecked() == true)
        _selectLog.brkInf = true;
    else
        _selectLog.brkInf = false;
//    if(ui->checkBox_camInf->isChecked() == true)
//        _selectLog.cameraInf = true;
//    else
//        _selectLog.cameraInf = false;
    if(ui->checkBox_drvInf->isChecked() == true)
        _selectLog.drvInf = true;
    else
        _selectLog.drvInf = false;
    if(ui->checkBox_otherInf->isChecked() == true)
        _selectLog.otherInf = true;
    else
        _selectLog.otherInf = false;
    if(ui->checkBox_strInf->isChecked() == true)
        _selectLog.strInf = true;
    else
        _selectLog.strInf = false;

    ui->checkBox_battInf->setEnabled(false);
    ui->checkBox_brkInf->setEnabled(false);
    ui->checkBox_drvInf->setEnabled(false);
    ui->checkBox_otherInf->setEnabled(false);
    ui->checkBox_strInf->setEnabled(false);
}

void MainWindow::stopLog()
{
    if(_logSave != NULL)
        fclose(_logSave);

    _selectLog.start = false;

    ui->checkBox_battInf->setEnabled(true);
    ui->checkBox_brkInf->setEnabled(true);
    ui->checkBox_drvInf->setEnabled(true);
    ui->checkBox_otherInf->setEnabled(true);
    ui->checkBox_strInf->setEnabled(true);

    ui->pushButton_startLog->setEnabled(true);
    ui->pushButton_stopLog->setEnabled(false);
}

/*QImage* MainWindow::grab2()
{
    IplImage* frame = cvQueryFrame(capture);
    return iplToQ(frame);
}*/

/*QImage* MainWindow::iplToQ(IplImage* _image)
{
    QImage* qimage = NULL;
    if(_image == NULL)
        return NULL;

    if(_image->depth!=IPL_DEPTH_8U)
        return qimage;
    if(_image->nChannels!=3)
        return qimage;

    qimage = new QImage((unsigned char*)_image->imageData, _image->width, _image->height, QImage::Format_RGB888);
    *qimage = qimage->rgbSwapped();
    return qimage;
}*/



/////////////////////////////////////////////////////////////////////////////
// drive slots
/////////////////////////////////////////////////////////////////////////////
void MainWindow::setDrvStrokeSBox()
{
    if(ui->horizontalSlider_drvStroke->value() != ui->spinBox_drvStroke->value()){
        ui->spinBox_drvStroke->setValue(ui->horizontalSlider_drvStroke->value());
        hev->SetDrvStroke(ui->horizontalSlider_drvStroke->value());
        _drvTargetStroke = ui->horizontalSlider_drvStroke->value();
    }
}

void MainWindow::setDrvVelocSBox()
{
    if(ui->horizontalSlider_drvVeloc->value() != ui->spinBox_drvVeloc->value()){
        ui->spinBox_drvVeloc->setValue(ui->horizontalSlider_drvVeloc->value());
        hev->SetDrvVeloc(ui->horizontalSlider_drvVeloc->value()*100);
        _drvTargetVeloc = ui->horizontalSlider_drvVeloc->value();
    }
}

void MainWindow::setDrvStrokeSlider()
{
    if(ui->spinBox_drvStroke->value() != ui->horizontalSlider_drvStroke->value()){
        ui->horizontalSlider_drvStroke->setValue(ui->spinBox_drvStroke->value());
        hev->SetDrvStroke(ui->spinBox_drvStroke->value());
        _drvTargetStroke = ui->spinBox_drvStroke->value();
    }
}

void MainWindow::setDrvVelocSlider()
{
    if(ui->spinBox_drvVeloc->value() != ui->horizontalSlider_drvVeloc->value()){
        ui->horizontalSlider_drvVeloc->setValue(ui->spinBox_drvVeloc->value());
        hev->SetDrvVeloc(ui->spinBox_drvVeloc->value()*100);
        _drvTargetVeloc = ui->spinBox_drvVeloc->value();
    }
}

void MainWindow::setDrvStrokeZero()
{
    hev->SetDrvStroke(0);
    ui->horizontalSlider_drvStroke->setValue(0);
    ui->spinBox_drvStroke->setValue(0);
}

void MainWindow::setDrvVelocZero()
{
    hev->SetDrvVeloc(0);
    ui->horizontalSlider_drvVeloc->setValue(0);
    ui->spinBox_drvVeloc->setValue(0);
}

void MainWindow::sndDrvModeM()
{
    hev->SetDrvMode(MODE_MANUAL);
    ui->horizontalSlider_drvStroke->setValue(0);
    ui->horizontalSlider_drvVeloc->setValue(0);
    _drvTargetVeloc = 0;
    _drvTargetStroke = 0;
}

void MainWindow::sndDrvModeP()
{
    hev->SetDrvMode(MODE_PROGRAM);
}

void MainWindow::sndDrvCModeS()
{
    hev->SetDrvCMode(CONT_MODE_STROKE);
}

void MainWindow::sndDrvCModeV()
{
    hev->SetDrvCMode(CONT_MODE_VELOCITY);
}

void MainWindow::sndDrvOModeON()
{
    hev->SetDrvOMode(OVERRIDE_MODE_ON);
}

void MainWindow::sndDrvOModeOFF()
{
    hev->SetDrvOMode(OVERRIDE_MODE_OFF);
}

void MainWindow::sndDrvServoON()
{
    hev->SetDrvServo(0x10);
}

void MainWindow::sndDrvServoOFF()
{
    hev->SetDrvServo(0x00);
}

void MainWindow::sndDrvShiftD()
{
    hev->SetDrvShiftMode(SHIFT_POS_D);
}

void MainWindow::sndDrvShiftN()
{
    hev->SetDrvShiftMode(SHIFT_POS_N);
}

void MainWindow::sndDrvShiftS()
{
    hev->SetDrvShiftMode(SHIFT_POS_START);
}

void MainWindow::sndDrvShiftR()
{
    hev->SetDrvShiftMode(SHIFT_POS_R);
}

void MainWindow::sndDrvShiftB()
{
    hev->SetDrvShiftMode(SHIFT_POS_B);
}

/////////////////////////////////////////////////////////////////////////////
// steer slots
/////////////////////////////////////////////////////////////////////////////
void MainWindow::setStrAngleSBox()
{
    if(ui->horizontalSlider_strAngle->value() != ui->spinBox_strAngle->value()){
        ui->spinBox_strAngle->setValue(ui->horizontalSlider_strAngle->value());
        hev->SetStrAngle(ui->spinBox_strAngle->value()*-10);
    }
}

void MainWindow::setStrTorqueSBox()
{
    if(ui->horizontalSlider_strTorque->value() != ui->spinBox_strTorque->value()){
        ui->spinBox_strTorque->setValue(ui->horizontalSlider_strTorque->value());
        hev->SetStrTorque(ui->spinBox_strTorque->value()*-1);
    }
}

void MainWindow::setStrAngleSlider()
{
    if(ui->spinBox_strAngle->value() != ui->horizontalSlider_strAngle->value()){
        ui->horizontalSlider_strAngle->setValue(ui->spinBox_strAngle->value());
        hev->SetStrAngle(ui->horizontalSlider_strAngle->value()*-10);
    }
}

void MainWindow::setStrTorqueSlider()
{
    if(ui->spinBox_strTorque->value() != ui->horizontalSlider_strTorque->value()){
        ui->horizontalSlider_strTorque->setValue(ui->spinBox_strTorque->value());
        hev->SetStrTorque(ui->horizontalSlider_strTorque->value()*-1);
    }
}

void MainWindow::setStrAngleZero()
{
    hev->SetStrAngle(0);
    ui->horizontalSlider_strAngle->setValue(0);
    ui->spinBox_strAngle->setValue(0);
}

void MainWindow::setStrTorqueZero()
{
    hev->SetStrTorque(0);
    ui->horizontalSlider_strTorque->setValue(0);
    ui->spinBox_strTorque->setValue(0);
}

void MainWindow::sndStrModeM()
{
    hev->SetStrMode(MODE_MANUAL);
}

void MainWindow::sndStrModeP()
{
    hev->SetStrMode(MODE_PROGRAM);
}

void MainWindow::sndStrCModeA()
{
    hev->SetStrCMode(CONT_MODE_ANGLE);
}

void MainWindow::sndStrCModeT()
{
    hev->SetStrCMode(CONT_MODE_TORQUE);
}

void MainWindow::sndStrOModeON()
{
    hev->SetStrOMOde(OVERRIDE_MODE_ON);
}

void MainWindow::sndStrOModeOFF()
{
    hev->SetStrOMOde(OVERRIDE_MODE_OFF);
}

void MainWindow::sndStrServoON()
{
    hev->SetStrServo(0x10);
}

void MainWindow::sndStrServoOFF()
{
    hev->SetStrServo(0x00);
}


/////////////////////////////////////////////////////////////////////////////
// brake slots
/////////////////////////////////////////////////////////////////////////////
void MainWindow::setBrkStrokeSBox()
{
    if(ui->horizontalSlider_brkStroke->value() != ui->spinBox_brkStroke->value()){
        ui->spinBox_brkStroke->setValue(ui->horizontalSlider_brkStroke->value());
        hev->SetBrakeStroke(ui->spinBox_brkStroke->value());
    }
}

void MainWindow::setBrkStrokeSlider()
{
    if(ui->spinBox_brkStroke->value() != ui->horizontalSlider_brkStroke->value()){
        ui->horizontalSlider_brkStroke->setValue(ui->spinBox_brkStroke->value());
        hev->SetBrakeStroke(ui->horizontalSlider_brkStroke->value());
    }
}

void MainWindow::setBrkStrokeZero()
{
    hev->SetBrakeStroke(0);
    ui->horizontalSlider_brkStroke->setValue(0);
    ui->spinBox_brkStroke->setValue(0);
}

/////////////////////////////////////////////////////////////////////////////
// Diag slots
/////////////////////////////////////////////////////////////////////////////
void MainWindow::SndDiagEngineReq()
{
    hev->SndDiagReq(ENGINE_ECU);
}

void MainWindow::SndDiagEngineClear()
{
    hev->SndDiagClear(ENGINE_ECU);
}

void MainWindow::SndDiagHVReq()
{
    hev->SndDiagReq(HV_ECU);
}

void MainWindow::SndDiagHVClear()
{
    hev->SndDiagClear(HV_ECU);
}

void MainWindow::SndDiagCruiseReq()
{
    hev->SndDiagReq(BRAKE_ECU);
}

void MainWindow::SndDiagBattReq()
{
    hev->SndDiagReq(BATTERY_ECU);
}

void MainWindow::SndDiagBattClear()
{
    hev->SndDiagClear(BATTERY_ECU);
}

void MainWindow::SndErrReq()
{
    hev->SndErrReq();
}

void MainWindow::GetConfStrGainKP()
{
    hev->GetConfig(CONFIG_ANGLE_CTRL_KP);
}

void MainWindow::GetConfStrGainKI()
{
    hev->GetConfig(CONFIG_ANGLE_CTRL_KI);
}

void MainWindow::GetConfStrGainKD()
{
    hev->GetConfig(CONFIG_ANGLE_CTRL_KD);
}

void MainWindow::GetConfStrDecTime()
{
    hev->GetConfig(CONFIG_ANGLE_CTRL_DEC_TIME);
}

void MainWindow::GetConfStrWindow()
{
    hev->GetConfig(CONFIG_ANGLE_CTRL_WINDOW);
}

void MainWindow::GetConfStrPunch()
{
    hev->GetConfig(CONFIG_ANGLE_CTRL_PUNCH);
}

void MainWindow::GetConfStrMaxTrq()
{
    hev->GetConfig(CONFIG_ANGLE_CTRL_OUTPUT_MAX);
}

void MainWindow::GetConfStrCCWFactor()
{
    hev->GetConfig(CONFIG_ANGLE_CTRL_CCW_FACTOR);
}

void MainWindow::GetConfVelocMaxSpeed()
{
    hev->GetConfig(CONFIG_VELOC_CTRL_MAX_SPEED);
}

void MainWindow::GetConfVelocGainKPP()
{
    hev->GetConfig(CONFIG_VELOC_CTRL_KP_P);
}

void MainWindow::GetConfVelocGainKPM()
{
    hev->GetConfig(CONFIG_VELOC_CTRL_KP_M);
}

void MainWindow::GetConfVelocWindowP()
{
    hev->GetConfig(CONFIG_VELOC_CTRL_WINDOW_P);
}

void MainWindow::GetConfVelocWindowM()
{
    hev->GetConfig(CONFIG_VELOC_CTRL_WINDOW_M);
}

void MainWindow::GetConfVelocPunchP()
{
    hev->GetConfig(CONFIG_VELOC_CTRL_PUNCH_P);
}

void MainWindow::GetConfVelocPunchM()
{
    hev->GetConfig(CONFIG_VELOC_CTRL_PUNCH_M);
}

void MainWindow::GetConfVelocMaxOutputP()
{
    hev->GetConfig(CONFIG_VELOC_CTRL_OUTPUT_MAX_P);
}

void MainWindow::GetConfVelocMaxOutputM()
{
    hev->GetConfig(CONFIG_VELOC_CTRL_OUTPUT_MAX_M);
}

void MainWindow::GetConfStrOverrideTh()
{
    hev->GetConfig(CONFIG_OVERRIDE_STEER_TH);
}

void MainWindow::GetConfAccOverrideTh()
{
    hev->GetConfig(CONFIG_OVERRIDE_ACCEL_TH);
}

void MainWindow::GetConfBrkOverrideTh()
{
    hev->GetConfig(CONFIG_OVERRIDE_BRAKE_TH);
}

void MainWindow::GetConfTestFlags()
{
    hev->GetConfig(TEST_FLAGS);
}

void MainWindow::SetConfStrGainKP()
{
    hev->SetControlGain(CONFIG_ANGLE_CTRL_KP, ui->spinBox_confStrGainKP->value());
}

void MainWindow::SetConfStrGainKI()
{
    hev->SetControlGain(CONFIG_ANGLE_CTRL_KI, ui->spinBox_confStrGainKI->value());
}

void MainWindow::SetConfStrGainKD()
{
    hev->SetControlGain(CONFIG_ANGLE_CTRL_KD, ui->spinBox_confStrGainKD->value());
}

void MainWindow::SetConfStrDecTime()
{
    hev->SetControlGain(CONFIG_ANGLE_CTRL_DEC_TIME, ui->spinBox_confStrDecTime->value());
}

void MainWindow::SetConfStrWindow()
{
    hev->SetControlGain(CONFIG_ANGLE_CTRL_WINDOW, ui->spinBox_confStrWindow->value());
}

void MainWindow::SetConfStrPunch()
{
    hev->SetControlGain(CONFIG_ANGLE_CTRL_PUNCH, ui->spinBox_confStrPunch->value());
}

void MainWindow::SetConfStrMaxTrq()
{
    hev->SetControlGain(CONFIG_ANGLE_CTRL_OUTPUT_MAX, ui->spinBox_confStrMaxTrq->value());
}

void MainWindow::SetConfStrCCWFactor()
{
    hev->SetControlGain(CONFIG_ANGLE_CTRL_CCW_FACTOR, ui->spinBox_confStrCCWFactor->value());
}

void MainWindow::SetConfVelocMaxSpeed()
{
    hev->SetControlGain(CONFIG_VELOC_CTRL_MAX_SPEED, ui->spinBox_confVelocMaxSpeed->value());
}

void MainWindow::SetConfVelocGainKPP()
{
    hev->SetControlGain(CONFIG_VELOC_CTRL_KP_P, ui->spinBox_confVelocGainKPP->value());
}

void MainWindow::SetConfVelocGainKPM()
{
    hev->SetControlGain(CONFIG_VELOC_CTRL_KP_M, ui->spinBox_confVelocGainKPM->value());
}

void MainWindow::SetConfVelocWindowP()
{
    hev->SetControlGain(CONFIG_VELOC_CTRL_WINDOW_P, ui->spinBox_confVelocWindowP->value());
}

void MainWindow::SetConfVelocWindowM()
{
    hev->SetControlGain(CONFIG_VELOC_CTRL_WINDOW_M, ui->spinBox_confVelocWindowM->value());
}

void MainWindow::SetConfVelocPunchP()
{
    hev->SetControlGain(CONFIG_VELOC_CTRL_PUNCH_P, ui->spinBox_confVelocPunchP->value());
}

void MainWindow::SetConfVelocPunchM()
{
    hev->SetControlGain(CONFIG_VELOC_CTRL_PUNCH_M, ui->spinBox_confVelocPunchM->value());
}

void MainWindow::SetConfVelocMaxOutputP()
{
    hev->SetControlGain(CONFIG_VELOC_CTRL_OUTPUT_MAX_P, ui->spinBox_confVelocMaxOutputP->value());
}

void MainWindow::SetConfVelocMaxOutputM()
{
    hev->SetControlGain(CONFIG_VELOC_CTRL_OUTPUT_MAX_M, ui->spinBox_confVelocMaxOutputM->value());
}

void MainWindow::SetConfStrOverrideTh()
{
    hev->SetControlGain(CONFIG_OVERRIDE_STEER_TH, ui->spinBox_confStrOverrideTh->value());
}

void MainWindow::SetConfAccOverrideTh()
{
    hev->SetControlGain(CONFIG_OVERRIDE_ACCEL_TH, ui->spinBox_confDrvOverrideTh->value());
}

void MainWindow::SetConfBrkOverrideTh()
{
    hev->SetControlGain(CONFIG_OVERRIDE_BRAKE_TH, ui->spinBox_confBrkOverrideTh->value());
}

void MainWindow::SetConfTestFlags()
{
    hev->SetControlGain(TEST_FLAGS, ui->spinBox_confTestFlags->value());
}

void MainWindow::SaveConfig()
{
    hev->SaveConfig();
}
void MainWindow::setGameEnable()
{
    if(ui->checkBox_gameEnable->isChecked() == true)
        _gameEnable = true;
    else
        _gameEnable = false;
}

void MainWindow::readLoop()
{
    hev->GetBattInf(&_battInf);
    hev->GetBrakeInf(&_brakeInf);
    hev->GetDrvInf(&_drvInf);
    hev->GetOtherInf(&_otherInf);
    hev->GetStrInf(&_strInf);
    hev->GetErrCode(&_errLevel, &_errCode);

    viewBattInf();
    viewBrakeInf();
    viewDrvInf();
    viewOtherInf();
    viewStrInf();
    viewErrCode();
}

void MainWindow::sndDrvTarget()
{
    if((_drvInf.mode == MODE_PROGRAM) && (_drvInf.servo == 0x10)){
        if((_drvInf.contMode == CONT_MODE_STROKE) && (_drvTargetStroke != 0)){
            hev->SetDrvStroke(_drvTargetStroke);
        } else if((_drvInf.contMode == CONT_MODE_VELOCITY) && (_drvTargetVeloc != 0)){
            hev->SetDrvVeloc(_drvTargetVeloc*100);
        }
    }
}

void MainWindow::writeLog()
{
    fprintf(_logSave,"%d/%d/%d/%d:%d:%d.%ld,",_s_time->tm_year+1900, _s_time->tm_mon+1, _s_time->tm_mday,
            _s_time->tm_hour, _s_time->tm_min, _s_time->tm_sec, _getTime.tv_usec);
    if(_selectLog.drvInf == true){
        fprintf(_logSave, "%d,%d,%d,%d,%d,%d,%d,%3.2f,%3.2f,%d,%d,%d",
                _drvInf.mode, _drvInf.contMode, _drvInf.overrideMode, _drvInf.servo,
                _drvInf.actualPedalStr, _drvInf.targetPedalStr, _drvInf.inputPedalStr,
                _drvInf.targetVeloc, _drvInf.veloc,
                _drvInf.actualShift, _drvInf.targetShift, _drvInf.inputShift);
    } else {
        fprintf(_logSave, ",,,,,,,,,,,,,,,,,,,,");
    }
    if(_selectLog.strInf == true){
        fprintf(_logSave,"%d,%d,%d,%d,%d,%d,%3.2f,%3.2f",
                _strInf.mode, _strInf.cont_mode, _strInf.overrideMode, _strInf.servo,
                _strInf.targetTorque, _strInf.torque,
                _strInf.angle, _strInf.targetAngle);
    } else {
        fprintf(_logSave,",,,,,,,,,,");
    }
    if(_selectLog.brkInf == true){
        fprintf(_logSave,"%d,%d,%d,%d,",
                _brakeInf.pressed, _brakeInf.actualPedalStr, _brakeInf.targetPedalStr, _brakeInf.inputPedalStr);
    } else {
        fprintf(_logSave,",,,,,,,,,,,");
    }
    if(_selectLog.battInf == true){
        fprintf(_logSave,"%3.2f,%d,%3.2f,%d,%d,%3.2f,%3.2f",
                _battInf.soc, _battInf.voltage, _battInf.current,
                _battInf.max_temp, _battInf.min_temp,
                _battInf.max_chg_current, _battInf.max_dischg_current);
    } else {
        fprintf(_logSave,",,,,,,");
    }
    if(_selectLog.otherInf == true){
        fprintf(_logSave,"%3.3f,%3.5f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%d,%d,%d,%3.1f,%d,%d,%d,%d,%d,%d,%d,",
                _otherInf.sideAcc, _otherInf.acc, _otherInf.angleFromP, _otherInf.brkPedalStrFromP,
                _otherInf.velocFrFromP, _otherInf.velocFlFromP, _otherInf.velocRrFromP, _otherInf.velocRlFromP,
                _otherInf.velocFromP2,
                _otherInf.drv_mode, _otherInf.drvPedalStrFromP, _otherInf.rpm,
                _otherInf.velocFlFromP,
                _otherInf.ev_mode, _otherInf.temp, _otherInf.shiftFromPrius, _otherInf.light,
                _otherInf.level, _otherInf.door, _otherInf.cluise);
    } else {
        fprintf(_logSave,",,,,,,,,,,");
    }

    fprintf(_logSave, "\n");
}

void MainWindow::updateTime()
{
    time(&_day_time);
    _s_time = gmtime(&_day_time);
//    _update = QString::number(_s_time->tm_year+1900 ,10);
    QString *date = new QString();
    *date = QString::number(_s_time->tm_year+1900, 10);
    if(_s_time->tm_mon+1 < 10)
        *date += "_0" + QString::number(_s_time->tm_mon+1 ,10);
    else if(_s_time->tm_mon < 12)
        *date += "-" + QString::number(_s_time->tm_mon+1 ,10);

    if(_s_time->tm_mday < 10)
        *date += "_0" + QString::number(_s_time->tm_mday ,10);
    else if(_s_time->tm_mday < 32)
        *date += "-" + QString::number(_s_time->tm_mday ,10);

    if(_s_time->tm_hour < 10)
        *date += "_0" + QString::number(_s_time->tm_hour ,10);
    else
        *date += "_" + QString::number(_s_time->tm_hour ,10);

    if(_s_time->tm_min < 10)
        *date += "_0" + QString::number(_s_time->tm_min ,10);
    else
        *date += "_" + QString::number(_s_time->tm_min ,10);

    if(_s_time->tm_sec < 10)
        *date += "_0" + QString::number(_s_time->tm_sec ,10);
    else
        *date += "_" + QString::number(_s_time->tm_sec ,10);

    gettimeofday(&_getTime, NULL);
    *date += "_" + QString::number(_getTime.tv_usec ,10);

    _update = *date;
//    ui->textEdit_updateTime->setText(update);
}

void MainWindow::viewBattInf()
{
    ui->textEdit_battSOC->setText(QString::number(_battInf.soc, 'f', 1));
    ui->textEdit_battVolt->setText(QString::number(_battInf.voltage, 10));
    ui->textEdit_battCurrent->setText(QString::number(_battInf.current, 'f', 1));
    ui->textEdit_battMaxTemp->setText(QString::number(_battInf.max_temp, 10));
    ui->textEdit_battMinTemp->setText(QString::number(_battInf.min_temp, 10));
//    ui->textEdit_battMaxChg->setText(QString::number(_battInf.max_chg_current, 'f', 1));
//    ui->textEdit_battMinChg->setText(QString::number(_battInf.max_dischg_current, 'f', 1));
}

void MainWindow::viewBrakeInf()
{
    if(_brakeInf.pressed == true){
        ui->textEdit_brkPressState->setText(QString("ON"));
    }else
        ui->textEdit_brkPressState->setText(QString("OFF"));

    ui->textEdit_brkStrokeInput->setText(QString::number(_brakeInf.inputPedalStr, 10));
    ui->textEdit_brkStrokeTarget->setText(QString::number(_brakeInf.targetPedalStr, 10));
    ui->textEdit_brkStrokeActual->setText(QString::number(_brakeInf.actualPedalStr, 10));

//    if((_drvInf.contMode == CONT_MODE_VELOCITY) && (_drvInf.mode == MODE_PROGRAM) && (_drvInf.servo == 0x10)){
//        if((_brakeInf.targetPedalStr != 2000) && (_brakeInf.targetPedalStr >= 100)){
//            _gameVeloc = 0;
//        }
//    }

//    ui->textEdit_brkStrokeRawSKS1->setText(QString::number(_brakeInf.sks1, 'f', 4));
//    ui->textEdit_brkStrokeRawSKS2->setText(QString::number(_brakeInf.sks2, 'f', 4));
//    ui->textEdit_brkStrokeRawPMC1->setText(QString::number(_brakeInf.pmc1, 'f', 4));
//    ui->textEdit_brkStrokeRawPMC2->setText(QString::number(_brakeInf.pmc2, 'f', 4));
//    ui->textEdit_brkRawPFL->setText(QString::number(_brakeInf.pfl, 'f', 4));
//    ui->textEdit_brkRawPFR->setText(QString::number(_brakeInf.pfr, 'f', 4));
//    ui->textEdit_brkRawPRL->setText(QString::number(_brakeInf.prl, 'f', 4));
//    ui->textEdit_brkRawPRR->setText(QString::number(_brakeInf.prr, 'f', 4));
}

void MainWindow::viewDrvInf()
{
    if(_drvInf.contMode == CONT_MODE_VELOCITY)
        ui->textEdit_drvCntMode->setText(QString("velocity"));
    else
        ui->textEdit_drvCntMode->setText(QString("stroke"));
    if(_drvInf.mode == MODE_MANUAL){
        ui->textEdit_drvMode->setText(QString("manual"));
        _drvTargetVeloc = 0;
        _drvTargetStroke = 0;
        ui->spinBox_drvVeloc->setValue(0);
        ui->horizontalSlider_drvVeloc->setValue(0);
    }else if(_drvInf.mode == MODE_PROGRAM){
        ui->textEdit_drvMode->setText(QString("program"));
    } else {
        ui->textEdit_drvMode->setText(QString("emergency"));
    }

    if(_drvInf.overrideMode == OVERRIDE_MODE_ON)
        ui->textEdit_drvOverMode->setText(QString("ON"));
    else
        ui->textEdit_drvOverMode->setText(QString("OFF"));

    if(_drvInf.servo == 0x00)
        ui->textEdit_drvServo->setText(QString("OFF"));
    else
        ui->textEdit_drvServo->setText(QString("ON"));

    ui->textEdit_drvVelocTarget->setText(QString::number(_drvInf.targetVeloc, 'f', 4));
    ui->textEdit_drvVelocActual->setText(QString::number(_drvInf.veloc, 'f', 4));

    ui->textEdit_drvStrokeInput->setText(QString::number(_drvInf.inputPedalStr, 10));
    ui->textEdit_drvStrokeTarget->setText(QString::number(_drvInf.targetPedalStr, 10));
    ui->textEdit_drvStrokeActual->setText(QString::number(_drvInf.actualPedalStr, 10));
//    ui->textEdit_drvStrokeRawVpa1->setText(QString::number(_drvInf.vpa1, 'f', 4));
//    ui->textEdit_drvStrokeRawVpa2->setText(QString::number(_drvInf.vpa2, 'f', 4));

    QString input;
    QString target;
    QString actual;

    switch(_drvInf.inputShift){
    case SHIFT_POS_B: input = "B"; break;
    case SHIFT_POS_D: input = 'D'; break;
    case SHIFT_POS_N: input = 'N'; break;
    case SHIFT_POS_R: input = 'R'; break;
    case SHIFT_POS_P: input = 'P'; break;
    case SHIFT_POS_START: input = 'S'; break;
    case SHIFT_POS_UNKNOWN: input = 'U'; break;
    default: input = 'U'; break;
    }

    switch(_drvInf.targetShift){
    case SHIFT_POS_B: target = "B"; break;
    case SHIFT_POS_D: target = 'D'; break;
    case SHIFT_POS_N: target = 'N'; break;
    case SHIFT_POS_R: target = 'R'; break;
    case SHIFT_POS_P: target = 'P'; break;
    case SHIFT_POS_START: target = 'S'; break;
    case SHIFT_POS_UNKNOWN: target = 'U'; break;
    default: target = 'U'; break;
    }

    switch(_drvInf.actualShift){
    case SHIFT_POS_B: actual = 'B'; break;
    case SHIFT_POS_D: actual = 'D'; break;
    case SHIFT_POS_N: actual = 'N'; break;
    case SHIFT_POS_R: actual = 'R'; break;
    case SHIFT_POS_P: actual = 'P'; break;
    case SHIFT_POS_START: actual = 'S'; break;
    case SHIFT_POS_UNKNOWN: actual = 'U'; break;
    default: actual = 'U'; break;
    }
    ui->textEdit_drvShiftInput->setText(input);
    ui->textEdit_drvShiftTarget->setText(target);
    ui->textEdit_drvShiftActual->setText(actual);
//    ui->textEdit_drvShiftRawVSX1->setText(QString::number(_drvInf.shiftRawVsx1, 10));
//    ui->textEdit_drvShiftRawVSX2->setText(QString::number(_drvInf.shiftRawVsx2, 10));
//    ui->textEdit_drvShiftRawVSX3->setText(QString::number(_drvInf.shiftRawVsx3, 10));
//    ui->textEdit_drvShiftRawVSX4->setText(QString::number(_drvInf.shiftRawVsx4, 10));

    ui->textEdit_gameButton->setText(QString::number(_gameData.button, 10));
    ui->textEdit_gameStr->setText(QString::number(_gameData.angle, 'f', 1));
    ui->textEdit_gameDrv->setText(QString::number(_gameData.drive, 10));
    ui->textEdit_gameBrk->setText(QString::number(_gameData.brake, 10));
}

void MainWindow::viewOtherInf()
{
    ui->textEdit_otherDoor->setText(QString("0x") + QString::number(_otherInf.door,16));
    ui->textEdit_otherLevel->setText(QString::number(_otherInf.level, 10));
    QString light = "";
    switch(_otherInf.light){
    case LIGHT_OFF: light = "OFF"; break;
    case LIGHT_PARK: light = "PARK"; break;
    case LIGHT_ON: light = "ON"; break;
    case LIGHT_HIGH: light = "HIGH"; break;
    default: light = "unknown";break;
    }
    ui->textEdit_otherLight->setText(light);
    QString dmode;
    switch(_otherInf.drv_mode){
    case 0: dmode = 'P'; break;
    case 1: dmode = 'R'; break;
    case 2: dmode = 'N'; break;
    case 3: dmode = 'D'; break;
    case 4: dmode = 'B'; break;
    default: dmode = 'S'; break;
    }
    ui->textEdit_otherDriveMode->setText(dmode);

    ui->textEdit_otherEvMode->setText(QString::number(_otherInf.ev_mode, 10));

    if(_otherInf.cluise == true)
        ui->textEdit_otherCluise->setText(QString("ON"));
    else
        ui->textEdit_otherCluise->setText(QString("OFF"));

    ui->textEdit_otherRpm->setText(QString::number(_otherInf.rpm, 10));
    ui->textEdit_otherCoolant->setText(QString::number(_otherInf.temp, 10));
//    ui->textEdit_otherAcc->setText(QString::number(_otherInf.acc * 0.0026,'f', 4));
//    ui->textEdit_otherSideAcc->setText(QString::number(_otherInf.sideAcc * 0.24,'f', 4));
    ui->textEdit_otherAcc->setText(QString::number(_otherInf.acc,'f', 4));
    ui->textEdit_otherSideAcc->setText(QString::number(_otherInf.sideAcc,'f', 4));

    QString *diag = new QString();
    *diag = QString::number(_otherInf.dtcData1, 16);
    *diag += " ";
    *diag += QString::number(_otherInf.dtcData2, 16);
    *diag += " ";
    *diag += QString::number(_otherInf.dtcData3, 16);
    *diag += " ";
    *diag += QString::number(_otherInf.dtcData4, 16);
    *diag += " ";
    *diag += QString::number(_otherInf.dtcData5, 16);
    *diag += " ";
    *diag += QString::number(_otherInf.dtcData6, 16);
    *diag += " ";
    *diag += QString::number(_otherInf.dtcData7, 16);
    *diag += " ";
    *diag += QString::number(_otherInf.dtcData8, 16);
    *diag += " ";
    ui->textEdit_diagState->setText(*diag);

    ui->textEdit_obdAngle->setText(QString::number(_otherInf.angleFromP, 'f', 1));

    QString shift;
    switch(_otherInf.shiftFromPrius){
    case 0: shift = 'B'; break;
    case 16: shift = 'D'; break;
    case 32: shift = 'N'; break;
    case 64: shift = 'R'; break;
    default: shift = 'S'; break;
    }
    ui->textEdit_obdShift->setText(shift);

    ui->textEdit_obdDrivePedal->setText(QString::number(_otherInf.drvPedalStrFromP, 'f',3));
    ui->textEdit_obdBrakePedal->setText(QString::number(_otherInf.brkPedalStrFromP, 'f',3));
    ui->textEdit_obdVelocFL->setText(QString::number(_otherInf.velocFlFromP, 'f', 2));
    ui->textEdit_obdVelocFR->setText(QString::number(_otherInf.velocFrFromP, 'f', 2));
    ui->textEdit_obdVelocRL->setText(QString::number(_otherInf.velocRlFromP, 'f', 2));
    ui->textEdit_obdVelocRR->setText(QString::number(_otherInf.velocRrFromP, 'f', 2));
    ui->textEdit_obdVeloc->setText(QString::number((int)_otherInf.velocFromP, 10));
    ui->textEdit_obdVeloc2->setText(QString::number((int)_otherInf.velocFromP2,10));
}

void MainWindow::viewStrInf()
{
    if(_strInf.mode == MODE_MANUAL)
        ui->textEdit_strMode->setText(QString("manual"));
    else if(_strInf.mode == MODE_PROGRAM){
        ui->textEdit_strMode->setText(QString("program"));
    } else{
        ui->textEdit_strMode->setText(QString("emergency"));
    }
    if(_strInf.cont_mode == CONT_MODE_TORQUE)
        ui->textEdit_strContMode->setText(QString("torque"));
    else
        ui->textEdit_strContMode->setText(QString("angle"));
    if(_strInf.overrideMode == OVERRIDE_MODE_ON)
        ui->textEdit_strOverMode->setText(QString("ON"));
    else
        ui->textEdit_strOverMode->setText(QString("OFF"));
    if(_strInf.servo == 0x00)
        ui->textEdit_strServo->setText(QString("OFF"));
    else
        ui->textEdit_strServo->setText(QString("ON"));

    ui->textEdit_strTorqueInput->setText(QString::number(_strInf.targetTorque, 10));
    ui->textEdit_strTorqueOutput->setText(QString::number(_strInf.torque, 10));
//    ui->textEdit_strTorqueRawTrq1->setText(QString::number(_strInf.trq1, 'f', 4));
//    ui->textEdit_strTorqueRawTrq2->setText(QString::number(_strInf.trq2, 'f', 4));

    ui->textEdit_strAngleTarget->setText(QString::number(_strInf.targetAngle, 'f', 1));
    ui->textEdit_strAngleActual->setText(QString::number(_strInf.angle, 'f', 1));
}

void MainWindow::viewErrCode()
{
    ui->textEdit_error_level->setText(QString::number(_errLevel, 10));
    ui->textEdit_error_code->setText(QString("0x") + QString::number(_errCode, 16));
}

void MainWindow::OnGameJsReceive()
{
    char buf[8];
    memset(buf, 0, 8);
    _Game.GetGameJsData(buf);
    short val = (buf[5] << 8) + buf[4];
    float strVal = 6660.0/32767.0;
    float strokeVal = 4096.0/65534.0;
    float sndAngle;
    float sndDrvStroke;
    float sndBrkStroke;

    if(_gameEnable == true){
        if(buf[6] == 0x01){
//            int buttonVal = buf[7];
//            ui->textEdit_gameButton->setText(QString::number((int)buf[7], 10));
            _gameData.button = buf[7];
            switch(buf[7]){
                case 0: // X Button
                    printf("SetDrvShiftMode(D)\n");
                    hev->SetDrvShiftMode(SHIFT_POS_D);
                     break;
                case 1: // □ Button
                    printf("SetDrvShiftMode(N)\n");
                    hev->SetDrvShiftMode(SHIFT_POS_N);
                    break;
                case 2: // ○ Button
                    printf("SetDrvShiftMode(R)\n");
                    hev->SetDrvShiftMode(SHIFT_POS_R);
                    break;
                case 3: // △ Button
                    printf("SetDrvShiftMode(B)\n");
                    hev->SetDrvShiftMode(SHIFT_POS_B);
                    break;
                case 4: // ハンドル裏ボタン右
                    break;
                case 5: // ハンドル裏ボタン左
                    break;
                case 6: // ハンドル上ボタン右
                    break;
                case 7: // ハンドル上ボタン左
                    break;
                case 8: // SELECT Button
                    printf("Pusshu SELECT\n");
                    hev->SetDrvMode(MODE_MANUAL);
                    hev->SetDrvCMode(CONT_MODE_STROKE);
                    hev->SetDrvOMode(OVERRIDE_MODE_ON);
                    hev->SetDrvServo(0x00);
                    hev->SetStrMode(MODE_MANUAL);
                    hev->SetStrCMode(CONT_MODE_ANGLE);
                    hev->SetStrOMOde(OVERRIDE_MODE_ON);
                    hev->SetStrServo(0x00);
                    break;
                case 9: // START Button
                    printf("Pusshu START\n");
                    hev->SetDrvMode(MODE_PROGRAM);
                    hev->SetDrvMode(CONT_MODE_STROKE);
                    hev->SetDrvOMode(OVERRIDE_MODE_OFF);
                    hev->SetDrvServo(0x10);
                    hev->SetStrMode(MODE_PROGRAM);
                    hev->SetStrCMode(CONT_MODE_ANGLE);
                    hev->SetStrOMOde(OVERRIDE_MODE_OFF);
                    hev->SetStrServo(0x10);
                    break;
                case 10: // R3 Button
                    break;
                case 11: // L3 Button
                    break;
                case 12: // シフトレバー手前
                    break;
                case 13: // シフトレバー奥
                    break;
                case 14: // 赤ボタンPusshu
                    break;
                case 15: // ＋ボタン
                    break;
                case 16: // 赤ボタン時計回り
                    break;
                case 17: // 赤ボタン反時計回り
                    break;
                case 18: // −ボタン
                    break;
                case 19: // 中央大ボタン
                    break;
                case 20: // PSボタン
                    break;
                default:
                    break;
            }
        } else if(buf[6] == 0x02){
            switch(buf[7]){
            case 0: // ハンドル
                sndAngle = val * strVal;
                sndAngle *= -1;
//                printf("SetStrAngle(%d)", (int)sndAngle );
//                ui->textEdit_gameStr->setText(QString::number(sndAngle, 'f', 1));
                if((_gameData.angle - sndAngle) < 3000){
                    _gameData.angle = sndAngle;
                    if((sndAngle <= 6660) && (sndAngle >= -6660)){
                        hev->SetStrAngle((int)sndAngle);
                    }
                }
                break;
            case 1: // アクセル
//                printf("val=%d -0x7fff=%d -0x7fff*-1=%d\n",
//                        val, val-0x7fff, (val-0x7fff)*-1);
    //          val -= 0x7fff;
    //          val *= -1;
                sndDrvStroke = (val-0x7fff)*-1 * strokeVal;
//                printf("SetDrvStroke(%d)\n", (int)sndDrvStroke);
//                ui->textEdit_gameDrv->setText(QString::number(sndDrvStroke));
                _gameData.drive = sndDrvStroke;
                hev->SetDrvStroke((int)sndDrvStroke);
                break;
            case 2: // ブレーキ
    //          val -= 0x7fff;
    //          val *= -1;
                sndBrkStroke = (val - 0x7fff)*-1 * strokeVal;
//                printf("SetBrakeStroke(%d)\n", (int)sndBrkStroke);
//                ui->textEdit_gameBrk->setText(QString::number(sndBrkStroke));
                _gameData.brake = sndBrkStroke;
                hev->SetBrakeStroke((int)sndBrkStroke);
                if(sndBrkStroke >= 200){
                    _drvTargetVeloc = 0;
                    _drvTargetStroke = 0;
                    hev->SetDrvVeloc(0);
                }
                break;
            case 3: // 十字ボタン左右
                break;
            case 4: // 十字ボタン上下
                if((unsigned short)val == 0x8001){
                    _drvTargetVeloc +=1;
                } else if((unsigned short)val == 0x7eff){
                    _drvTargetVeloc -=1;
                }
//                ui->spinBox_drvVeloc->setValue(_gameVeloc);
                hev->SetDrvVeloc(_drvTargetVeloc*100);
                break;
            default: break;
            }
        }
    }

/*
// Microsoft XBox Controller
// type 1 = Push Button, 2 = analog input
// Push Button
//    0=A, 1=B, 2=X , 3=Y , 4=LB , 5=RB, 6=BACK, 7=START, 8=中央, 9=左スティック, a=右スティック
// Analog input
//    0=左左右, 1=左上下, 2=LT 3=右左右, 4=右上下, 5=RT, 6=十字左右, 7=十字上下
*/
}


void MainWindow::OnGameHidReceive()
{
    char buf[8];
    memset(buf, 0, 8);
    _Game.GetGameHidData(buf);
    short val = (buf[5] << 8) + buf[4];
    val -= 8192.0;
    if(val <= -257)
        val += 256;

    float strVal = 6000.0/8192.0;
    float  sndAngle = 0.0f;

     if(_gameEnable == true){
        sndAngle = val * strVal;
        sndAngle *= -1;
        printf("SetStrAngle(%d)", (int)sndAngle );
        _gameData.angle = sndAngle;
        if((sndAngle <= 6000) && (sndAngle >= -6000)){
            hev->SetStrAngle((int)sndAngle);
        }
    }
}


void MainWindow::UpdateConfig(int num, int index, int data[])
{
    for(int i=0; i<num; i++){
        _config.data[index-100] = data[i];
    }

    switch(index){
    case CONFIG_ANGLE_CTRL_KP:
        ui->spinBox_confStrGainKP->setValue(_config.data[0]);
        if(num >=2)
            ui->spinBox_confStrGainKI->setValue(_config.data[1]);
        if(num >= 3)
            ui->spinBox_confStrGainKD->setValue(_config.data[2]);
        break;
    case CONFIG_ANGLE_CTRL_KI:
        ui->spinBox_confStrGainKI->setValue(_config.data[1]);
        if(num >=2)
            ui->spinBox_confStrGainKD->setValue(_config.data[2]);
        if(num >= 3)
            ui->spinBox_confStrDecTime->setValue(_config.data[3]);
        break;
    case CONFIG_ANGLE_CTRL_KD:
        ui->spinBox_confStrGainKD->setValue(_config.data[2]);
        if(num >=2)
            ui->spinBox_confStrDecTime->setValue(_config.data[3]);
        if(num >= 3)
            ui->spinBox_confStrWindow->setValue(_config.data[4]);
        break;
    case CONFIG_ANGLE_CTRL_DEC_TIME:
        ui->spinBox_confStrDecTime->setValue(_config.data[3]);
        if(num >=2)
            ui->spinBox_confStrWindow->setValue(_config.data[4]);
        if(num >= 3)
            ui->spinBox_confStrPunch->setValue(_config.data[5]);
        break;
    case CONFIG_ANGLE_CTRL_WINDOW:
        ui->spinBox_confStrWindow->setValue(_config.data[4]);
        if(num >=2)
            ui->spinBox_confStrPunch->setValue(_config.data[5]);
        if(num >= 3)
            ui->spinBox_confStrMaxTrq->setValue(_config.data[6]);
        break;
    case CONFIG_ANGLE_CTRL_PUNCH:
        ui->spinBox_confStrPunch->setValue(_config.data[5]);
        if(num >=2)
            ui->spinBox_confStrMaxTrq->setValue(_config.data[6]);
        if(num >= 3)
            ui->spinBox_confStrCCWFactor->setValue(_config.data[7]);
        break;
    case CONFIG_ANGLE_CTRL_OUTPUT_MAX:
        ui->spinBox_confStrMaxTrq->setValue(_config.data[6]);
        if(num >=2)
            ui->spinBox_confStrCCWFactor->setValue(_config.data[7]);
        if(num >= 3)
            ui->spinBox_confVelocMaxSpeed->setValue(_config.data[8]);
        break;
    case CONFIG_ANGLE_CTRL_CCW_FACTOR:
        ui->spinBox_confStrCCWFactor->setValue(_config.data[7]);
        if(num >=2)
            ui->spinBox_confVelocMaxSpeed->setValue(_config.data[8]);
        if(num >= 3)
            ui->spinBox_confVelocGainKPP->setValue(_config.data[9]);
        break;
    case CONFIG_VELOC_CTRL_MAX_SPEED:
        ui->spinBox_confVelocMaxSpeed->setValue(_config.data[8]);
        if(num >=2)
            ui->spinBox_confVelocGainKPP->setValue(_config.data[9]);
        if(num >= 3)
            ui->spinBox_confVelocGainKPM->setValue(_config.data[10]);
        break;
    case CONFIG_VELOC_CTRL_KP_P:
        ui->spinBox_confVelocGainKPP->setValue(_config.data[9]);
        if(num >=2)
            ui->spinBox_confVelocGainKPM->setValue(_config.data[10]);
        if(num >= 3)
            ui->spinBox_confVelocWindowP->setValue(_config.data[11]);
        break;
    case CONFIG_VELOC_CTRL_KP_M:
        ui->spinBox_confVelocGainKPM->setValue(_config.data[10]);
        if(num >=2)
            ui->spinBox_confVelocWindowP->setValue(_config.data[11]);
        if(num >= 3)
            ui->spinBox_confVelocWindowM->setValue(_config.data[12]);
        break;
    case CONFIG_VELOC_CTRL_WINDOW_P:
        ui->spinBox_confVelocWindowP->setValue(_config.data[11]);
        if(num >=2)
            ui->spinBox_confVelocWindowM->setValue(_config.data[12]);
        if(num >= 3)
            ui->spinBox_confVelocPunchP->setValue(_config.data[13]);
        break;
    case CONFIG_VELOC_CTRL_WINDOW_M:
        ui->spinBox_confVelocWindowM->setValue(_config.data[12]);
        if(num >=2)
            ui->spinBox_confVelocPunchP->setValue(_config.data[13]);
        if(num >= 3)
            ui->spinBox_confVelocPunchM->setValue(_config.data[14]);
        break;
    case CONFIG_VELOC_CTRL_PUNCH_P:
        ui->spinBox_confVelocPunchP->setValue(_config.data[13]);
        if(num >=2)
            ui->spinBox_confVelocPunchM->setValue(_config.data[14]);
        if(num >= 3)
            ui->spinBox_confVelocMaxOutputP->setValue(_config.data[15]);
        break;
    case CONFIG_VELOC_CTRL_PUNCH_M:
        ui->spinBox_confVelocPunchM->setValue(_config.data[14]);
        if(num >=2)
            ui->spinBox_confVelocMaxOutputP->setValue(_config.data[15]);
        if(num >= 3)
            ui->spinBox_confVelocMaxOutputM->setValue(_config.data[16]);
        break;
    case CONFIG_VELOC_CTRL_OUTPUT_MAX_P:
        ui->spinBox_confVelocMaxOutputP->setValue(_config.data[15]);
        if(num >=2)
            ui->spinBox_confVelocMaxOutputM->setValue(_config.data[16]);
        if(num >= 3)
            ui->spinBox_confStrOverrideTh->setValue(_config.data[17]);
        break;
    case CONFIG_VELOC_CTRL_OUTPUT_MAX_M:
        ui->spinBox_confVelocMaxOutputM->setValue(_config.data[16]);
        if(num >=2)
            ui->spinBox_confStrOverrideTh->setValue(_config.data[17]);
        if(num >= 3)
            ui->spinBox_confDrvOverrideTh->setValue(_config.data[18]);
        break;
    case CONFIG_OVERRIDE_STEER_TH:
        ui->spinBox_confStrOverrideTh->setValue(_config.data[17]);
        if(num >=2)
            ui->spinBox_confDrvOverrideTh->setValue(_config.data[18]);
        if(num >= 3)
            ui->spinBox_confBrkOverrideTh->setValue(_config.data[19]);
        break;
    case CONFIG_OVERRIDE_ACCEL_TH:
        ui->spinBox_confDrvOverrideTh->setValue(_config.data[18]);
        if(num >=2)
            ui->spinBox_confBrkOverrideTh->setValue(_config.data[19]);
        if(num >= 3)
            ui->spinBox_confTestFlags->setValue(_config.data[20]);
        break;
    case CONFIG_OVERRIDE_BRAKE_TH:
        ui->spinBox_confBrkOverrideTh->setValue(_config.data[19]);
        if(num >=2)
            ui->spinBox_confTestFlags->setValue(_config.data[20]);
        break;
    case TEST_FLAGS:
        ui->spinBox_confTestFlags->setValue(_config.data[20]);
        break;
    default:
        break;
    }
}
