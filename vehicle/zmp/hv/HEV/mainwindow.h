#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtGui/QMainWindow>
#include <QtGui/QGraphicsView>
#include <QtGui/QPixmap>
#include <QtGui/QGraphicsPixmapItem>
#include <QtGui/QFileDialog>
#include <QTimer>
#include <QTime>

//#include "OtToolBar.h"

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include "GameControl.h"
#include "HevCnt.h"
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include <string>
#include <sys/time.h>
#include <fstream>
//#include <iostream>

#include "../autoware_socket/data.h"

namespace Ui {
class MainWindow;
}

/*struct selectLogInf {
    bool start;
    bool drvInf;
    bool strInf;
    bool brkInf;
    bool battInf;
    bool otherInf;
    bool cameraInf;
};*/

class MainWindow : public QMainWindow, GameReceiveHandler, ChangeConfig
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    bool GameInit();
    bool GameStart();

   /*===========HevBase用関数===================*/ 

    void UpdateState();//hevの状態を更新
    void ChangeShiftMode(double cmd_velocity); //シフトチェック
    void SteeringControl(double cmd_steering_angle);
    void AccelerateControl(double current_velocity,double cmd_velocity); //自動運転：加速
    void DecelerateControl(double current_velocity,double cmd_velocity); //自動運転：減速
    void StoppingControl(double current_velocity,double cmd_velocity); //自動運転：停止
    bool Brake(int target_brake, int gain); //ブレーキの関数
    bool Accel(int target_accel, int gain); //アクセルの関数

    /*============ここまで=================*/

protected:
    void changeEvent(QEvent *);

public slots:
//    void grab();
//    void load();
//    void save();
//    void startTimer();
//    void stopTimer();
//    void zoomIn();
//    void zoomOut();

private slots:
    // drive slots
    void setDrvStrokeSBox();
    void setDrvVelocSBox();
    void setDrvStrokeSlider();
    void setDrvVelocSlider();
    void setDrvVelocZero();
    void setDrvStrokeZero();
    void sndDrvModeM();
    void sndDrvModeP();
    void sndDrvCModeS();
    void sndDrvCModeV();
    void sndDrvOModeON();
    void sndDrvOModeOFF();
    void sndDrvServoON();
    void sndDrvServoOFF();
    void sndDrvShiftD();
    void sndDrvShiftN();
//    void sndDrvShiftP();
    void sndDrvShiftR();
    void sndDrvShiftB();
    void sndDrvShiftS();

    // steer slots
    void setStrAngleSBox();
    void setStrTorqueSBox();
    void setStrAngleSlider();
    void setStrTorqueSlider();
    void setStrTorqueZero();
    void setStrAngleZero();
    void sndStrModeM();
    void sndStrModeP();
    void sndStrCModeA();
    void sndStrCModeT();
    void sndStrOModeON();
    void sndStrOModeOFF();
    void sndStrServoON();
    void sndStrServoOFF();

    // brake slots
    void setBrkStrokeSBox();
    void setBrkStrokeSlider();
    void setBrkStrokeZero();

    // Diag slots
    void SndDiagEngineReq();
    void SndDiagEngineClear();
    void SndDiagHVReq();
    void SndDiagHVClear();
    void SndDiagCruiseReq();
    void SndDiagBattReq();
    void SndDiagBattClear();
    void SndErrReq();

    void GetConfStrGainKP();
    void GetConfStrGainKI();
    void GetConfStrGainKD();
    void GetConfStrDecTime();
    void GetConfStrWindow();
    void GetConfStrPunch();
    void GetConfStrMaxTrq();
    void GetConfStrCCWFactor();
    void GetConfVelocMaxSpeed();
    void GetConfVelocGainKPP();
    void GetConfVelocGainKPM();
    void GetConfVelocWindowP();
    void GetConfVelocWindowM();
    void GetConfVelocPunchP();
    void GetConfVelocPunchM();
    void GetConfVelocMaxOutputP();
    void GetConfVelocMaxOutputM();
    void GetConfStrOverrideTh();
    void GetConfAccOverrideTh();
    void GetConfBrkOverrideTh();
    void GetConfTestFlags();

    void SetConfStrGainKP();
    void SetConfStrGainKI();
    void SetConfStrGainKD();
    void SetConfStrDecTime();
    void SetConfStrWindow();
    void SetConfStrPunch();
    void SetConfStrMaxTrq();
    void SetConfStrCCWFactor();
    void SetConfVelocMaxSpeed();
    void SetConfVelocGainKPP();
    void SetConfVelocGainKPM();
    void SetConfVelocWindowP();
    void SetConfVelocWindowM();
    void SetConfVelocPunchP();
    void SetConfVelocPunchM();
    void SetConfVelocMaxOutputP();
    void SetConfVelocMaxOutputM();
    void SetConfStrOverrideTh();
    void SetConfAccOverrideTh();
    void SetConfBrkOverrideTh();
    void SetConfTestFlags();

    void SaveConfig();

    // Timer loop
    void readLoop();
    void sndDrvTarget();

    // Camera slots
//    void startVideo();
//    void stopVideo();
//    void takeCapture();
//    void saveCapture();
//    void loadCapture();

//    void startVideo2();
//    void stopVideo2();
//    void takeCapture2();
    // Log slots
//    void setSaveLogFile();
    void startLog();
    void stopLog();

    // Game slots
    void setGameEnable();

 
private:

    /*===========HevBase用関数===================*/ 

    void HevBaseActivate();
    static void* HevBaseThreadEntry(void* arg);

    /*============ここまで=================*/

    void viewBattInf();
    void viewBrakeInf();
    void viewOtherInf();
    void viewDrvInf();
    void viewStrInf();
    void viewErrCode();
    void updateTime();
    void writeLog();

    void logThread();
    static void* LogThreadEntry(void* arg);

    void OnGameJsReceive();
    void OnGameHidReceive();

    void UpdateConfig(int num, int index, int data[]);

    Ui::MainWindow *ui;

    QImage image;
    QGraphicsPixmapItem* pixmapItem;
    QGraphicsScene scene;
    QTimer timer;
//    double captureInterval;
    double scaleValue;
    CvCapture* capture;

//    OtToolBar* getToolBar( QWidget* parent);
//    void showQImage(QImage* qimage);
//    void scaleXY(double d);
//    QImage* grab2();
//    QImage* iplToQ(IplImage* image);

    QTimer *readTimer;
    QTimer *drvTimer;

    HevCnt* hev;
    BattInf _battInf;
    BrakeInf _brakeInf;
    OtherInf _otherInf;
    DrvInf _drvInf;
    StrInf _strInf;
    ConfigInf _config;
    int _errCode;
    int _errLevel;
    GameData _gameData;

    struct tm *_s_time;
    time_t _day_time;
    QString _update;
    timeval _getTime;
//    bool _videoSave;
//    cv::VideoCapture *_cap;
//    cv::Size *_cap_size;
//    CvVideoWriter *_writer;
//    cv::Mat *_frame;



    QString _logFileName;
    int _tmp[100];
    struct selectLogInf _selectLog;
    FILE* _logSave;

    pthread_t _logThread;

    GameController _Game;
    bool _gameRes;
    bool _gameEnable;


    int _drvTargetVeloc;
    int _drvTargetStroke;

//function of autoware socket
    int canduration;
    int cmdduration;
    pthread_t _cmdgetter;

    void sendDataGetAndSend();
    bool setConfig();
    void wrapSender();
    static void* CMDGetterEntry(void *a);

};


#endif // MAINWINDOW_H
