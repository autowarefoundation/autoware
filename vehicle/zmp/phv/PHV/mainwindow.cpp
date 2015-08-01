#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;
using namespace zmp::hev;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    _selectLog.start = false;

    phv = new PhvCnt();
    phv->Init();
    phv->Start();

    readTimer = new QTimer(this);
    drvTimer = new QTimer(this);

    ui->setupUi(this);
    pixmapItem = NULL;
    scaleValue      = 1.0;
    _drvTargetVeloc = 0;
    _drvTargetStroke = 0;

    for(int i=0; i<21; i++)
    {
        _config.data[i] = 0;
    }

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
    connect(ui->pushButton_brkLamp_ON, SIGNAL(clicked()), this, SLOT(sndBrkLampON()));
    connect(ui->pushButton_brkLamp_OFF, SIGNAL(clicked()), this, SLOT(sndBrkLampOFF()));
    connect(ui->pushButton_brkLeftBlinker_ON, SIGNAL(clicked()), this, SLOT(sndLeftBlinkerON()));
    connect(ui->pushButton_brkLeftBlinker_OFF, SIGNAL(clicked()), this, SLOT(sndLeftBlinkerOFF()));
    connect(ui->pushButton_brkRightBlinker_ON, SIGNAL(clicked()), this, SLOT(sndRightBlinkerON()));
    connect(ui->pushButton_brkRightBlinker_OFF, SIGNAL(clicked()), this, SLOT(sndRightBlinkerOFF()));
    connect(ui->pushButton_brkAutoMode_ON, SIGNAL(clicked()), this, SLOT(sndBrkAutoModeON()));
    connect(ui->pushButton_brkAutoMode_OFF, SIGNAL(clicked()), this, SLOT(sndBrkAutoModeOFF()));

/*    connect(ui->pushButton_diagEngineGet, SIGNAL(clicked()), this, SLOT(SndDiagEngineReq()));
    connect(ui->pushButton_diagEngineClear, SIGNAL(clicked()), this, SLOT(SndDiagEngineClear()));
    connect(ui->pushButton_diagHVGet, SIGNAL(clicked()), this, SLOT(SndDiagHVReq()));
    connect(ui->pushButton_diagHVClear, SIGNAL(clicked()), this, SLOT(SndDiagHVClear()));
    connect(ui->pushButton_diagCruiseGet, SIGNAL(clicked()), this, SLOT(SndDiagCruiseReq()));
    connect(ui->pushButton_diagBattGet, SIGNAL(clicked()), this, SLOT(SndDiagBattReq()));
    connect(ui->pushButton_diagBattClear, SIGNAL(clicked()), this, SLOT(SndDiagBattClear()));
*/
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
    ui->textEdit_drvStrokeTarget->setFont(font);
    ui->textEdit_drvVelocActual->setFont(font);
    ui->textEdit_drvVelocTarget->setFont(font);

    ui->textEdit_strAngleActual->setFont(font);
    ui->textEdit_strAngleTarget->setFont(font);
    ui->textEdit_strContMode->setFont(font);
    ui->textEdit_strMode->setFont(font);
    ui->textEdit_strOverMode->setFont(font);
    ui->textEdit_strServo->setFont(font);

    ui->textEdit_brkPressState->setFont(font);
    ui->textEdit_brkStrokeActual->setFont(font);
    ui->textEdit_brkStrokeTarget->setFont(font);


    ui->pushButton_startLog->setEnabled(true);
    ui->pushButton_stopLog->setEnabled(false);

    /////////////////////////////////////////////////////////
    // Autoware extension
    if (!ConfigSocket()) {
      printf("Error: failed to configure Autoware socket!\n");    
    }
    pthread_create(&_cmdgetter, NULL, CMDGetterEntry, this);
    pthread_detach(_cmdgetter);
    ////////////////////////////////////////////////////////

    pthread_create(&_logThread, NULL, LogThreadEntry, this);
    readTimer->start(100);
    drvTimer->start(200);

    phv->SetDrvMode(MODE_MANUAL);
    phv->SetStrMode(MODE_MANUAL);

    _gameData.angle = 0;
    _gameData.brake = 0;
    _gameData.button = 0;
    _gameData.drive = 0;

    _gameEnable = false;
    _gameRes = _Game.GameInit();
    _Game.SetGameReceiveHandler(this);

    _Game.GameStart();
    phv->SetConfigCallback(this);
}

MainWindow::~MainWindow()
{
    phv->Stop();
    phv->Close();
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
//    readTimer->start(10);
    while(1){
        updateTime();
        if(_selectLog.start == true){
	    /////////////////////////////////////////////////
	    // Autoware extension
	    //writeLog();
	    SendCAN();
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


/////////////////////////////////////////////////////////////////////////////
// drive slots
/////////////////////////////////////////////////////////////////////////////
void MainWindow::setDrvStrokeSBox()
{
    if(ui->horizontalSlider_drvStroke->value() != ui->spinBox_drvStroke->value()){
        ui->spinBox_drvStroke->setValue(ui->horizontalSlider_drvStroke->value());
        phv->SetDrvStroke(ui->horizontalSlider_drvStroke->value()/1000.0f);
        _drvTargetStroke = ui->horizontalSlider_drvStroke->value()/1000.0f;
    }
}

void MainWindow::setDrvVelocSBox()
{
    if(ui->horizontalSlider_drvVeloc->value() != ui->spinBox_drvVeloc->value()){
        ui->spinBox_drvVeloc->setValue(ui->horizontalSlider_drvVeloc->value());
        phv->SetDrvVeloc(ui->horizontalSlider_drvVeloc->value());
        _drvTargetVeloc = ui->horizontalSlider_drvVeloc->value();
    }
}

void MainWindow::setDrvStrokeSlider()
{
    if(ui->spinBox_drvStroke->value() != ui->horizontalSlider_drvStroke->value()){
        ui->horizontalSlider_drvStroke->setValue(ui->spinBox_drvStroke->value());
        phv->SetDrvStroke(ui->spinBox_drvStroke->value()/1000.0f);
        _drvTargetStroke = ui->spinBox_drvStroke->value()/1000.0f;
    }
}

void MainWindow::setDrvVelocSlider()
{
    if(ui->spinBox_drvVeloc->value() != ui->horizontalSlider_drvVeloc->value()){
        ui->horizontalSlider_drvVeloc->setValue(ui->spinBox_drvVeloc->value());
        phv->SetDrvVeloc(ui->spinBox_drvVeloc->value());
        _drvTargetVeloc = ui->spinBox_drvVeloc->value();
    }
}

void MainWindow::setDrvStrokeZero()
{
    phv->SetDrvStroke(0);
    ui->horizontalSlider_drvStroke->setValue(0);
    ui->spinBox_drvStroke->setValue(0);
}

void MainWindow::setDrvVelocZero()
{
    phv->SetDrvVeloc(0);
    ui->horizontalSlider_drvVeloc->setValue(0);
    ui->spinBox_drvVeloc->setValue(0);
}

void MainWindow::sndDrvModeM()
{
    phv->SetDrvMode(MODE_MANUAL);
    ui->horizontalSlider_drvStroke->setValue(0);
    ui->horizontalSlider_drvVeloc->setValue(0);
    _drvTargetVeloc = 0;
    _drvTargetStroke = 0;
}

void MainWindow::sndDrvModeP()
{
    phv->SetDrvMode(MODE_PROGRAM);
}

void MainWindow::sndDrvCModeS()
{
    phv->SetDrvCMode(CONT_MODE_STROKE);
}

void MainWindow::sndDrvCModeV()
{
    phv->SetDrvCMode(CONT_MODE_VELOCITY);
}

void MainWindow::sndDrvOModeON()
{
    phv->SetDrvOMode(OVERRIDE_MODE_ENABLE);
}

void MainWindow::sndDrvOModeOFF()
{
    phv->SetDrvOMode(OVERRIDE_MODE_DISABLE);
}

void MainWindow::sndDrvServoON()
{
    phv->SetDrvServo(SERVO_MODE_ON);
}

void MainWindow::sndDrvServoOFF()
{
    phv->SetDrvServo(SERVO_MODE_OFF);
}

void MainWindow::sndDrvShiftD()
{
    phv->SetDrvShiftMode(SHIFT_POS_D);
}

void MainWindow::sndDrvShiftN()
{
    phv->SetDrvShiftMode(SHIFT_POS_N);
}

void MainWindow::sndDrvShiftS()
{
    phv->SetDrvShiftMode(SHIFT_POS_S);
}

void MainWindow::sndDrvShiftR()
{
    phv->SetDrvShiftMode(SHIFT_POS_R);
}

void MainWindow::sndDrvShiftB()
{
    phv->SetDrvShiftMode(SHIFT_POS_B);
}

/////////////////////////////////////////////////////////////////////////////
// steer slots
/////////////////////////////////////////////////////////////////////////////
void MainWindow::setStrAngleSBox()
{
    if(ui->horizontalSlider_strAngle->value() != ui->spinBox_strAngle->value()){
        ui->spinBox_strAngle->setValue(ui->horizontalSlider_strAngle->value());
        phv->SetStrAngle(ui->spinBox_strAngle->value()*-10);
    }
}

void MainWindow::setStrTorqueSBox()
{
    if(ui->horizontalSlider_strTorque->value() != ui->spinBox_strTorque->value()){
        ui->spinBox_strTorque->setValue(ui->horizontalSlider_strTorque->value());
        phv->SetStrTorque(ui->spinBox_strTorque->value()/-1000.0f);
    }
}

void MainWindow::setStrAngleSlider()
{
    if(ui->spinBox_strAngle->value() != ui->horizontalSlider_strAngle->value()){
        ui->horizontalSlider_strAngle->setValue(ui->spinBox_strAngle->value());
        phv->SetStrAngle(ui->horizontalSlider_strAngle->value()*-10);
    }
}

void MainWindow::setStrTorqueSlider()
{
    if(ui->spinBox_strTorque->value() != ui->horizontalSlider_strTorque->value()){
        ui->horizontalSlider_strTorque->setValue(ui->spinBox_strTorque->value());
        phv->SetStrTorque(ui->horizontalSlider_strTorque->value()/-1000.0f);
    }
}

void MainWindow::setStrAngleZero()
{
    phv->SetStrAngle(0);
    ui->horizontalSlider_strAngle->setValue(0);
    ui->spinBox_strAngle->setValue(0);
}

void MainWindow::setStrTorqueZero()
{
    phv->SetStrTorque(0.0f);
    ui->horizontalSlider_strTorque->setValue(0);
    ui->spinBox_strTorque->setValue(0);
}

void MainWindow::sndStrModeM()
{
    phv->SetStrMode(MODE_MANUAL);
}

void MainWindow::sndStrModeP()
{
    phv->SetStrMode(MODE_PROGRAM);
}

void MainWindow::sndStrCModeA()
{
    phv->SetStrCMode(CONT_MODE_ANGLE);
}

void MainWindow::sndStrCModeT()
{
    phv->SetStrCMode(CONT_MODE_TORQUE);
}

void MainWindow::sndStrOModeON()
{
    phv->SetStrOMOde(OVERRIDE_MODE_ENABLE);
}

void MainWindow::sndStrOModeOFF()
{
    phv->SetStrOMOde(OVERRIDE_MODE_DISABLE);
}

void MainWindow::sndStrServoON()
{
    phv->SetStrServo(SERVO_MODE_ON);
}

void MainWindow::sndStrServoOFF()
{
    phv->SetStrServo(SERVO_MODE_OFF);
}


/////////////////////////////////////////////////////////////////////////////
// brake slots
/////////////////////////////////////////////////////////////////////////////
void MainWindow::setBrkStrokeSBox()
{
    if(ui->horizontalSlider_brkStroke->value() != ui->spinBox_brkStroke->value()){
        ui->spinBox_brkStroke->setValue(ui->horizontalSlider_brkStroke->value());
        phv->SetBrakeStroke(ui->spinBox_brkStroke->value()/1000.0f);
    }
}

void MainWindow::setBrkStrokeSlider()
{
    if(ui->spinBox_brkStroke->value() != ui->horizontalSlider_brkStroke->value()){
        ui->horizontalSlider_brkStroke->setValue(ui->spinBox_brkStroke->value());
        phv->SetBrakeStroke(ui->horizontalSlider_brkStroke->value()/1000.0f);
    }
}

void MainWindow::setBrkStrokeZero()
{
    phv->SetBrakeStroke(0);
    ui->horizontalSlider_brkStroke->setValue(0);
    ui->spinBox_brkStroke->setValue(0);
}

void MainWindow::sndBrkLampON()
{
    phv->SndBrakeLamp(1);
}

void MainWindow::sndBrkLampOFF()
{
    phv->SndBrakeLamp(0);
}

void MainWindow::sndLeftBlinkerON()
{
    phv->SndLeftBlinker(1);
}

void MainWindow::sndLeftBlinkerOFF()
{
    phv->SndLeftBlinker(0);
}

void MainWindow::sndRightBlinkerON()
{
    phv->SndRightBlinker(1);
}

void MainWindow::sndRightBlinkerOFF()
{
    phv->SndRightBlinker(0);
}

void MainWindow::sndBrkAutoModeON()
{
    phv->SndBrakeAutoLamp(1);
}

void MainWindow::sndBrkAutoModeOFF()
{
    phv->SndBrakeAutoLamp(0);
}


/////////////////////////////////////////////////////////////////////////////
// Diag slots
/////////////////////////////////////////////////////////////////////////////
/*void MainWindow::SndDiagEngineReq()
{
    phv->SndDiagReq(ENGINE_ECU);
}

void MainWindow::SndDiagEngineClear()
{
    phv->SndDiagClear(ENGINE_ECU);
}

void MainWindow::SndDiagHVReq()
{
    phv->SndDiagReq(HV_ECU);
}

void MainWindow::SndDiagHVClear()
{
    phv->SndDiagClear(HV_ECU);
}

void MainWindow::SndDiagCruiseReq()
{
    phv->SndDiagReq(BRAKE_ECU);
}

void MainWindow::SndDiagBattReq()
{
    phv->SndDiagReq(BATTERY_ECU);
}

void MainWindow::SndDiagBattClear()
{
    phv->SndDiagClear(BATTERY_ECU);
}*/

void MainWindow::GetConfStrGainKP()
{
    phv->GetConfig(CONFIG_ANGLE_CTRL_KP);
}

void MainWindow::GetConfStrGainKI()
{
    phv->GetConfig(CONFIG_ANGLE_CTRL_KI);
}

void MainWindow::GetConfStrGainKD()
{
    phv->GetConfig(CONFIG_ANGLE_CTRL_KD);
}

void MainWindow::GetConfStrDecTime()
{
    phv->GetConfig(CONFIG_ANGLE_CTRL_DEC_TIME);
}

void MainWindow::GetConfStrWindow()
{
    phv->GetConfig(CONFIG_ANGLE_CTRL_WINDOW);
}

void MainWindow::GetConfStrPunch()
{
    phv->GetConfig(CONFIG_ANGLE_CTRL_PUNCH);
}

void MainWindow::GetConfStrMaxTrq()
{
    phv->GetConfig(CONFIG_ANGLE_CTRL_OUTPUT_MAX);
}

void MainWindow::GetConfStrCCWFactor()
{
    phv->GetConfig(CONFIG_ANGLE_CTRL_CCW_FACTOR);
}

void MainWindow::GetConfVelocMaxSpeed()
{
    phv->GetConfig(CONFIG_VELOC_CTRL_MAX_SPEED);
}

void MainWindow::GetConfVelocGainKPP()
{
    phv->GetConfig(CONFIG_VELOC_CTRL_KP_P);
}

void MainWindow::GetConfVelocGainKPM()
{
    phv->GetConfig(CONFIG_VELOC_CTRL_KP_M);
}

void MainWindow::GetConfVelocWindowP()
{
    phv->GetConfig(CONFIG_VELOC_CTRL_WINDOW_P);
}

void MainWindow::GetConfVelocWindowM()
{
    phv->GetConfig(CONFIG_VELOC_CTRL_WINDOW_M);
}

void MainWindow::GetConfVelocPunchP()
{
    phv->GetConfig(CONFIG_VELOC_CTRL_PUNCH_P);
}

void MainWindow::GetConfVelocPunchM()
{
    phv->GetConfig(CONFIG_VELOC_CTRL_PUNCH_M);
}

void MainWindow::GetConfVelocMaxOutputP()
{
    phv->GetConfig(CONFIG_VELOC_CTRL_OUTPUT_MAX_P);
}

void MainWindow::GetConfVelocMaxOutputM()
{
    phv->GetConfig(CONFIG_VELOC_CTRL_OUTPUT_MAX_M);
}

void MainWindow::GetConfStrOverrideTh()
{
    phv->GetConfig(CONFIG_OVERRIDE_STEER_TH);
}

void MainWindow::GetConfAccOverrideTh()
{
    phv->GetConfig(CONFIG_OVERRIDE_ACCEL_TH);
}

void MainWindow::GetConfBrkOverrideTh()
{
    phv->GetConfig(CONFIG_OVERRIDE_BRAKE_TH);
}

void MainWindow::GetConfTestFlags()
{
    phv->GetConfig(TEST_FLAGS);
}

void MainWindow::SetConfStrGainKP()
{
    phv->SetControlGain(CONFIG_ANGLE_CTRL_KP, ui->spinBox_confStrGainKP->value());
}

void MainWindow::SetConfStrGainKI()
{
    phv->SetControlGain(CONFIG_ANGLE_CTRL_KI, ui->spinBox_confStrGainKI->value());
}

void MainWindow::SetConfStrGainKD()
{
    phv->SetControlGain(CONFIG_ANGLE_CTRL_KD, ui->spinBox_confStrGainKD->value());
}

void MainWindow::SetConfStrDecTime()
{
    phv->SetControlGain(CONFIG_ANGLE_CTRL_DEC_TIME, ui->spinBox_confStrDecTime->value());
}

void MainWindow::SetConfStrWindow()
{
    phv->SetControlGain(CONFIG_ANGLE_CTRL_WINDOW, ui->spinBox_confStrWindow->value());
}

void MainWindow::SetConfStrPunch()
{
    phv->SetControlGain(CONFIG_ANGLE_CTRL_PUNCH, ui->spinBox_confStrPunch->value());
}

void MainWindow::SetConfStrMaxTrq()
{
    phv->SetControlGain(CONFIG_ANGLE_CTRL_OUTPUT_MAX, ui->spinBox_confStrMaxTrq->value());
}

void MainWindow::SetConfStrCCWFactor()
{
    phv->SetControlGain(CONFIG_ANGLE_CTRL_CCW_FACTOR, ui->spinBox_confStrCCWFactor->value());
}

void MainWindow::SetConfVelocMaxSpeed()
{
    phv->SetControlGain(CONFIG_VELOC_CTRL_MAX_SPEED, ui->spinBox_confVelocMaxSpeed->value());
}

void MainWindow::SetConfVelocGainKPP()
{
    phv->SetControlGain(CONFIG_VELOC_CTRL_KP_P, ui->spinBox_confVelocGainKPP->value());
}

void MainWindow::SetConfVelocGainKPM()
{
    phv->SetControlGain(CONFIG_VELOC_CTRL_KP_M, ui->spinBox_confVelocGainKPM->value());
}

void MainWindow::SetConfVelocWindowP()
{
    phv->SetControlGain(CONFIG_VELOC_CTRL_WINDOW_P, ui->spinBox_confVelocWindowP->value());
}

void MainWindow::SetConfVelocWindowM()
{
    phv->SetControlGain(CONFIG_VELOC_CTRL_WINDOW_M, ui->spinBox_confVelocWindowM->value());
}

void MainWindow::SetConfVelocPunchP()
{
    phv->SetControlGain(CONFIG_VELOC_CTRL_PUNCH_P, ui->spinBox_confVelocPunchP->value());
}

void MainWindow::SetConfVelocPunchM()
{
    phv->SetControlGain(CONFIG_VELOC_CTRL_PUNCH_M, ui->spinBox_confVelocPunchM->value());
}

void MainWindow::SetConfVelocMaxOutputP()
{
    phv->SetControlGain(CONFIG_VELOC_CTRL_OUTPUT_MAX_P, ui->spinBox_confVelocMaxOutputP->value());
}

void MainWindow::SetConfVelocMaxOutputM()
{
    phv->SetControlGain(CONFIG_VELOC_CTRL_OUTPUT_MAX_M, ui->spinBox_confVelocMaxOutputM->value());
}

void MainWindow::SetConfStrOverrideTh()
{
    phv->SetControlGain(CONFIG_OVERRIDE_STEER_TH, ui->spinBox_confStrOverrideTh->value());
}

void MainWindow::SetConfAccOverrideTh()
{
    phv->SetControlGain(CONFIG_OVERRIDE_ACCEL_TH, ui->spinBox_confDrvOverrideTh->value());
}

void MainWindow::SetConfBrkOverrideTh()
{
    phv->SetControlGain(CONFIG_OVERRIDE_BRAKE_TH, ui->spinBox_confBrkOverrideTh->value());
}

void MainWindow::SetConfTestFlags()
{
    phv->SetControlGain(TEST_FLAGS, ui->spinBox_confTestFlags->value());
}

void MainWindow::SaveConfig()
{
    phv->SaveConfig();
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
    phv->GetBattInf(&_battInf);
    phv->GetBrakeInf(&_brakeInf);
    phv->GetDrvInf(&_drvInf);
    phv->GetOtherInf(&_otherInf);
    phv->GetStrInf(&_strInf);
    phv->GetErrCode(&_errLevel, &_errCode);
    phv->GetImuInf(&_imuInf);
    phv->GetPosInf(&_posInf);
    phv->GetIncInf(&_incInf);

    viewBattInf();
    viewBrakeInf();
    viewDrvInf();
    viewOtherInf();
    viewStrInf();
    viewImuInf();
    viewPosInf();
    viewIncInf();
//    viewErrCode();
}

void MainWindow::sndDrvTarget()
{
    if((_drvInf.mode == MODE_PROGRAM) && (_drvInf.servo == SERVO_MODE_ON)){
        if((_drvInf.contMode == CONT_MODE_STROKE) && (_drvTargetStroke != 0)){
            phv->SetDrvStroke(_drvTargetStroke);
        } else if((_drvInf.contMode == CONT_MODE_VELOCITY) && (_drvTargetVeloc != 0)){
            phv->SetDrvVeloc(_drvTargetVeloc);
        }
    }
}

void MainWindow::writeLog()
{
    fprintf(_logSave,"%d/%d/%d/%d:%d:%d.%ld,",_s_time->tm_year+1900, _s_time->tm_mon+1, _s_time->tm_mday,
            _s_time->tm_hour, _s_time->tm_min, _s_time->tm_sec, _getTime.tv_usec);
    if(_selectLog.drvInf == true){
        fprintf(_logSave, "%d,%d,%d,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%d,%d,%d",
                _drvInf.mode, _drvInf.contMode, _drvInf.overrideMode, _drvInf.servo,
                _drvInf.actualPedalStr, _drvInf.targetPedalStr, _drvInf.inputPedalStr,
                _drvInf.targetVeloc, _drvInf.actualVeloc,
                _drvInf.actualShift, _drvInf.targetShift, _drvInf.inputShift);
    } else {
        fprintf(_logSave, ",,,,,,,,,,,,,,,,,,,,");
    }
    if(_selectLog.strInf == true){
        fprintf(_logSave,"%d,%d,%d,%d,%3.2f,%3.2f,%3.2f,%3.2f",
                _strInf.mode, _strInf.cont_mode, _strInf.overrideMode, _strInf.servo,
                _strInf.targetTorque, _strInf.actualTorque,
                _strInf.actualAngle, _strInf.targetAngle);
    } else {
        fprintf(_logSave,",,,,,,,,,,");
    }
    if(_selectLog.brkInf == true){
        fprintf(_logSave,"%1.2f,%1.2f,%1.2f,%d,%d,%d,%d,%d,%d,%d,%d",
                _brakeInf.actualPedalStr, _brakeInf.targetPedalStr, _brakeInf.inputPedalStr,
                _brakeInf.sks,_brakeInf.sla,_brakeInf.regPress,_brakeInf.wheelPress,
                _brakeInf.slr,_brakeInf.src,_brakeInf.sksT,_brakeInf.regT);
    } else {
        fprintf(_logSave,",,,,,,,,,,,");
    }
    if(_selectLog.battInf == true){
        fprintf(_logSave,"%3.2f,%3.2f,%3.2f,,%3.2f",
                _battInf.soc, _battInf.voltage, _battInf.current, _battInf.subVoltage);
    } else {
        fprintf(_logSave,",,,,,,");
    }
    if(_selectLog.otherInf == true){
        fprintf(_logSave,"%d,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%d,%1.2f,%1.2f,%d,%3.1f,%d,%d,%d,%d,",
                _otherInf.odometry,
                _otherInf.velocFrFromP,_otherInf.velocFlFromP, _otherInf.velocRrFromP, _otherInf.velocRlFromP,
                _otherInf.velocFromP, _otherInf.engineRpm, _otherInf.motorRpm,
                _otherInf.coolantTemp, _otherInf.shiftFromPrius,
                _otherInf.drvPedalStrFromP, _otherInf.brkPedalStrFromP, _otherInf.brkState,
                _otherInf.angleFromP,
                _otherInf.drv_mode, _otherInf.ecoMode, _otherInf.ev_mode,
                _otherInf.light);
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
}

void MainWindow::viewBattInf()
{
    ui->textEdit_viBattSOC->setText(QString::number(_battInf.soc, 'f', 1));
    ui->textEdit_viBattVolt->setText(QString::number(_battInf.voltage, 'f',2));
    ui->textEdit_viBattCurrent->setText(QString::number(_battInf.current, 'f', 2));
    ui->textEdit_viBattSubVolt->setText(QString::number(_battInf.subVoltage, 'f',2));
}

void MainWindow::viewBrakeInf()
{
    if(_otherInf.brkState == true){
        ui->textEdit_brkPressState->setText(QString("ON"));
    }else
        ui->textEdit_brkPressState->setText(QString("OFF"));

    ui->textEdit_brkStrokeInput->setText(QString::number(_brakeInf.inputPedalStr, 'f',2));
    ui->textEdit_brkStrokeTarget->setText(QString::number(_brakeInf.targetPedalStr, 'f',2));
    ui->textEdit_brkStrokeActual->setText(QString::number(_brakeInf.actualPedalStr, 'f',2));

    ui->textEdit_brkSksActual->setText(QString::number(_brakeInf.sks, 10));
    ui->textEdit_brkSksTarget->setText(QString::number(_brakeInf.sksT, 10));
    ui->textEdit_brkRegActual->setText(QString::number(_brakeInf.regPress, 10));
    ui->textEdit_brkRegTarget->setText(QString::number(_brakeInf.regT, 10));
    ui->textEdit_brkWheelActual->setText(QString::number(_brakeInf.wheelPress, 10));
    ui->textEdit_brkSrc->setText(QString::number(_brakeInf.src, 10));
//    ui->textEdit_brkSla->setText(QString::number(_brakeInf.sla, 10));
//    ui->textEdit_brkSlr->setText(QString::number(_brakeInf.slr, 10));

}

void MainWindow::viewDrvInf()
{
    if(_drvInf.contMode == CONT_MODE_VELOCITY)
        ui->textEdit_drvCntMode->setText(QString("velocity"));
    else
        ui->textEdit_drvCntMode->setText(QString("pedal"));
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

    if(_drvInf.overrideMode == OVERRIDE_MODE_DISABLE)
        ui->textEdit_drvOverMode->setText(QString("disable"));
    else
        ui->textEdit_drvOverMode->setText(QString("enable"));

    if(_drvInf.servo == SERVO_MODE_OFF)
        ui->textEdit_drvServo->setText(QString("OFF"));
    else
        ui->textEdit_drvServo->setText(QString("ON"));

    ui->textEdit_drvVelocTarget->setText(QString::number(_drvInf.targetVeloc, 'f', 2));
    ui->textEdit_drvVelocActual->setText(QString::number(_drvInf.actualVeloc, 'f', 2));

    ui->textEdit_drvStrokeInput->setText(QString::number(_drvInf.inputPedalStr, 'f', 2));
    ui->textEdit_drvStrokeTarget->setText(QString::number(_drvInf.targetPedalStr, 'f',2));
    ui->textEdit_drvStrokeActual->setText(QString::number(_drvInf.actualPedalStr, 'f', 2));

    QString input;
    QString target;
    QString actual;

    switch(_drvInf.inputShift){
    case SHIFT_POS_B: input = "B"; break;
    case SHIFT_POS_D: input = 'D'; break;
    case SHIFT_POS_N: input = 'N'; break;
    case SHIFT_POS_R: input = 'R'; break;
    case SHIFT_POS_P: input = 'P'; break;
    case SHIFT_POS_S: input = 'S'; break;
    case SHIFT_POS_U: input = 'U'; break;
    default: input = 'U'; break;
    }

    switch(_drvInf.targetShift){
    case SHIFT_POS_B: target = "B"; break;
    case SHIFT_POS_D: target = 'D'; break;
    case SHIFT_POS_N: target = 'N'; break;
    case SHIFT_POS_R: target = 'R'; break;
    case SHIFT_POS_P: target = 'P'; break;
    case SHIFT_POS_S: target = 'S'; break;
    case SHIFT_POS_U: target = 'U'; break;
    default: target = 'U'; break;
    }

    switch(_drvInf.actualShift){
    case SHIFT_POS_B: actual = 'B'; break;
    case SHIFT_POS_D: actual = 'D'; break;
    case SHIFT_POS_N: actual = 'N'; break;
    case SHIFT_POS_R: actual = 'R'; break;
    case SHIFT_POS_P: actual = 'P'; break;
    case SHIFT_POS_S: actual = 'S'; break;
    case SHIFT_POS_U: actual = 'U'; break;
    default: actual = 'U'; break;
    }
    ui->textEdit_drvShiftInput->setText(input);
    ui->textEdit_drvShiftTarget->setText(target);
    ui->textEdit_drvShiftActual->setText(actual);

    ui->textEdit_gameButton->setText(QString::number(_gameData.button, 10));
    ui->textEdit_gameStr->setText(QString::number(_gameData.angle/10.0f, 'f', 1));
    ui->textEdit_gameDrv->setText(QString::number(_gameData.drive/4095.0f, 'f',2));
    ui->textEdit_gameBrk->setText(QString::number(_gameData.brake/4095.0f, 'f',2));
}

void MainWindow::viewOtherInf()
{
    QString light = "";
    switch(_otherInf.light){
    case LIGHT_OFF: light = "OFF"; break;
    case LIGHT_PARK: light = "PARK"; break;
    case LIGHT_ON: light = "ON"; break;
    case LIGHT_HIGH: light = "HIGH"; break;
    default: light = "unknown";break;
    }
    ui->textEdit_viLight->setText(light);
    QString dmode;
    switch(_otherInf.drv_mode){
    case 0: dmode = 'P'; break;
    case 1: dmode = 'R'; break;
    case 2: dmode = 'N'; break;
    case 3: dmode = 'D'; break;
    case 4: dmode = 'B'; break;
    default: dmode = 'S'; break;
    }

    ui->textEdit_viEngineRPM->setText(QString::number(_otherInf.engineRpm, 'f',2));
    ui->textEdit_viMotorRPM->setText(QString::number(_otherInf.motorRpm, 'f',2));
    ui->textEdit_viCoolantTemp->setText(QString::number(_otherInf.coolantTemp, 'f',1));
    ui->textEdit_viOdometer->setText(QString::number(_otherInf.odometry, 'f',1));

    QString *diag = new QString();
    *diag = QString::number(_otherInf.diagData[0], 16);
    *diag += " ";
    *diag += QString::number(_otherInf.diagData[1], 16);
    *diag += " ";
    *diag += QString::number(_otherInf.diagData[2], 16);
    *diag += " ";
    *diag += QString::number(_otherInf.diagData[3], 16);
    *diag += " ";
    *diag += QString::number(_otherInf.diagData[4], 16);
    *diag += " ";
    *diag += QString::number(_otherInf.diagData[5], 16);
    *diag += " ";
    *diag += QString::number(_otherInf.diagData[6], 16);
    *diag += " ";
    *diag += QString::number(_otherInf.diagData[7], 16);
    *diag += " ";
//    ui->textEdit_diagState->setText(*diag);

    ui->textEdit_viStrAngle->setText(QString::number(_otherInf.angleFromP, 'f', 1));

    QString shift;
    switch(_otherInf.shiftFromPrius){
    case SHIFT_POS_P: shift = 'P'; break;
    case SHIFT_POS_R: shift = 'R'; break;
    case SHIFT_POS_N: shift = 'N'; break;
    case SHIFT_POS_D: shift = 'D'; break;
    case SHIFT_POS_B: shift = 'B'; break;
    case SHIFT_POS_S: shift = 'S'; break;
    default: shift = 'U'; break;
    }
    ui->textEdit_viShiftPos->setText(shift);

    ui->textEdit_viDrvStroke->setText(QString::number(_otherInf.drvPedalStrFromP, 'f',2));
    ui->textEdit_viBrkStroke->setText(QString::number(_otherInf.brkPedalStrFromP, 'f',2));
    if(_otherInf.brkState == true){
        ui->textEdit_viBrkState->setText(QString("ON"));
    }else
        ui->textEdit_viBrkState->setText(QString("OFF"));
    ui->textEdit_viVelocFL->setText(QString::number(_otherInf.velocFlFromP, 'f', 2));
    ui->textEdit_viVelocFR->setText(QString::number(_otherInf.velocFrFromP, 'f', 2));
    ui->textEdit_viVelocRL->setText(QString::number(_otherInf.velocRlFromP, 'f', 2));
    ui->textEdit_viVelocRR->setText(QString::number(_otherInf.velocRrFromP, 'f', 2));
    ui->textEdit_viVeloc->setText(QString::number((int)_otherInf.velocFromP, 10));
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
    if(_strInf.overrideMode == OVERRIDE_MODE_ENABLE)
        ui->textEdit_strOverMode->setText(QString("enable"));
    else
        ui->textEdit_strOverMode->setText(QString("disable"));
    if(_strInf.servo == SERVO_MODE_OFF)
        ui->textEdit_strServo->setText(QString("OFF"));
    else
        ui->textEdit_strServo->setText(QString("ON"));

    ui->textEdit_strTorqueTarget->setText(QString::number(_strInf.targetTorque, 'f',2));
    ui->textEdit_strTorqueActual->setText(QString::number(_strInf.actualTorque, 'f', 2));

    ui->textEdit_strAngleTarget->setText(QString::number(_strInf.targetAngle, 'f', 1));
    ui->textEdit_strAngleActual->setText(QString::number(_strInf.actualAngle, 'f', 1));
}

void MainWindow::viewImuInf()
{
    ui->textEdit_ImuAccX->setText(QString::number(_imuInf.accX, 'f', 5));
    ui->textEdit_ImuAccY->setText(QString::number(_imuInf.accY, 'f', 5));
    ui->textEdit_ImuAccZ->setText(QString::number(_imuInf.accZ, 'f', 5));

    ui->textEdit_ImuGyroX->setText(QString::number(_imuInf.gyroX, 'f', 5));
    ui->textEdit_ImuGyroY->setText(QString::number(_imuInf.gyroY, 'f', 5));
    ui->textEdit_ImuGyroZ->setText(QString::number(_imuInf.gyroZ, 'f', 5));

    ui->textEdit_ImuCompX->setText(QString::number(_imuInf.compX, 'f', 5));
    ui->textEdit_ImuCompY->setText(QString::number(_imuInf.compY, 'f', 5));
    ui->textEdit_ImuCompZ->setText(QString::number(_imuInf.compZ, 'f', 5));
}

void MainWindow::viewIncInf()
{
    ui->textEdit_incX->setText(QString::number(_incInf.angleX, 'f', 5));
    ui->textEdit_incY->setText(QString::number(_incInf.angleY, 'f', 5));

}

void MainWindow::viewPosInf()
{
    ui->textEdit_UTC->setText(QString::number(_posInf.gga.utcTime.h, 10) + ':' +
                              QString::number(_posInf.gga.utcTime.m, 10) + ':' +
                              QString::number(_posInf.gga.utcTime.s, 10) + '.' +
                              QString::number(_posInf.gga.utcTime.ms, 10));
    ui->textEdit_latitude->setText(QString::number(_posInf.gga.latitude, 'f', 5));
    ui->textEdit_latitudeSigne->setText(QString::number(_posInf.gga.latitudeSign, 10));
    ui->textEdit_longitude->setText(QString::number(_posInf.gga.longitude, 'f', 6));
    ui->textEdit_longitudeSigne->setText(QString::number(_posInf.gga.longitudeSign, 10));
    ui->textEdit_satelliteNum->setText(QString::number(_posInf.gga.seaLevelAttitude, 10));
    ui->textEdit_quality->setText(QString::number(_posInf.gga.qualityIndicator, 10));
    ui->textEdit_hdop->setText(QString::number(_posInf.gga.hdop));
    ui->textEdit_seaAttitude->setText(QString::number(_posInf.gga.seaLevelAttitude, 10));
    ui->textEdit_seaAttitudeSigne->setText(QString::number(_posInf.gga.seaLevelAttitudeSign, 10));
    ui->textEdit_geoAttitude->setText(QString::number(_posInf.gga.geoLevelAttitude, 10));
    ui->textEdit_geoAttitudeSigne->setText(QString::number(_posInf.gga.geoLevelAttitudeSign, 10));
    ui->textEdit_geoAttitude->setText(QString::number(_posInf.gga.geoLevelAttitude, 10));
    ui->textEdit_geoAttitudeSigne->setText(QString::number(_posInf.gga.geoLevelAttitudeSign, 10));
    ui->textEdit_ageOfDgps->setText(QString::number(_posInf.gga.ageOfDgps, 10));
    ui->textEdit_StationID->setText(QString::number(_posInf.gga.dgpsStationID, 10));
    ui->textEdit_humid->setText(QString::number(_posInf.humid, 'f', 2));
    ui->textEdit_hTemp->setText(QString::number(_posInf.hTemp, 'f', 2));

    ui->textEdit_press->setText(QString::number(_posInf.press, 'f', 2));
    ui->textEdit_pTemp->setText(QString::number(_posInf.pTemp, 'f', 2));
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
                    phv->SetDrvShiftMode(SHIFT_POS_D);
                     break;
                case 1: // □ Button
                    printf("SetDrvShiftMode(N)\n");
                    phv->SetDrvShiftMode(SHIFT_POS_N);
                    break;
                case 2: // ○ Button
                    printf("SetDrvShiftMode(R)\n");
                    phv->SetDrvShiftMode(SHIFT_POS_R);
                    break;
                case 3: // △ Button
                    printf("SetDrvShiftMode(B)\n");
                    phv->SetDrvShiftMode(SHIFT_POS_B);
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
                    phv->SetDrvMode(MODE_MANUAL);
                    phv->SetDrvCMode(CONT_MODE_STROKE);
                    phv->SetDrvOMode(OVERRIDE_MODE_ENABLE);
                    phv->SetDrvServo(SERVO_MODE_OFF);
                    phv->SetStrMode(MODE_MANUAL);
                    phv->SetStrCMode(CONT_MODE_ANGLE);
                    phv->SetStrOMOde(OVERRIDE_MODE_ENABLE);
                    phv->SetStrServo(SERVO_MODE_OFF);
                    break;
                case 9: // START Button
                    printf("Pusshu START\n");
                    phv->SetDrvMode(MODE_PROGRAM);
                    phv->SetDrvCMode(CONT_MODE_STROKE);
                    phv->SetDrvOMode(OVERRIDE_MODE_DISABLE);
                    phv->SetDrvServo(SERVO_MODE_ON);
                    phv->SetStrMode(MODE_PROGRAM);
                    phv->SetStrCMode(CONT_MODE_ANGLE);
                    phv->SetStrOMOde(OVERRIDE_MODE_DISABLE);
                    phv->SetStrServo(SERVO_MODE_ON);
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
                        phv->SetStrAngle((int)sndAngle);
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
                phv->SetDrvStroke(sndDrvStroke/4095.0f/1.5f);
                break;
            case 2: // ブレーキ
    //          val -= 0x7fff;
    //          val *= -1;
                sndBrkStroke = (val - 0x7fff)*-1 * strokeVal;
//                printf("SetBrakeStroke(%d)\n", (int)sndBrkStroke);
//                ui->textEdit_gameBrk->setText(QString::number(sndBrkStroke));
                _gameData.brake = sndBrkStroke;
//                phv->SetBrakeStroke(sndBrkStroke/4095.0f/2.0f);
                if(sndBrkStroke >= 200){
                    _drvTargetVeloc = 0;
                    _drvTargetStroke = 0;
                    phv->SetDrvVeloc(0);
                }
                if(sndBrkStroke <= 70){
                    phv->SetBrakeStroke(0);
                } else {
                    phv->SetBrakeStroke(sndBrkStroke/4095.0f/2.0f);
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
                phv->SetDrvVeloc(_drvTargetVeloc);
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
            phv->SetStrAngle((int)sndAngle);
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
