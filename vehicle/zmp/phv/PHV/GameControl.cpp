/**
 * @file
 * @~english
 * @brief	GameControll class implementation.
 * @author	Koji Sekiguchi
 * @~japanese
 * @brief	Game制御クラス実装.
 * @author	関口 浩司
 * @~
 * @date	2010-08-27
 * Copyright (c) 2010 ZMP Inc. All rights reserved.
 */

#include "GameControl.h"

namespace zmp {
    namespace hev {

const int GameController::_DEFAULT_BUFFER_DEPTH = 200;

GameController::GameController()
    : _b_on_GameJsRcv_thread(false)
    , _b_on_GameHidRcv_thread(false)
    , _callback_handler(NULL)
    , _GameJsrcvBuffer(NULL)
    , _GameHidrcvBuffer(NULL)
{
}

GameController::~GameController() 
{
}

bool GameController::SetGameReceiveHandler(GameReceiveHandler* handler)
{
    printf("SetGameReceiveHandler() \n");
    if(handler != NULL){
	_callback_handler = handler;
	return true;
    } else {
	_callback_handler = NULL;
	return false;
    }
}

bool GameController::GameInit()
{
//    printf("GameInit()\n");
    _GameJsrcvBuffer=new CRingBuffer(GAME_RECEIVE_SIZE* _DEFAULT_BUFFER_DEPTH + 1);
    _GameJsrcvBuffer->Empty();
    bool res;
    res = GameJsInitDevice();
    if(res != true)
    {
        return false;
    }

    res = GameJsInitThread();
    if(res != true)
    {
        return false;
    }

    _GameHidrcvBuffer=new CRingBuffer(GAME_RECEIVE_SIZE* _DEFAULT_BUFFER_DEPTH +1);
    _GameHidrcvBuffer->Empty();
    res = GameHidInitDevice();
    if(res != true)
    {
        return false;
    }

    res = GameHidInitThread();
    if(res != true)
    {
        return false;
    }
    return true;
}


bool GameController::GameJsInitThread()
{
    int res;
    res = pthread_create(&_GameJsthread, NULL, GameJsReadThreadEntry , this);

    if(res != 0)
        return false;

    return true;
}

bool GameController::GameHidInitThread()
{
    int res;
    res = pthread_create(&_GameHidthread, NULL, GameHidReadThreadEntry , this);

    if(res != 0)
        return false;

    return true;
}

bool GameController::GameJsInitDevice()
{
    if((_gameJsfd = open(DEVICE_NAMEGAME_JS ,O_RDWR | O_NOCTTY))==-1)
        return false;

    _GameJsclient.fd    = _gameJsfd;
    _GameJsclient.events = POLLIN;

    tcgetattr(_gameJsfd, &_GameJsoldtio);
    bzero(&_GameJsnewtio, sizeof(_GameJsnewtio));
    _GameJsnewtio.c_cflag = (GAME_BAUDRATE | CS8 | CLOCAL | CREAD);
        _GameJsnewtio.c_iflag = (IGNPAR );
        _GameJsnewtio.c_oflag = 0;
        _GameJsnewtio.c_lflag = 0;
        _GameJsnewtio.c_cc[VMIN] = 1;

    tcflush(_gameJsfd, TCIOFLUSH);
    tcsetattr(_gameJsfd, TCSANOW, &_GameJsnewtio);

    return true;
}


bool GameController::GameHidInitDevice()
{
    if((_gameHidfd = open(DEVICE_NAMEGAME_HID ,O_RDWR | O_NOCTTY))==-1)
    {
        return false;
    }

    _GameHidclient.fd   = _gameHidfd;
    _GameHidclient.events = POLLIN;

    tcgetattr(_gameHidfd, &_GameHidoldtio);
    bzero(&_GameHidnewtio, sizeof(_GameHidnewtio));
    _GameHidnewtio.c_cflag = (GAME_BAUDRATE | CS8 | CLOCAL | CREAD);
    _GameHidnewtio.c_iflag = (IGNPAR );
    _GameHidnewtio.c_oflag = 0;
    _GameHidnewtio.c_lflag = 0;
    _GameHidnewtio.c_cc[VMIN] = 1;

    tcflush(_gameHidfd, TCIOFLUSH);
    tcsetattr(_gameHidfd, TCSANOW, &_GameHidnewtio);

    return true;
}

bool GameController::GetGameJsData(char* msg)
{
//    printf("GetGameData() \n");
    int getsize = _GameJsrcvBuffer->GetQueueLength();
    if(_GameJsrcvBuffer->Get(msg, getsize)){
        return true;
    }else
        return false;
}

bool GameController::GetGameHidData(char* msg)
{
//    printf("GetGameData() \n");
    int getsize = _GameHidrcvBuffer->GetQueueLength();
    if(_GameHidrcvBuffer->Get(msg, getsize)){
        return true;
    }else
        return false;
}


void GameController::GameStart()
{
    printf("GameStart() \n");
    _b_on_GameJsRcv_thread = true;
    _b_on_GameHidRcv_thread = true;
}

void GameController::GameStop()
{
    _b_on_GameJsRcv_thread = false;
    _b_on_GameHidRcv_thread = false;
}


void GameController::GameClose()
{
    _b_on_GameJsRcv_thread = false;
    _b_on_GameHidRcv_thread = false;

    tcsetattr(_gameJsfd, TCSANOW, &_GameJsoldtio);
    if(_gameJsfd!= 0){
        close(_gameJsfd);
    }

    tcsetattr(_gameHidfd, TCSANOW, &_GameHidoldtio);
    if(_gameHidfd!= 0){
        close(_gameHidfd);
    }

    if(_GameJsthread != 0){
        pthread_join(_GameJsthread, 0);
    }

    if(_GameHidthread != 0){
        pthread_join(_GameHidthread, 0);
    }

    if(_GameJsrcvBuffer){
        delete _GameJsrcvBuffer;
        _GameJsrcvBuffer= NULL;
    }

    if(_GameHidrcvBuffer){
        delete _GameHidrcvBuffer;
        _GameHidrcvBuffer= NULL;
    }
}

void* GameController::GameJsReadThreadEntry(void* arg)
{
    printf("GameJsReadThreadEntry() \n");
    GameController* serial_read = (GameController*)arg;
    serial_read->GameJsReadThread();
    return NULL;
}

void* GameController::GameHidReadThreadEntry(void* arg)
{
    printf("GameHidReadThreadEntry() \n");
    GameController* serial_read = (GameController*)arg;
    serial_read->GameHidReadThread();
    return NULL;
}

unsigned char GameController::calc_sum(const unsigned char *data, int len) 
{
    unsigned char sum = 0;
    for(int i=0; i<len; i++){
	sum += data[i];
    }
    return sum;
}

void GameController::GameJsReadThread()
 {
    printf("GameJsReadThread() \n");
    unsigned char res[8];
    int read_res;

    while (1) {
        if(_b_on_GameJsRcv_thread){
            if((read_res = read(_gameJsfd, &res, 8)) > 0) {
                _GameJsrcvBuffer->Put((char*)res, read_res);
                if(_callback_handler) {
                    _callback_handler->OnGameJsReceive();
                }
            } else {
                usleep(10*1000);;
            }
        } else {
            usleep(10 * 1000);
        }
    }
}

void GameController::GameHidReadThread()
 {
    printf("GameHidReadThread() \n");
    unsigned char res[16];
    int read_res;

    while (1) {
        if(_b_on_GameHidRcv_thread){
            if((read_res = read(_gameHidfd, &res, 16)) > 0) {
                _GameHidrcvBuffer->Put((char*)res, read_res);
                if(_callback_handler) {
                    _callback_handler->OnGameHidReceive();
                }
            } else {
                usleep(10*1000);;
            }
        } else {
            usleep(10 * 1000);
        }
    }
}
    }
}
