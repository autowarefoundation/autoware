#ifndef GAMECONTROL_H_
#define GAMECONTROL_H_

#include "RingBuffer.h"
#include <termios.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <strings.h>
#include <unistd.h>

#define GAME_RECEIVE_SIZE	8	
#define DEVICE_NAMEGAME_JS      "/dev/input/js0"
#define DEVICE_NAMEGAME_HID     "/dev/hidraw0"
#define GAME_BAUDRATE		B115200

namespace zmp {
    namespace hev {
class CRingBuffer;

struct GameData {
    float angle;
    int drive;
    int brake;
    int button;
};

class GameReceiveHandler{
public:
    virtual void OnGameJsReceive()	= 0;
    virtual void OnGameHidReceive() = 0;
};

class GameController{
public:
	GameController();
	virtual ~GameController();

private:
	static const int _DEFAULT_BUFFER_DEPTH;
    bool    _b_on_GameJsRcv_thread;
    bool    _b_on_GameHidRcv_thread;
    int _gameJsfd;
    int _gameHidfd;
    pthread_t _GameJsthread;
    pthread_t _GameHidthread;

    struct termios _GameJsoldtio;
    struct termios _GameHidoldtio;
    struct termios _GameJsnewtio;
    struct termios _GameHidnewtio;
    struct pollfd _GameJsclient;
    struct pollfd _GameHidclient;
    GameReceiveHandler* _callback_handler;
    CRingBuffer*    _GameJsrcvBuffer;
    CRingBuffer*    _GameHidrcvBuffer;

public:
	bool GameInit();
	bool SetGameReceiveHandler(GameReceiveHandler *handler);
    bool GetGameJsData(char* msg);
    bool GetGameHidData(char* msg);
    void GameStart();
	void GameStop();
	void GameClose();

private:
    bool GameJsInitThread();
    bool GameHidInitThread();
    bool GameJsInitDevice();
    bool GameHidInitDevice();
    void GameJsReadThread();
    void GameHidReadThread();
    static void* GameJsReadThreadEntry(void* pParam);
    static void* GameHidReadThreadEntry(void* pParam);
    unsigned char calc_sum(const unsigned char *data, int len);
};
	}
}
#endif /* GAMECONTROL_H_*/
