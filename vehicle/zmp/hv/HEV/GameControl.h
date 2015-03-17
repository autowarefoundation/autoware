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
