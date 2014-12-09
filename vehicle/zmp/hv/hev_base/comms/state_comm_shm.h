
#ifndef __STATE_COMM_SHM_H__
#define __STATE_COMM_SHM_H__

#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include "../common/shm_common.h"
#include "shm_client.h"

static bool GLOBAL_term_signal_received;

#define STATE_COMM_DEFAULT_ODOM_PORT 20001
#define STATE_COMM_DEFAULT_VEL_CMD_PORT 20002
#define STATE_COMM_DEFAULT_BASE_PORT  20003
#define STATE_COMM_DEFAULT_LRF_PORT  20009

#define STATE_COMM_DEFAULT_SERVER_HOSTNAME "192.168.0.3"

#define STATE_VEL 1
#define STATE_ODOM 2
#define STATE_VEL_CMD 3
#define STATE_LRF 4
#define STATE_BASE 5 

#define SERVER_MODE 100
#define CLIENT_MODE 101

class StateCommShm {
 public:
  StateCommShm(SHMClient * shm, 
	       int type = STATE_ODOM, 
	       int mode = SERVER_MODE, 
	       int port = STATE_COMM_DEFAULT_ODOM_PORT, 
	       int (*commHandler)(int socket, SHMClient * shm) = NULL,
	       char * hostname = NULL);
  /*StateCommShm(LRFSHMClient * lrf, 
	       int type = STATE_LRF, 
	       int mode = SERVER_MODE, 
	       int port = STATE_COMM_DEFAULT_LRF_PORT, 
	       int (*commHandler)(int socket, SHMClient * shm) = NULL,
	       char * hostname = NULL);*/
  void init(int type, int mode, int port, 
	    int (*commHandler)(int socket, SHMClient * shm), char * hostname);
  ~StateCommShm(){}
  int start();
  int stop();

  void commActive(bool s) { _commActive = s; }
  bool commActive() { return _commActive;}
  void commRun(bool s) { _commRun = s; }
  bool commRun() { return _commRun;}
 
  int getMode() { return _mode; }
  int getType() { return _type; }
  char * getHostname() { return _hostname; }
  pthread_t  getCommThreadID() { return _commThreadID;}
  pthread_t getMainThreadID() { return _mainThreadID; }
  int portNum() { return _portNum; }
  SHMClient * getShm() { return _shm; } 
  //LRFSHMClient * getLRFShm() {return _lrf; }
 private:
  pthread_t _commThreadID;
  pthread_t _mainThreadID;
  bool _commActive;
  bool _commRun; 
  int _portNum;
  char _hostname[256];
  int _mode;
  int _type;
  
  SHMClient *  _shm;
  //LRFSHMClient * _lrf;
};




#endif // __STATE_COMM_SHM_H__
