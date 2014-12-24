#include "state_comm_shm.h"
#include <iostream>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include "signal.h"
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>                                                       
#include <sstream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

using namespace std;

StateCommShm * comm_GLOBAL;
int (*commHandler_GLOBAL)(int socket, SHMClient * shm);
static bool stop_recv = false;

static int socket_OK;

void server_sigpipe_handler(int q __attribute__((unused)))
{
  fprintf(stderr,"STATE_COMM_SHM: SIGPIPE caught\n");
  socket_OK = 1;
}



//-------------------------------------------------------------
//
// clientConnectSocket
//
//
//------------------------------------------------------------
int clientConnectSocket(char * hostname, int port_num) 
{
  struct sockaddr_in server;
  int sock_fd, ret;

  sock_fd = socket(AF_INET, SOCK_STREAM, 0);
  server.sin_family = AF_INET;
  server.sin_port = htons(port_num);
  server.sin_addr.s_addr = inet_addr(hostname);
  fprintf(stderr,"calling connect on %s:%d\n", hostname, port_num);
  if ((ret = connect(sock_fd, (struct sockaddr *)&server, 
		     sizeof(server))) < 0) {
    fprintf(stderr, "clientConnectSocket() could not connect to %d at %s, ret = %d, errno = %d\n", port_num, hostname, ret, errno);
    exit(1);
  }
  fprintf(stderr,"connect succefull returning socket %d\n", sock_fd);
  return sock_fd;
}

//-------------------------------------------------------------
//
// serverOpenSocket
//
//
//------------------------------------------------------------
int serverOpenSocket(int * sock, int port_num)
{
  *sock = socket(AF_INET, SOCK_STREAM, 0);
  

  struct sockaddr_in addr, client;
  struct timeval timeout;

  // set to non-blocking read
  // int flags = fcntl(*sock, F_GETFL, 0);
  //  fcntl(*sock, F_SETFL, flags | O_NONBLOCK);
  //  fd_set rdfds;
  ///  timeout.tv_sec = 1;
  //  timeout.tv_usec = 0;

  memset(&addr, 0, sizeof(struct sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_num);
  addr.sin_addr.s_addr = INADDR_ANY;
  
  bind(*sock, (struct sockaddr *)&addr, sizeof(addr));
 
  
  fprintf(stderr,"serverOpenSocket() calling listen on sock %d, port %d\n", *sock, port_num);
  if((listen(*sock, 10)) < 0) {
    fprintf(stderr,"serverOpenSocket() listen error for port %d\n", port_num);
  }
  fprintf(stderr,"after listen...\n");


  
  
  //  fprintf(stderr,"select_exited\n");
  int len = sizeof(client);
  fprintf(stderr,"calling accept\n");
  int ret = accept(*sock, 
		   (struct sockaddr *)&client,
		   (socklen_t *)&len);
  
  fprintf(stderr,"after accept ret = %d, sock = %d\n", ret, *sock);

  
  return ret;
}




//------------------------------------------------------------
//
//         startComm
//
//
//------------------------------------------------------------
void  startComm(void)
{
  int socket, socket_fd;
  
 tag1:;
  
  //  signal(SIGPIPE, server_sigpipe_handler);
  signal(SIGPIPE, SIG_IGN);

  int port_num = comm_GLOBAL->portNum();
  int mode = comm_GLOBAL->getMode();
  int type = comm_GLOBAL->getType();

  SHMClient * shm = comm_GLOBAL->getShm();

  
  if (mode == SERVER_MODE){
    socket_fd = serverOpenSocket(&socket, port_num);
    socket_OK =1;

    fprintf(stderr,"startComm() server opened socket %d with fd %d\n", socket, socket_fd);
  }
  else if (mode == CLIENT_MODE) {
    socket_fd = clientConnectSocket(comm_GLOBAL->getHostname(), port_num);
    socket_OK = 1;
  }
  

  int status;;
   
  
  comm_GLOBAL->commActive(true);
  comm_GLOBAL->commRun(true);
  fprintf(stderr,"entering while loop\n");


  while (comm_GLOBAL->commRun()){ // send loop

    if (mode == SERVER_MODE && !socket_OK){
      fprintf(stderr,"WARNING socket not OK\n");
      break;

    }
     fprintf(stderr,"calling commHandler_GLOBAL\n");
    // execute user supplied function to handle comms
    status = commHandler_GLOBAL(socket_fd, shm);

    if (status == 0) {
      if (mode == SERVER_MODE) {
	fprintf(stderr,"trouble in handler - restart\n");

	close(socket_fd);
	close(socket);

	goto tag1;
      }
    }

    if  (mode == CLIENT_MODE) {
      // reading socket 
      if (type == STATE_VEL_CMD)
	usleep (10000);
      else
	usleep(5000);
    }
    else { // server
      // writing socket
      if (type ==STATE_VEL_CMD)
	usleep(5000);
      else
	usleep(10000);
    }
   
  } // while - send loop
  fprintf(stderr,"after while loop\n");
 close(socket_fd);
  
 if (mode == SERVER_MODE) 
    close(socket);
  comm_GLOBAL->commActive(false);

}



//------------------------------------------------------------
//
//             ServerShm : a socket server for accessing shm data
//
//  constructor
//
//------------------------------------------------------------
StateCommShm::StateCommShm(SHMClient * shm, int type, int mode, int port, int (*commHandler) (int socket, SHMClient * shm), char * hostname)
{
 
  _shm = shm;
  // _lrf = NULL;
  init(type, mode, port, commHandler, hostname);

 }
/*StateCommShm::StateCommShm(LRFSHMClient * lrf, int type, int mode, int port, int (*commHandler) (int socket, SHMClient * shm), char * hostname)
{
  
  if (mode != STATE_LRF) {
    cerr << "StateServerShm Contstuctor: shm mode mismatch (lrf shm with " << mode << " ... exiting" << endl;
      exit(1);
  }
  
  _shm = NULL;
  _lrf = lrf;

  init(type, mode, port, commHandler, hostname);

 }
*/

void StateCommShm::init(int type, int mode, int port, int (*commHandler)(int socket, SHMClient * shm), char * hostname)
{
 comm_GLOBAL = this; 
 _type = type;
 _mode = mode;
  _portNum = port;

  if (_mode == CLIENT_MODE && hostname != NULL) {
    strcpy(_hostname ,hostname);
  }

  commHandler_GLOBAL = commHandler;
  commActive(false);
  commRun(false);

}


//------------------------------------------------------------
//
//             ServerShm
//   
//
//  start()
//
//------------------------------------------------------------
int StateCommShm::start()
{

  //  timer.start();
  int status;
  cerr << "StateCommShm::start()... " << endl; 
  status = pthread_create(&_commThreadID, NULL, 
			  (void*(*)(void*))(startComm), NULL);
  
    
  return(1);
}

//------------------------------------------------------------
//
//             StateServerShm
//
//  stop()
//
//------------------------------------------------------------
int StateCommShm::stop()
{
  
  int status;
  cerr << "StateCommShm::stop()... " << endl;
  if (commActive()) 
    commRun(false);
  
  fprintf(stderr,"waiting for comm to deactivate\n");
  int count = 0;
  while (commActive() && count < 10){
    fprintf(stderr,"Comm remains active\n");
    usleep(100000);
    count++;
  }
  fprintf(stderr,"calling exit --- comm (comm sticks in connect if no connection)\n");
  exit(1);
}
