#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "brake_up_slope.h" // brake pedal values defined in this file
#define MAX_INDEX_BRAKE 4000 // the number of index of array which contains brake pedal stroke values

#include "accel_up_slope.h" // accel pedal values defined in this file
#define MAX_INDEX_ACCEL 1500    // the number of index of array which contains accel pedal stroke values

/* server info */
#define PORT_NUM 12347

/* socket for server */
static int server_sock;
/* socket for client */
static int client_sock;

/* prius control protocol */
typedef enum Prius_control_protocol_enum {
    DISCONNECT = 0,
    BRAKE = 1,
    SAFETY = 2   
} Prius_ctl_prt;

void getConnect();
void disConnect();
void autoSmartStart();
void autoSmartStop();

void MainWindow::autoActivate(void)
{

  char recv_buf[4];             // braking data is int type
  int detection_res;
  int recv_size = 0;

  // connect
  getConnect();

  // while reiving
  for (;;) {
    recv_size = recv(client_sock, recv_buf, sizeof(recv_buf), 0);
    if (recv_size == -1) {      // error semantics
      break;
    }
    detection_res = atoi(recv_buf);

    /* control prius according to detection result */
    switch(detection_res) {
    case DISCONNECT:
      disConnect();             // disconnect
      printf("disconnected client.\n");
      exit(1);
      break;
    case BRAKE:
      //autoSmartStop();
      printf("BRAKE\n");
      break;
    case SAFETY:
      //autoSmartCleep();
      //usleep(500*1000);
      //autoSmartStart();
      printf("SAFETY\n");
      break;
    }

  }


}

void getConnect(void)
{
  int server_addrlength, client_addrlength;
  struct sockaddr_in server_addr, client_addr;

  /* create socket for server */
  server_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (server_sock == -1) {
    printf("Can't create socket.\n");
    exit(1);
  }

  /* configure socket */
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(PORT_NUM);
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addrlength = sizeof(server_addr);

  /* name socket (bind) */
  if ( bind(server_sock, (struct sockaddr *)&server_addr, server_addrlength) == -1)
    {
      printf("Can't assign the address to a socket.\n");
      exit(1);
    }
  
  /* wait for connection from client */
  /* the maximum number of waiting : 2 */
  if ( listen(server_sock, 2) == -1 )
    {
      printf("Can't listen for connections.\n");
      exit(1);
    }
  printf("listing...\n");
  
  /* accept connection */
  //  client_sock = accept(server_sock, (struct sockaddr *)&client_addr, (socklen_t *)(&client_addrlength));
  client_sock = accept(server_sock, (struct sockaddr *)&server_addr, (socklen_t *)(&server_addrlength));
  if (client_sock == -1) {
    printf("Can't accept a connection on the socket.\n");
    exit(1);
  }
  printf("accept.\n");

  return;
  
}

void disConnect(void)
{
  /* close socket for server */
  close(server_sock);
  /* close socket for client */
  close(client_sock);

  printf("disconnect.\n");
}


void MainWindow::autoSmartStart(void)
{
  //  hev->SetBrakeStroke(0);
  /* send accel pedal values according to log */
  for (int i=0; i<MAX_INDEX_ACCEL; i++) {
     hev->SetDrvStroke(__accel_up_slope[i]);
     usleep(1000);
  }
}

void MainWindow::autoSmartStop(void)
{
  /* send brake pedal values according to log */
  for (int i=0; i<MAX_INDEX_BRAKE; i++) {
    hev->SetBrakeStroke(__brake_up_slope[i]);
    usleep(1000);
  }
}

void MainWindow::autoSmartCleep(void)
{
  hev->SetDrvStroke(0);
  hev->SetBrakeStroke(0);
}
