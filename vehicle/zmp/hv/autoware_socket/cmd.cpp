#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "../HEV/mainwindow.h"
#include "data.h"

CMDDATA cmddata;

std::vector<std::string> split(const std::string& input, char delimiter)
{
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

void Getter(){
  std::string request;
  std::string cmdRes;
  char recvdata[32];

  struct sockaddr_in server;
  int sock;
  //  char deststr[80] = serverIP.c_str();
  unsigned int **addrptr;

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    perror("socket");
    return;
  }

  server.sin_family = AF_INET;
  server.sin_port = htons(10001); /* HTTPのポートは80番です */

  server.sin_addr.s_addr = inet_addr(rosServerIP.c_str());
  if (server.sin_addr.s_addr == 0xffffffff) {
    struct hostent *host;

    host = gethostbyname(rosServerIP.c_str());
    if (host == NULL) {
      if (h_errno == HOST_NOT_FOUND) {
        /* h_errnoはexternで宣言されています */
        fprintf(stderr,"cmd : host not found : %s\n", rosServerIP.c_str());
      } else {
        /*
          HOST_NOT_FOUNDだけ特別扱いする必要はないですが、
          とりあえず例として分けてみました
        */
        fprintf(stderr,"cmd : %s : %s\n", hstrerror(h_errno), rosServerIP.c_str());
      }
      return;
    }

    addrptr = (unsigned int **)host->h_addr_list;

    while (*addrptr != NULL) {
      server.sin_addr.s_addr = *(*addrptr);

      /* connect()が成功したらloopを抜けます */
      if (connect(sock,
		  (struct sockaddr *)&server,
		  sizeof(server)) == 0) {
	break;
      }

      addrptr++;
      /* connectが失敗したら次のアドレスで試します */
    }
   
    /* connectが全て失敗した場合 */
    if (*addrptr == NULL) {
      perror("cmd : connect");
      return;
    }
  } else {
    if (connect(sock,
		(struct sockaddr *)&server, sizeof(server)) != 0) {
      perror("cmd : connect");
      return;
    }
  }

  int n;

  while (true) {
    memset(recvdata, 0, sizeof(recvdata));
    n = recv(sock, recvdata, sizeof(recvdata),0);
    if (n < 0) {
      perror("cmd : read erro");
      return;
    }else if(n == 0){
      break;
    }
    cmdRes.append(recvdata,n);
  }

  if(cmdRes.compare("no command data") == 0){
    fprintf(stderr,"cmd : Command data is not received.\ncmd : Check autoware topic\n");
  }else{
    std::vector<std::string> cmdVector;
    cmdVector = split(cmdRes,',');
    if(cmdVector.size() == 2){
      cmddata.linear_x = atof(cmdVector[0].c_str());
      cmddata.angular_z = atof(cmdVector[1].c_str());
      printf("cmd : linear:%f angular:%f\n",cmddata.linear_x,cmddata.angular_z);
    }else{
      fprintf(stderr,"cmd : Recv data is invalid\n");
    }


  }

  close(sock);

  printf("cmd : return data : %s\n",cmdRes.c_str());
  return;
}

/*
void CMDGetter(){

  pthread_t _getter;
  while(1){
    if(pthread_create(&_getter, NULL, Getter, NULL)){
      fprintf(stderr,"cmd : pthread create error");
      return;
    }
    pthread_detach(_getter);
    usleep(10*1000);
  }
}
*/

void *MainWindow::CMDGetterEntry(void *a){
  //MainWindow* main = (MainWindow*)a;
  //main->CMDGetter();
  while(1){
    Getter();
    usleep(20*1000);
  }
  return NULL;
}
