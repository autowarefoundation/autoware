#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <vector>
#include <string>


long startTime[10] = {
    1255698868,
    1255723190,
    1266425473,
    1266765883,
    1266851714,
    1266938129,
    1267471638,
    1267542268,
    1267715826,
    1268755256
};

class SendData{
    
public:
    
SendData() : 
    counter(0),testFlag(0),serverName(NULL)
    {
    }
    
SendData(int flag) : 
    counter(0),testFlag(flag),serverName(NULL)
    {
    }
    
    ~SendData(){}
    
    std::string Sender(){
        
      
    /*********************************************
   format data to send
    *******************************************/
    char data[256];
    char recvdata[1024];
    int n;
    std::string result = "";

    struct sockaddr_in server;
    int sock;
    unsigned int **addrptr;

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
      perror("socket");
      return result;
    }

    server.sin_family = AF_INET;
    server.sin_port = htons(5678); // HTTP port is 80 

    server.sin_addr.s_addr = inet_addr(serverName);
    if (server.sin_addr.s_addr == 0xffffffff) {
      struct hostent *host;

      host = gethostbyname(serverName);
      if (host == NULL) {
        if (h_errno == HOST_NOT_FOUND) {
          // h_errno is defined in extern 
          printf("host not found : %s\n", serverName);
        } else {

          printf("%s : %s\n", hstrerror(h_errno), serverName);
        }
        return result;
      }

      addrptr = (unsigned int **)host->h_addr_list;

      while (*addrptr != NULL) {
        server.sin_addr.s_addr = *(*addrptr);

        // if connect() is succeed , exit loop 
        if (connect(sock,
                    (struct sockaddr *)&server,
                    sizeof(server)) == 0) {
          break;
        }

        addrptr++;
        // if connect is failed, try next address
      }

      // the case of connection failed 
      if (*addrptr == NULL) {
        perror("connect");
        return result;
      }
    } else {
      if (connect(sock,
                  (struct sockaddr *)&server, sizeof(server)) != 0) {
        perror("connect");
        return result;
      }
    }

    if(testFlag == 1){
        sprintf(data,"select order\t%d",counter);
        printf("test");
    }else{
        sprintf(data,"select order");
    }
    printf("%s\n",data);
    n = write(sock, data, (int)strlen(data));
    if (n < 0) {
        perror("write");
        return result;
    }
        

    //cation : もしサーバ側で一回の通信でクローズするようになっていない場合は
    //readで0が返ってこないのでループから抜けられなくなる
    //If server do not close in one communication,loop forever because read() do not return 0.
    while(1){
        memset(recvdata, 0, sizeof(recvdata));
        n = read(sock, recvdata, sizeof(recvdata));
        if (n < 0) {
            perror("recv error");
            result = "";
            return result;
        }else if(n==0){
            break;
        }
        result += recvdata;    
    }

    if(testFlag == 1){
        counter++;
        if(counter > 5000) counter = 0;
    }

    close(sock);
    
    return result;
    
  }
  void setServerName(char * server){
    serverName = server;
  }

private:

/*
  int gpx_id[10] = {
      545340,
      545531,
      666356,
      666357,
      666358,
      666359,
      666360,
      666361,
      666362,
      666363
  };
*/

//start time by seconds
  //long startTime[10];

  int counter;
  int testFlag;
  char *serverName;
 
};
