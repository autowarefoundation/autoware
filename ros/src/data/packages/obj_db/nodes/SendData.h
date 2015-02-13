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


class SendData{
    
public:
    
SendData() : 
  serverName("db1.ertl.jp"),port(5700),value("")
    {
    }
  /*    
 SendData(int flag,char server,int pt) : 
  counter(0),testFlag(flag),serverName(server),port(pt),value(NULL),valueSize(0)
  {
  }
  */

 SendData(char *server,int pt) : 
  serverName(server),port(pt),value("")
  {
  }

    
  ~SendData(){}
    
  std::string Sender(){
        
      
    /*********************************************
   format data to send
    *******************************************/
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
    server.sin_port = htons(port); // HTTP port is 80 

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

    /*
    if(testFlag == 1){
        sprintf(data,"select order\t%d",counter);
        printf("test");
    }else{
        sprintf(data,"select order");
    }
    */

    if(value == ""){
      fprintf(stderr,"no data\n");
      return result;
    }

    printf("send data : %s\n",value.c_str());
    n = write(sock, value.c_str(), value.size());
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
	return result;
      }else if(n==0){
	break;
      }

      if(strlen(recvdata)>=sizeof(recvdata)){
	result.append(recvdata,sizeof(recvdata));
      }else{
	result.append(recvdata,strlen(recvdata));
      }
    }
    
    close(sock);
    
    return result;
    
  }
  void setServerName(char * server){
    serverName = server;
  }

  void setPort(int pt){
    port = pt;
  }

  void setValue(std::string v){
    value = v;
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

  char *serverName;
  int port;
  std::string value;
 
};
