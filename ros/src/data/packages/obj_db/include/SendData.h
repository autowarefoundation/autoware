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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <string>

class SendData{
public:

SendData() :
  serverName("db1.ertl.jp"),port(5700)
    {
    }

 SendData(std::string server,int pt) :
  serverName(server),port(pt)
  {
  }

  ~SendData(){}

  std::string Sender(std::string value){

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

    server.sin_addr.s_addr = inet_addr(serverName.c_str());
    if (server.sin_addr.s_addr == 0xffffffff) {
      struct hostent *host;

      host = gethostbyname(serverName.c_str());
      if (host == nullptr) {
        if (h_errno == HOST_NOT_FOUND) {
          // h_errno is defined in extern
          printf("host not found : %s\n", serverName.c_str());
        } else {

          printf("%s : %s\n", hstrerror(h_errno), serverName.c_str());
        }
        return result;
      }

      addrptr = (unsigned int **)host->h_addr_list;

      while (*addrptr != nullptr) {
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
      if (*addrptr == nullptr) {
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

    if(value == ""){
      fprintf(stderr,"no data\n");
      return result;
    }

    //    printf("send data : %s\n",value.c_str());
    std::cout << "send data : " << value << std::endl;
    n = write(sock, value.c_str(), value.size());
    if (n < 0) {
        perror("write");
        return result;
    }

    //Caution : If server do not close in one communication,loop forever because read() do not return 0.
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

private:
  std::string serverName;
  int port;
};
