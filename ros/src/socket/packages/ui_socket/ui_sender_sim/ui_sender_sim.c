#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

int main(int argc, char *argv[])
{
	int sock;
	struct sockaddr_in sin;
	in_addr_t addr;
	int info[2];
	int signal;
	int i, j, count = 100;
	uint16_t port = 12345;

	if (argc >= 2) {
		port = (int)strtoul(argv[1], NULL, 0);
		if (argc >= 3) {
			count = (int)strtoul(argv[2], NULL, 0);
		}
	}

	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		perror("socket");
		return 1;
	}

	memset(&sin, 0, sizeof(sin));
	sin.sin_family = AF_INET;
	sin.sin_port = htons(port);
	addr = inet_addr("127.0.0.1");
	memcpy(&sin.sin_addr, &addr, sizeof(sin.sin_addr));
	if (connect(sock, (struct sockaddr *)&sin, sizeof(sin)) == -1) {
		perror("connect");
		return 2;
	}

	for (i = 0, j = 0; i < count; i++) {
		j = (j+1) & 1;
		info[0] = j+1;
		info[1] = i;
		if (send(sock, info, sizeof(info), 0) == -1) {
			perror("send");
			break;
		}
		if (recv(sock, &signal, sizeof(signal), 0) == -1) {
			perror("recv");
			break;
		}
		fprintf(stderr, "get signal %d\n", signal);
	}
	info[0] = 0;
	send(sock, info, sizeof(info), 0);

	close(sock);

	return 0;
}
