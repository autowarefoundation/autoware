/*
 * Copyright 2015 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <errno.h>
#include <fcntl.h>
#include <jni.h>
#include <stdlib.h>

#include <sys/select.h>
#include <sys/socket.h>

#include <netinet/in.h>

JNIEXPORT jint JNICALL
Java_com_ghostagent_SoundManagementNative_socket(JNIEnv *env, jobject obj)
{
	return (jint)socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
}

JNIEXPORT jint JNICALL
Java_com_ghostagent_SoundManagementNative_connect(JNIEnv *env, jobject obj,
						  jint sockfd, jint timeout,
						  jstring address, jint port)
{
	const char *cp;
	struct sockaddr_in addr;
	int flags;
	int ret;
	fd_set wset;
	struct timeval tv;
	int error;
	socklen_t len;

	cp = (*env)->GetStringUTFChars(env, address, NULL);
	if (cp == NULL)
		return -1;

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = inet_addr(cp);

	(*env)->ReleaseStringUTFChars(env, address, cp);

	flags = fcntl(sockfd, F_GETFL);
	if (flags < 0)
		return -1;

	if (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) < 0)
		return -1;

	ret = connect(sockfd, (struct sockaddr *)&addr, sizeof(addr));
	if (ret == 0) {
		fcntl(sockfd, F_SETFL, flags);
		return 0;
	}
	if (errno != EINPROGRESS) {
		fcntl(sockfd, F_SETFL, flags);
		return -1;
	}

	FD_ZERO(&wset);
	FD_SET(sockfd, &wset);

	memset(&tv, 0, sizeof(tv));
	tv.tv_sec = (time_t)timeout;

	if (select(sockfd + 1, NULL, &wset, NULL, &tv) <= 0) {
		fcntl(sockfd, F_SETFL, flags);
		return -1;
	}

	len = sizeof(error);
	ret = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &error, &len);
	if (ret < 0 || error != 0) {
		fcntl(sockfd, F_SETFL, flags);
		return -1;
	}

	fcntl(sockfd, F_SETFL, flags);
	return 0;
}

JNIEXPORT jint JNICALL
Java_com_ghostagent_SoundManagementNative_close(JNIEnv *env, jobject obj,
						jint sockfd)
{
	return (jint)close(sockfd);
}

JNIEXPORT void JNICALL
Java_com_ghostagent_SoundManagementNative_sendInt(JNIEnv *env, jobject obj,
						  jint sockfd, jint timeout,
						  jint arg0)
{
	fd_set wset;
	struct timeval tv;
	int buf[1];

	FD_ZERO(&wset);
	FD_SET(sockfd, &wset);

	memset(&tv, 0, sizeof(tv));
	tv.tv_sec = (time_t)timeout;

	if (select(sockfd + 1, NULL, &wset, NULL, &tv) <= 0)
		return;

	buf[0] = arg0;

	send(sockfd, buf, sizeof(buf), 0);
}

JNIEXPORT void JNICALL
Java_com_ghostagent_SoundManagementNative_sendIntTuple(JNIEnv *env,
						       jobject obj,
						       jint sockfd,
						       jint timeout,
						       jint arg0,
						       jint arg1)
{
	fd_set wset;
	struct timeval tv;
	int buf[2];

	FD_ZERO(&wset);
	FD_SET(sockfd, &wset);

	memset(&tv, 0, sizeof(tv));
	tv.tv_sec = (time_t)timeout;

	if (select(sockfd + 1, NULL, &wset, NULL, &tv) <= 0)
		return;

	buf[0] = arg0;
	buf[1] = arg1;

	send(sockfd, buf, sizeof(buf), 0);
}

JNIEXPORT void JNICALL
Java_com_ghostagent_SoundManagementNative_sendDoubleArray(JNIEnv *env,
							  jobject obj,
							  jint sockfd,
							  jint timeout,
							  jdoubleArray arg0)
{
	fd_set wset;
	struct timeval tv;
	jdouble *buf;

	FD_ZERO(&wset);
	FD_SET(sockfd, &wset);

	memset(&tv, 0, sizeof(tv));
	tv.tv_sec = (time_t)timeout;

	if (select(sockfd + 1, NULL, &wset, NULL, &tv) <= 0)
		return;

	buf = (*env)->GetDoubleArrayElements(env, arg0, NULL);
	if (buf == NULL)
		return;

	send(sockfd, buf,
	     sizeof(jdouble) * (*env)->GetArrayLength(env, arg0), 0);

	(*env)->ReleaseDoubleArrayElements(env, arg0, buf, 0);
}

JNIEXPORT jint JNICALL
Java_com_ghostagent_SoundManagementNative_recvInt(JNIEnv *env, jobject obj,
						  jint sockfd, jint timeout)
{
	fd_set rset;
	struct timeval tv;
	int buf[1];
	ssize_t nbytes;

	FD_ZERO(&rset);
	FD_SET(sockfd, &rset);

	memset(&tv, 0, sizeof(tv));
	tv.tv_sec = (time_t)timeout;

	if (select(sockfd + 1, &rset, NULL, NULL, &tv) <= 0)
		return -1;

	nbytes = recv(sockfd, buf, sizeof(buf), 0);
	if (nbytes != sizeof(buf))
		return -1;

	return (jint)buf[0];
}

JNIEXPORT jint JNICALL
Java_com_ghostagent_SoundManagementNative_recvNDT(JNIEnv *env, jobject obj,
						  jint sockfd, jint timeout)
{
	fd_set rset;
	struct timeval tv;
	int buf[6];
	ssize_t nbytes;

	FD_ZERO(&rset);
	FD_SET(sockfd, &rset);

	memset(&tv, 0, sizeof(tv));
	tv.tv_sec = (time_t)timeout;

	if (select(sockfd + 1, &rset, NULL, NULL, &tv) <= 0)
		return -1;

	nbytes = recv(sockfd, buf, sizeof(buf), 0);
	if (nbytes != sizeof(buf))
		return -1;

	if (buf[1] > 5)
		return 0;
	else
		return 1;
}
