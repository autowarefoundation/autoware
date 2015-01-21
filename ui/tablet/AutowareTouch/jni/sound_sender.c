#include <jni.h>
#include <stdlib.h>

#include <sys/socket.h>

#include <netinet/in.h>

JNIEXPORT jint JNICALL
Java_com_ghostagent_SoundManagementNative_socket(JNIEnv *env, jobject obj)
{
	return (jint)socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
}

JNIEXPORT jint JNICALL
Java_com_ghostagent_SoundManagementNative_connect(JNIEnv *env, jobject obj,
						  jint sockfd, jstring address,
						  jint port)
{
	const char *cp;
	struct sockaddr_in addr;

	cp = (*env)->GetStringUTFChars(env, address, NULL);
	if (cp == NULL)
		return -1;

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = inet_addr(cp);

	(*env)->ReleaseStringUTFChars(env, address, cp);

	return (jint)connect(sockfd, (struct sockaddr *)&addr, sizeof(addr));
}

JNIEXPORT jint JNICALL
Java_com_ghostagent_SoundManagementNative_close(JNIEnv *env, jobject obj,
						jint sockfd)
{
	return (jint)close(sockfd);
}

JNIEXPORT void JNICALL
Java_com_ghostagent_SoundManagementNative_sendInt(JNIEnv *env, jobject obj,
						  jint sockfd, jint arg0)
{
	int buf[1];
	uint8_t *bufp;
	size_t bufsize;
	ssize_t nbytes;

	buf[0] = arg0;

	bufp = (uint8_t *)buf;
	bufsize = sizeof(buf);
	while (bufsize) {
		nbytes = send(sockfd, bufp, bufsize, 0);
		if (nbytes < 0) {
			break;
		}
		bufp += nbytes;
		bufsize -= nbytes;
	}
}

JNIEXPORT void JNICALL
Java_com_ghostagent_SoundManagementNative_sendIntTuple(JNIEnv *env,
						       jobject obj,
						       jint sockfd,
						       jint arg0,
						       jint arg1)
{
	int buf[2];
	uint8_t *bufp;
	size_t bufsize;
	ssize_t nbytes;

	buf[0] = arg0;
	buf[1] = arg1;

	bufp = (uint8_t *)buf;
	bufsize = sizeof(buf);
	while (bufsize) {
		nbytes = send(sockfd, bufp, bufsize, 0);
		if (nbytes < 0) {
			break;
		}
		bufp += nbytes;
		bufsize -= nbytes;
	}
}

JNIEXPORT void JNICALL
Java_com_ghostagent_SoundManagementNative_sendDoubleArray(JNIEnv *env,
							  jobject obj,
							  jint sockfd,
							  jdoubleArray arg0)
{
	jdouble *buf;
	uint8_t *bufp;
	size_t bufsize;
	ssize_t nbytes;

	buf = (*env)->GetDoubleArrayElements(env, arg0, NULL);
	if (buf == NULL)
		return;

	bufp = (uint8_t *)buf;
	bufsize = sizeof(jdouble) * (*env)->GetArrayLength(env, arg0);
	while (bufsize) {
		nbytes = send(sockfd, bufp, bufsize, 0);
		if (nbytes < 0) {
			break;
		}
		bufp += nbytes;
		bufsize -= nbytes;
	}

	(*env)->ReleaseDoubleArrayElements(env, arg0, buf, 0);
}

JNIEXPORT jint JNICALL
Java_com_ghostagent_SoundManagementNative_recvInt(JNIEnv *env, jobject obj,
						  jint sockfd)
{
	int buf[1];
	uint8_t *bufp;
	size_t bufsize;
	ssize_t nbytes;

	bufp = (uint8_t *)buf;
	bufsize = sizeof(buf);
	while (bufsize) {
		nbytes = recv(sockfd, bufp, bufsize, 0);
		if (nbytes < 0) {
			buf[0] = -1;
			break;
		}
		bufp += nbytes;
		bufsize -= nbytes;
	}

	return (jint)buf[0];
}
