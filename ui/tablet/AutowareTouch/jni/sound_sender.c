#include <jni.h>
#include <android/log.h>

#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define LOG_TAG	"ghostagent"
#define LOGI(...)	__android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)	__android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

//Global variable
int sock = -1;

// Convert Endian
int convertEndian(void *input, size_t s);

JNIEXPORT jint JNICALL Java_com_ghostagent_SoundManagementNative_connect(JNIEnv * env, jobject obj, jstring address, jint port_number) {
	struct sockaddr_in server;
	const char *address_number = (*env)->GetStringUTFChars(env, address, 0);

	/* create socket */
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		return -1;
	}

	/* Preparation of the structure for the specified destination */
	server.sin_family = AF_INET;
	server.sin_port = htons(port_number);
	server.sin_addr.s_addr = inet_addr(address_number);

	/* connect to server */
	if (connect(sock, (struct sockaddr *)&server, sizeof(server)) == -1) {
		close(sock);
		sock = -1;
		return -1;
	}
	LOGI("connect", "connected");
	return 0;
}

JNIEXPORT jint JNICALL Java_com_ghostagent_SoundManagementNative_close(JNIEnv * env) {
	close(sock);
	sock = -1;
	LOGI("close", "done");
	return 0;
}


JNIEXPORT jint JNICALL Java_com_ghostagent_SoundManagementNative_send(JNIEnv * env, jobject thiz, jint type, jint data) {
	int sdata[2];
	jint ret;

	if (sock < 0) {
		return -1;
	}

	sdata[0] = type;
	sdata[1] = data;
	if (send(sock, sdata, sizeof(sdata), 0) == -1) {
		LOGE("sendSoundData", "send failed");
		return -1;
	}

	ret = recv(sock, sdata, sizeof(int), 0);
	if (ret == -1) {
		LOGE("sendSoundData", "recv failed");
	}
	return (ret != -1) ? sdata[0]:ret;
}

JNIEXPORT jint JNICALL Java_com_ghostagent_SoundManagementNative_sendDoubleArray(JNIEnv * env, jobject thiz, jint type, jdoubleArray data) {
	int sdata[2];
	jint ret;
	jdouble * array;
	int array_size;

	if (sock < 0) {
		return -1;
	}

	array = (*env)->GetDoubleArrayElements(env, data, NULL);
	if (array == NULL) {
		return -1;
	}

	array_size = (*env)->GetArrayLength(env, data);

	sdata[0] = type;
	sdata[1] = sizeof(jdouble) * array_size;
	if (send(sock, sdata, sizeof(sdata), 0) == -1) {
		LOGE("sendSoundData", "send failed");
		goto fail;
	}

	if (send(sock, array, sdata[1], 0) == -1) {
		LOGE("sendSoundData", "send failed");
		goto fail;
	}

	ret = recv(sock, sdata, sizeof(int), 0);
	if (ret == -1) {
		LOGE("sendSoundData", "recv failed");
		goto fail;
	}

	(*env)->ReleaseDoubleArrayElements(env, data, array, 0);
	return sdata[0];

fail:
	(*env)->ReleaseDoubleArrayElements(env, data, array, 0);
	return -1;
}

int convertEndian(void *input, size_t s){
	int i;   // counter
	unsigned char *temp;   // temp

	if((temp = (char *)calloc( s, sizeof(unsigned char))) == NULL){
		return 0;   // error
	}

	for(i=0; i<s; i++){   // save input to temp
		temp[i] = ((unsigned char *)input)[i];
	}

	for(i=1; i<=s; i++){   // reverse
		((unsigned char *)input)[i-1] = temp[s-i];
	}

	free(temp);   // free

	return 1;   // finish
}

