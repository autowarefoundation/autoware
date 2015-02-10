package com.ghostagent;

public class SoundManagementNative {
	static{
		System.loadLibrary("soundsender");
	}
	public static native int socket();
	public static native int connect(int sockfd, int timeout, String address, int port);
	public static native int close(int sockfd);
	public static native void sendInt(int sockfd, int timeout, int arg0);
	public static native void sendIntTuple(int sockfd, int timeout, int arg0, int arg1);
	public static native void sendDoubleArray(int sockfd, int timeout, double arg0[]);
	public static native int recvInt(int sockfd, int timeout);
}
