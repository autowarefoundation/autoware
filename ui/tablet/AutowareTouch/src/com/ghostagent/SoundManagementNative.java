package com.ghostagent;

public class SoundManagementNative {
	static{
		System.loadLibrary("soundsender");
	}
	public static native int socket();
	public static native int connect(int sockfd, String address, int port);
	public static native int close(int sockfd);
	public static native void sendInt(int sockfd, int arg0);
	public static native void sendIntTuple(int sockfd, int arg0, int arg1);
	public static native void sendDoubleArray(int sockfd, double arg0[]);
	public static native int recvInt(int sockfd);
}
