package com.ghostagent;

public class SoundManagementNative {
	static{
		System.loadLibrary("soundsender");
	}
	public static native int connect(String address, int port_number);
	public static native int close();
	public static native int send(int type, int data);
}
