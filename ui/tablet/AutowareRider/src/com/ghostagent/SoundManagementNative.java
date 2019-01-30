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
	public static native int recvNDT(int sockfd, int timeout);
}
