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

package com.example.candatasender.toserver;

import java.util.HashMap;
import java.util.Map;

import android.util.Log;

//シングルトン
//開始、停止はToSeverのconnect(),disconnect()で行う。
public class DataCommunications {
	private static DataCommunications DcInstance = new DataCommunications();
	
	private DataCommunications() {}
	public static DataCommunications getInstance() {
		return DcInstance;
	}
	
	//sd,rdタグ
	private final String tagSd = "send_data";
	private final String tagRd= "response_data";
	
	public String getTagSd() {
		return tagSd;
	}

	public String getTagRd() {
		return tagRd;
	}

	//送信データインスタンス、受信データインスタンス
	//サーバから帰ってきたデータはrdに入れる
	public synchronized void addSendData(Map<String,Object> sd,Map<String,Object> rd) {
		Log.i("MethodCall","DataCommunications.addSendData");
		Map<String,Map<String,Object>> element = new HashMap<String, Map<String, Object>>();
		element.put(getTagSd(), sd);
		element.put(getTagRd(), rd);
		ToServer.getInstance().sendDataOnThread(element);
		while(ToServer.getInstance().GetRcsdLock()) {
			try {
				Thread.sleep(10);
			} catch(Exception e) {
				e.printStackTrace();
			}
		}
	}

	public synchronized void addSendDataNoWait(Map<String,Object> sd,Map<String,Object> rd) {
		Log.i("MethodCall","DataCommunications.addSendDataBg");
		Map<String,Map<String,Object>> element = new HashMap<String, Map<String, Object>>();
		element.put(getTagSd(), sd);
		element.put(getTagRd(), rd);

		while(ToServer.getInstance().GetRcsdLock()) {
			try {
				Thread.sleep(10);
			} catch(Exception e) {
				e.printStackTrace();
			}
		}
		ToServer.getInstance().sendDataOnThread(element);
	}
}
