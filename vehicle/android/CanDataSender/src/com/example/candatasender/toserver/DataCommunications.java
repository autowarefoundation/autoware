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
