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
