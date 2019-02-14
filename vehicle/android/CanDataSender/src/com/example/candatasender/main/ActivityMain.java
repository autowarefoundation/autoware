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

package com.example.candatasender.main;

import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.IBinder;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.DialogInterface;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.SharedPreferences.Editor;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.ListView;
import android.util.Log;

import com.example.candatasender.R;
import com.example.candatasender.portforward.ActivityPortforward;
import com.example.candatasender.portforward.PortforwardData;
import com.example.candatasender.toserver.ToServer;
import com.example.candatasender.service.CanDataAutoSend;
import com.example.useful.*;
//import com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverService;
//import com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverServiceCallbackListener;

public class ActivityMain extends Activity {
	private String tableName  = null;
	private String dstDirName = null;
	private final int A_TABLELIST = 0x1111;
	//private ICarLinkDataReceiverService sampleDataRecvService;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		Log.d("MethodCall", "ActivityMain.onCreate");
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_listview);
		//sampleDataRecvService = null;
	}

	@Override
	public void onResume() {
		Log.d("MethodCall", "ActivityMain.onResume");
		super.onResume();
		initMainMenu();
	}

	@Override
	protected void onPause() {
		super.onPause();
	}

	@Override
	protected void onDestroy() {
		super.onDestroy();
	}

	/**
	 * 初期化
	 */
	private void initMainMenu() {
		Log.d("MethodCall", "ActivityMain.initMainMenu");

		String[] menu = {
				"サーバ接続：" + getConnectedServer(),
				"データ登録先テーブル：" + getTableName(),
				"RAWデータ送信先ディレクトリ：" + getRawDataDstDir(),
				"",
				"データ自動登録",
				"既存データ登録",
				"既存RAWデータ送信",
				"",
				"終了",
		};
		ArrayAdapter<String> adpt = new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1, menu);
		ListView lv = (ListView)findViewById(R.id.lv_menu);
		lv.setAdapter(adpt);
		lv.setOnItemClickListener(new AdapterView.OnItemClickListener() {
			@Override
			public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
				ListView list = (ListView)parent;
				String item = (String)list.getItemAtPosition(position);
				if(item.startsWith("サーバ接続：")) {
					onClkServer();
				}
				else if(item.startsWith("データ登録先テーブル：")) {
					onClkTable();
				}
				else if(item.startsWith("RAWデータ送信先ディレクトリ：")) {
					onClkDstDir();
				}
				else if(item.equals("データ自動登録")) {
					onClkAutoDataSend();
				}
				else if(item.equals("既存データ登録")) {
					onClkExistDataSend(false);
				}
				else if(item.equals("既存RAWデータ送信")) {
					onClkExistDataSend(true);
				}
				else if(item.equals("終了")) {
					//終了処理
					onClkItemExitDataSender();
					//アプリ終了
					moveTaskToBack(true);
				}
			}
		});
	}

	/**
	 * 接続中のサーバ名取得
	 */
	private String getConnectedServer() {
		String str = "未接続";
		if (ToServer.getInstance().isConnected()) {
			str = PortforwardData.GetConnectedSetting().split(",")[0];
		}
		return str;
	}

	/**
	 *  データ登録先テーブル名取得
	 */
	private String getTableName() {
		String str = tableName;
		if (str == null) str = "未設定";
		return str;
	}

	/**
	 * RAWデータ送信先ディレクトリ名取得
	 */
	private String getRawDataDstDir() {
		String str = dstDirName;
		if (str == null) str = "未設定";
		return str;
	}

	/**
	 * 「サーバ接続」がクリックされたとき
	 */
	private void onClkServer() {
		Intent intent = new Intent(this, ActivityPortforward.class);
		startActivity(intent);
	}

	/**
	 * 「登録先デーブル」がクリックされたとき
	 */
	private void onClkTable() {
		if(ToServer.getInstance().isConnected()) {
			Intent intent = new Intent(this, ActivityTableList.class);
			startActivityForResult(intent, A_TABLELIST);
		} else {
			ShortAlertDialog.sad("Error Message", "サーバーに接続してください", this);
		}
	}

	/**
	 * Activity結果取得
	 */
	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent intent) {
		if (resultCode != Activity.RESULT_OK) return;
		switch (requestCode) {
		case A_TABLELIST:
			tableName = intent.getStringExtra("table");
			break;
		}
	}

	/**
	 * 「RAWデータ送信先ディレクトリ」がクリックされたとき
	 */
	private void onClkDstDir() {
		Log.d("MethodCall", "ActivityMain.selectDir");

		final LayoutInflater inflater = LayoutInflater.from(this);
		final View view = inflater.inflate(R.layout.raw_data_setting, null);
		final EditText editText = (EditText)view.findViewById(R.id.editRawDataDir);
		editText.setText(getRawDataDstDirPrev());

		AlertDialog.Builder adb = new AlertDialog.Builder(this);
		adb.setTitle("RAWデータの送信先ディレクトリ名を入力してください");
		adb.setView(view);
		adb.setPositiveButton("設定", new DialogInterface.OnClickListener() {
			@Override
			public void onClick(DialogInterface dialog, int which) {
				dstDirName = editText.getText().toString();
				setRawDataDstDirCurrent(dstDirName);
				Log.i("carlinklog", "dstDir: " + dstDirName);

				initMainMenu();
			}
		});
		adb.setNegativeButton("キャンセル", null);
		adb.show();
	}

	private String getRawDataDstDirPrev() {
		SharedPreferences sp = getPreferences(MODE_PRIVATE);
		return sp.getString("raw_data_dir", "${HOME}/CanData");
	}

	private void setRawDataDstDirCurrent(String dir) {
		SharedPreferences sp = getPreferences(MODE_PRIVATE);
		Editor e = sp.edit();
		e.putString("raw_data_dir", dir);
		e.commit();
	}

	/**
	 * 「データ自動登録」がクリックされたとき
	 */
	private void onClkAutoDataSend() {
		Log.d("MethodCall", "ActivityMain.onClkAutoDataSend");
		if (!ToServer.getInstance().isConnected()) {
			ShortAlertDialog.sad("Error Message", "サーバに接続してください", this);
			return;
		}
		if (tableName == null) {
			ShortAlertDialog.sad("Error Message", "データ登録先テーブルを設定してください", this);
			return;
		}
		if (dstDirName == null) {
			ShortAlertDialog.sad("Error Message", "RAWデータ送信先ディレクトリを設定してください", this);
			return;
		}
		final Intent intent = new Intent(this, CanDataAutoSend.class);
		AlertDialog.Builder adb = new AlertDialog.Builder(this);
		adb.setTitle("データ自動登録");
		adb.setMessage("サービスを開始しますか？停止しますか？");
		adb.setPositiveButton("開始", new DialogInterface.OnClickListener() {
			@Override
			public void onClick(DialogInterface dialog, int which) {
				intent.putExtra("table", tableName);
				//intent.putExtra("dir", dstDirName);
				intent.putExtra("terminal", getTerminal());
				startService(intent);
			}
		});
		adb.setNegativeButton("停止", new DialogInterface.OnClickListener() {
			@Override
			public void onClick(DialogInterface dialog, int which) {
				stopService(intent);
			}
		});
		adb.show();
	}

	/**
	 * 「既存データXXX」がクリックされたとき
	 */
	private void onClkExistDataSend(boolean raw) {
		if (!ToServer.getInstance().isConnected()) {
			ShortAlertDialog.sad("Error Message", "サーバに接続してください", this);
			return;
		}
		if ((raw == false) && (tableName == null)) {
			ShortAlertDialog.sad("Error Message", "データ登録先テーブルを設定してください", this);
			return;
		}
		if (dstDirName == null) {
			ShortAlertDialog.sad("Error Message", "RAWデータ送信先ディレクトリを設定してください", this);
			return;
		}
		Intent intent = new Intent(this, ActivityDataList.class);
		intent.putExtra("table", tableName);
		intent.putExtra("dir", dstDirName);
		intent.putExtra("raw", raw);
		startActivity(intent);
	}

	private void onClkItemExitDataSender() {
		try {
			//sampleDataRecvService.close();
		} catch(Exception e) {
			e.printStackTrace();
		}
		//サービス停止
		Intent intent = new Intent(this, CanDataAutoSend.class);
		stopService(intent);
		//接続切断
		if(ToServer.getInstance().isConnected()) {
			ToServer.getInstance().disConnect();
			PortforwardData.SetConnectedSetting("");
		}
	}
	
	private String getTerminal() {
		WifiManager wifiManager = (WifiManager)getSystemService(Context.WIFI_SERVICE);
		WifiInfo wifiInfo = wifiManager.getConnectionInfo();
		return wifiInfo.getMacAddress();
	}
	
	/*
	ServiceConnection servConnection = new ServiceConnection() {
		@Override
		public void onServiceConnected(ComponentName className, IBinder binder) {
			sampleDataRecvService = ICarLinkDataReceiverService.Stub.asInterface(binder);
			try {
				sampleDataRecvService.setEventListener(serviceCallback);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		@Override
        public void onServiceDisconnected(ComponentName className) {
			sampleDataRecvService = null;
        }

	};
	
	ICarLinkDataReceiverServiceCallbackListener.Stub serviceCallback =
			new ICarLinkDataReceiverServiceCallbackListener.Stub() {
	};*/


}
