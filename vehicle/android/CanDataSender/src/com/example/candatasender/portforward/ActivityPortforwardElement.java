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

package com.example.candatasender.portforward;

import java.util.Map;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;

import com.example.candatasender.main.PrivateAppData;
import com.example.candatasender.sender.DataType;
import com.example.candatasender.toserver.ToServer;
import com.example.candatasender.R;
import com.example.useful.ShortAlertDialog;

import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;


public class ActivityPortforwardElement extends Activity {
	@Override
    protected void onCreate(Bundle savedInstanceState) {
		Log.d("MethodCall", "ActivityPortforwardElement.onCreate(Bundle saveInstanceState);");
        super.onCreate(savedInstanceState);
        initAcitivity();
    }
	
	@Override
	protected void onResume() {
		Log.d("MethodCall", "ActivityPortforward.onResume();");
		super.onResume();		
		initMenuList();
	}
	
	/**
	 * 初期化
	 */
	private void initAcitivity() {
		Log.d("MethodCall", "ActivityPortforward.initActivity();");
		setContentView(R.layout.activity_listview);
	}
	
	/**
	 * メニューの初期化
	 */
	private void initMenuList() {
		Log.d("MethodCall", "ActivityPortforwardElement.initMenuList();");
        
        //メニューアイテムの設定
    	String menu [] = {"接続", "設定変更", "設定削除"};
        
    	//アダプタを設定
        ListView lv = (ListView)findViewById(R.id.lv_menu);
        ArrayAdapter<String> adpt = new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1, menu);
        lv.setAdapter(adpt);
        
        //リストビューのアイテムがクリックされた際のリスナーの登録
        lv.setOnItemClickListener(new AdapterView.OnItemClickListener() {
        	@Override
       		public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
        		ListView list = (ListView)parent;
        		String item = (String)list.getItemAtPosition(position); 
        		if(item.equals("接続")) {
        			onClkItemCnct();
        		} else if(item.equals("設定変更")) {      		
        			onClkItemChangeSetting();
        		} else if(item.equals("設定削除")) {
        			onClkItemDeleteSetting();
        		}
        	}
        });
	}
	
	/***
	 * 「接続」がクリックされたとき
	 */
	private void onClkItemCnct() {
		Log.d("MethodCall", "ActivityPortforward.onClkItemCnct();");
		//既に接続中の接続がある場合
		if(!PortforwardData.GetConnectedSetting().equals("")) {
			String title = "";
			String message = "既に接続中です。再接続する場合は一度、接続の終了をしてください";
			ShortAlertDialog.sad(title, message, this);
			return;
		}
		
		Intent intent = getIntent();
		 //ActivityPortforward.onClickItemCnctで保存したpfData、選択された設定を取得
		PortforwardData pfData = (PortforwardData)intent.getSerializableExtra("pfData");
		//ssh接続
		if(SshTunnel.getInstance().connect(pfData)) {		
			Log.i("log","ActivityPortForwardElement.onClkItemCnct() - Success connect to ssh server");
			int port = Integer.valueOf(pfData.getLocalPort()).intValue();			
			String host = "127.0.0.1";
			ToServer.getInstance().setServerData(host, port);
			Log.i("ServerData", "host:" + host + "\nport:" + port);
			//poststoreと接続
			Map<String,String> rd = ToServer.getInstance().connect();
			if(((String)rd.get("TYPE")).equals(DataType.DT_CONNECTION_SUCCESS.getType())) {
				//共有データに接続状態を保存
				PortforwardData.SetConnectedSetting(pfData.getString());	
				Log.i("log","sd.set_connet:" + pfData.getString());
				Log.i("log", "接続成功");
				finish();
			} else {
				//error
				//ssh接続を切断
				SshTunnel.getInstance().disConnect();
				Log.i("log","ActivityPortforwardingElement - 接続失敗");
				Log.i("log",(String)rd.get("MESSAGE"));
				String title = "Error Message";
				String message = (String)rd.get("MESSAGE");
				ShortAlertDialog.sad(title, message, this);				
			}
		} else {
			//ssh接続を切断
			SshTunnel.getInstance().disConnect();
			Log.i("log","sshサーバとの接続に失敗しました");
			String title = "Error Message";
			String message = "sshサーバとの接続に失敗しました";
			ShortAlertDialog.sad(title,message,this);
		}
	}
	
	/*設定を変更する
	 */
	private void onClkItemChangeSetting() {	
		//ActivityPortforward.onClickItemCnctで保存したpfData、選択された設定を取得
		Intent g_intent = getIntent();		
		PortforwardData pfData = (PortforwardData)g_intent.getSerializableExtra("pfData");
		Intent intent = new Intent(this,ActivityPortforwardSetting.class);
		intent.putExtra("pfData", pfData);
		startActivity(intent);						
	}
	/*設定を削除する	
	 */
	private void onClkItemDeleteSetting() {
		Intent intent = getIntent();
		PortforwardData pfData = (PortforwardData)intent.getSerializableExtra("pfData");
		PrivateAppData.deletePortforwardData(this, pfData);
		finish();
	}
}
