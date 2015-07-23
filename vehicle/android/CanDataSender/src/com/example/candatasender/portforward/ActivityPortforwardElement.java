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
