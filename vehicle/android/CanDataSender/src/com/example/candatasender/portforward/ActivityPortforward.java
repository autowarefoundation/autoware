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

import java.util.ArrayList;

import com.example.candatasender.main.*;
import com.example.candatasender.toserver.ToServer;
import com.example.candatasender.R;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;

public class ActivityPortforward extends Activity {
	private final int MAX_SIZE = 16; //最大登録数
	private ArrayList<PortforwardData> pfDataList = new ArrayList<PortforwardData>();
	
	@Override
    protected void onCreate(Bundle savedInstanceState) {
		Log.d("MethodCall", "ActivityPortforward.onCreate(Bundle saveInstanceState);");
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
		Log.d("MethodCall", "ActivityPortforward.initMenuList();");

        pfDataList.clear();
        PrivateAppData.loadPortforwardData(this, pfDataList);
        
        //メニューアイテムの設定
    	String[] menu = new String[pfDataList.size()+2];
    	menu[0] = "接続先の追加";    	
    	//すでに設定があればそれをメニューに追加
        if(pfDataList.size() > 0){
        	for(int i=0; i<pfDataList.size(); i++) {
        		if(pfDataList.get(i).getString().equals(PortforwardData.GetConnectedSetting()) &&
        		   ToServer.getInstance().isConnected()) {
        			menu[i+1] = "接続中 : " + pfDataList.get(i).getName();
        		} else {
        			menu[i+1] = pfDataList.get(i).getName();
        		}
        	}
        }
        menu[menu.length-1] = "接続の終了";
        
        
    	//アダプタを設定
        ListView lv = (ListView)findViewById(R.id.lv_menu);
        ArrayAdapter<String> adpt = new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1, menu);
        lv.setAdapter(adpt);
        
        //リストビューのアイテムがクリックされた際のリスナーの登録
        lv.setOnItemClickListener(new AdapterView.OnItemClickListener() {
        	@Override
       		public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
        		ListView list = (ListView)parent;
        		String item = (String)list.getItemAtPosition(position); //クリックされたアイテム名
        		if(item.equals("接続先の追加")) {
        			onClkItemAddConnect();
        		} else if(item.equals("接続の終了")) {
        			onClkItemDisconnect();
        		} else {
        			onClkItemConnect(parent, item, (int)id);
        		}
        	}
        });
	}
	
	/***
	 * 「接続の追加」がクリックされたとき
	 */
	private void onClkItemAddConnect() {
		Log.d("MethodCall", "ActivityPortforward.onClkItemAddCnct();");
		
		//ToDo ダイアログで登録できない旨を表示するようにする
		if(pfDataList.size() >= MAX_SIZE) return;
		
		Intent intent = new Intent(this, ActivityPortforwardSetting.class);
		startActivity(intent);
	}
	
	/**
	 * 「接続の終了」がクリックされたとき
	 */
	private void onClkItemDisconnect() {
		Log.d("MethodCall", "ActivityPortforward.onClkItemDisCnct();");
		PortforwardData.SetConnectedSetting("");
		if(ToServer.getInstance().disConnect()) {
			if(SshTunnel.getInstance().disConnect()) {				
				//メニューアイテムの設定
				String[] menu = new String[pfDataList.size()+2];
				menu[0] = "接続先の追加";
				//すでに設定があればそれをメニューに追加
				if(pfDataList.size() > 0) {
					for(int i=0; i<pfDataList.size(); i++) {
						menu[i+1] = pfDataList.get(i).getName();
					}
				}
				menu[menu.length-1] = "接続の終了";
				//アダプタを設定
				ListView lv = (ListView)findViewById(R.id.lv_menu);
				ArrayAdapter<String> adpt = new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1, menu);
				lv.setAdapter(adpt);	
			}
		} else {
			Log.i("log","ActivityPortfroward.onClkItemDisCnct()--接続終了に失敗");
		}
	}

	/**
	 * 登録された接続名がクリックされたとき
	 * @param id	クリックされたアイテムの番号
	 */
	private void onClkItemConnect(AdapterView<?> parent, String item, int id) {
		Log.d("MethodCall", "ActivityPortforward.onClkItemCnct(int id);");
		PortforwardData pfData = pfDataList.get(id-1);
		Intent intent = new Intent(this,ActivityPortforwardElement.class);
		intent.putExtra("pfData", pfData);
		startActivity(intent);						
	}
}
