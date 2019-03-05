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

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.example.candatasender.sender.ClientInstruction;
import com.example.candatasender.sender.DataType;
import com.example.candatasender.toserver.DataCommunications;
import com.example.candatasender.R;
import com.example.useful.ShortAlertDialog;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;

public class ActivityTableList extends Activity {
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		Log.i("log","ActivityTableList");
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_listview);
	}

	@Override
	protected void onResume() {
		super.onResume();
		initMenu();
	}

	private void initMenu() {
		Log.d("MethodCall", "ActivityTableList.initMenu();");
		//poststoreからテーブル名を取得
		Map<String,Object> send = new HashMap<String,Object>();
		send.put("TYPE",DataType.DT_INST.getType());
		Map<String,String> data = new HashMap<String,String>();
		data.put("INST_NUM", ClientInstruction.INST_GET_TABLE_NAME.getNum());
		send.put("DATA", data);
		Map<String,Object> response = new HashMap<String,Object>();
		DataCommunications.getInstance().addSendData(send, response);
		if(((String)response.get("TYPE")).equals(DataType.DT_TABLE.getType())) {
			//メニューアイテムの設定
			@SuppressWarnings("unchecked")
			List<String> tb = (List<String>)response.get("DATA");
			String[] menu = new String[tb.size()];
			for(int i=0;i < tb.size();i++) {
				menu[i] = (String)tb.get(i);
			}
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

					Intent intent = new Intent();
					intent.putExtra("table", item);
					setResult(Activity.RESULT_OK, intent);
					finish();
				}
			});
		} else {
			Log.e("error","ActivityTableList initMenu - mistake get table name list - " + ((String)response.get("TYPE")));
			String title ="Error message";
			String message = "サーバからのテーブル名の一覧取得に失敗しました\n";
			message += (String)response.get("MESSAGE");
			ShortAlertDialog.sad(title, message, this);
		}
		Log.d("location","initMenu finish");
	}
}
