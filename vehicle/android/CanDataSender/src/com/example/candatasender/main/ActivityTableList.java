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
