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

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

import com.example.candatasender.portforward.PortforwardData;

import android.app.Activity;
import android.content.Context;
import android.util.Log;


//内部保存領域とデータをやりとりするメソッド群

public class PrivateAppData {
	private static final String FILE_PORTFORWARD 	= "file_portforward"; 	//フォワーディング設定を保存するファイル
	/**
	 * ポートフォワード設定を内部ファイルに出力する
	 * @param act		呼び出し元のアクティビティ
	 * @param pfData	出力するデータ
	 */
	public static void storePortforwardData(Activity act, PortforwardData pfData) {
		Log.d("MethodCall", "PrivateAppData.storePortforwardData(Activity act, PortforwardData pfData);");
		
		try {
			ArrayList<String> strs = new ArrayList<String>();
			//読み込み
			try {
				BufferedReader br = new BufferedReader(
										new InputStreamReader(
											act.openFileInput(FILE_PORTFORWARD)));			
				//1行ごとに読み込み
				String str;
				while((str=br.readLine()) != null) {
					System.out.println(str);
					strs.add(str);
				}
				br.close();					
				Log.i("log", "データ読み込み完了");
			} catch(FileNotFoundException e) {
				Log.e("error", "ファイルが見つかりません： " + FILE_PORTFORWARD);
			} catch(IOException e) {
				Log.e("error", "読み込みエラー");
			}
			//重複設定を調べる。
			for(int i=0; i<strs.size(); i++) {
				if(strs.get(i).equals(pfData.getString())) {
					return;
				}
			}
			
			BufferedWriter bw = new BufferedWriter(
					new OutputStreamWriter(
						act.openFileOutput(FILE_PORTFORWARD, Context.MODE_PRIVATE | Context.MODE_APPEND)));			
			bw.write(pfData.getString());
			System.out.println(pfData.getString());
			bw.newLine();
			bw.flush();
			bw.close();
		} catch(FileNotFoundException e) {
			e.printStackTrace();
			Log.e("error", "ファイルが存在しません： " + FILE_PORTFORWARD);
		} catch(IOException e){
			e.printStackTrace();
			Log.e("error", "IOエラー");
		}
	}
	
	/**
	 * ポートフォワード設定を内部ファイルから読み込む
	 * @param 		act						呼び出し元のアクティビティ
	 * @param		pfData					読み込んだデータを入れる変数
	 * @return		データが存在した場合			読み込んだ設定の数
	 * 				データが存在しなかった場合		0
	 */
	public static int loadPortforwardData(Activity act, ArrayList<PortforwardData> pfData) {
		Log.d("MethodCall", "PrivateAppData.loadPortforwardData(Activity act, ArrayList<PortforwardData> pfData);");

		try {
			BufferedReader br = new BufferedReader(
									new InputStreamReader(
										act.openFileInput(FILE_PORTFORWARD)));
			
			//1行ごとに読み込み
			String str;
			while((str=br.readLine()) != null){
				System.out.println(str);
				PortforwardData pfd = new PortforwardData();
				pfd.parseString(str);
				pfData.add(pfd);
			}
			br.close();
					
			Log.i("log", "データ読み込み完了");
			return pfData.size();
		} catch(FileNotFoundException e) {
			Log.e("error", "ファイルが見つかりません： " + FILE_PORTFORWARD);
			return 0;
		} catch(IOException e) {
			Log.e("error", "読み込みエラー");
			return 0;
		}
	}

	//ポートフォワーディングの設定を消す。
	public static boolean deletePortforwardData(Activity act, PortforwardData pfd)
	{
		Log.d("MethodCall", "PrivateAppData.deletePortForwardData(Activity act, String tableName);");
		
		
		ArrayList<String> strs = new ArrayList<String>();
		//読み込み
		try {
			BufferedReader br = new BufferedReader(
									new InputStreamReader(
										act.openFileInput(FILE_PORTFORWARD)));
			
			//1行ごとに読み込み
			String str;
			while((str=br.readLine()) != null){
				System.out.println(str);
				strs.add(str);
			}
			br.close();					
			Log.i("log", "データ読み込み完了");
		} catch(FileNotFoundException e) {
			Log.e("error", "ファイルが見つかりません： " + FILE_PORTFORWARD);
			return false;
		} catch(IOException e) {
			Log.e("error", "読み込みエラー");
			return false;
		}		
		//削除
		for(int i=0; i<strs.size(); i++){
			if(strs.get(i).equals(pfd.getString())){
				strs.set(i, "");
			}
		}
		
		//書き出し
		try {
			BufferedWriter bw = new BufferedWriter(
									new OutputStreamWriter(
										act.openFileOutput( FILE_PORTFORWARD, Context.MODE_PRIVATE)));
			for(int i=0; i<strs.size(); i++){
				if(strs.get(i).isEmpty()) continue;
				bw.write(strs.get(i));
				bw.newLine();
				Log.i("log", "STORE: " + strs.get(i));
			}
			bw.flush();
			bw.close();
		} catch(FileNotFoundException e) {
			e.printStackTrace();
			Log.e("error", "ファイルが存在しません： " + FILE_PORTFORWARD);
			return false;
		} catch(IOException e) {
			e.printStackTrace();
			Log.e("error", "IOエラー");
			return false;
		}

		return true;
	}
}
