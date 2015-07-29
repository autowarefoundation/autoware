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
