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

package com.example.candatasender.sender;


/**
 * クライアントがサーバーに送信するデータの先頭に着ける識別子
 * サーバー側で処理を振り分ける際の識別子となる
 * 1~4はMapPlotは使用しないが、DataSenderと同じくpoststoreに命令を送信してデータを受信するため、残してある。
 */

public enum ClientInstruction {
	/*データ送信形式（poststoreは受信した命令のcsvで区切られた先頭の値で処理を分岐する）
	 * ClientInstruction.INST_INSERT.getNum(),***;*/
	INST_QUIT	("0"),		//接続終了要求
	INST_INSERT	("1"),		//DBへのインサート
	//INST_CREATE	("2"),		//テーブルの作成
	//INST_DROP	("3"),		//テーブルの削除
	//INST_STOP   ("4"),
	INST_COUNT	("5"),	//命令数
	INST_SELECT_LOCATE("6"),  //位置情報を要求
	INST_GET_TABLE_NAME("7"), //テーブル名の一覧を要求
	INST_UPDATE_OR_INSERT ("8"),//各端末の最新の位置情報を送信する。古いデータは上書きする。
	INST_CHECK_CONNECTION ("9"),
	INST_INSERT_CL_DATA ("10"),		// CarLinkデータ送信
	INST_INSERT_CG_DATA ("11");		// CanGatherデータ送信

	
	private String num;
	
	private ClientInstruction(String n)
	{
		num = n;
	}
	
	public String getNum()
	{
		return num;
	}
	
}
