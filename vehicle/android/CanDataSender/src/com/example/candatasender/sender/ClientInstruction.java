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
