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

//ToSever.rd,ToServer.sdに使う
public enum DataType {	
	DT_ERROR    ("ERROR"),
	//INST_UPDATE_OR_INSERT　からの返事。
	DT_IUOI_MESSAGE ("IUOI_M"),
	//INST_INSERT　時の返事
	DT_INSERT_MESSAGE ("INSERT_MESSAGE"),
	//
	DT_INST      ("INSTRUCTION"),
	DT_CHECK_CONNECTION ("CHECK_CONNECTION"),
	//位置情報
	DT_LOCATION ("LOCATION"),	
	//接続されてないとき
	DT_DIS_CONNECTION ("DIS_CONNECTION"),
	//poststoreからレスポンスが来ないとき
	DT_RESPONSE_TIMEOUT ("RESPONSE_TIMEOUT"),
	//送信データがセットされていない（null)のとき
	DT_NO_SEND_DATA ("NO_SEND_DATA"),
	//通信スレッドが稼働中
	DT_DATA_COMMUNICATIONS_THREAD_ACTIVE ("DATA_COMMUNICATIONS_THREAD_ACTIVE"),
	//接続成功
	DT_CONNECTION_SUCCESS("CONNECTION_SUCCESS"),
	DT_EXCEPTION_ERROR("EXCEPTION_ERROR"),
	DT_TABLE   ("TABLE_NAME");			
	private String type;	
	private DataType(String n)
	{
		type = n;
	}
	
	public String getType()
	{
		return type;
	}
}
