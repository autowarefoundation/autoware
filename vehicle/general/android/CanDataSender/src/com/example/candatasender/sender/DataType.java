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
