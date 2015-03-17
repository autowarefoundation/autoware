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

package com.example.candatasender.toserver;


import java.io.IOException;
import java.io.ObjectOutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.io.InputStream;
import java.util.*;
import java.io.*;

import android.util.Log;

import com.example.candatasender.portforward.SshTunnel;
import com.example.candatasender.sender.*;


public class ToServer {
	private static ToServer ts = new ToServer();
	//reciveContinuouslySendDataOnThreadで使用する手動ロック用の変数。
	private volatile static boolean rcsd_lock = false;		
	private Socket sock = null;
	private String host = "";
    private int port = 0;
    //sendDataOnThreadのタイムアウト設定
    private final int SDOT_TIME = 5000;
    //ストリーム
    private OutputStream os = null;
    private InputStream is = null;
    //送信データ、受信データを保管
    private volatile Map<String,Object> m_rd = null;
    private Map<String,Object> m_sd = null;
    //ロック中(通信スレッドが稼動中）ならtrue
    public boolean GetRcsdLock() {
    	return rcsd_lock;
    }    
    
    public static ToServer getInstance() {
    	return ts;
    }
    
	/**
	 * ホストとポートを設定
	 * @param h	ホスト名
	 * @param p	ポート番号
	 */
	public void setServerData(String h, int p) {
		Log.d("MethodCall", "ToServer.setServerData(String h, int p);");
		host = h;
		port = p;
	}
	
	/**
	 * サーバーと接続されているかの確認
	 * @return	true	接続されている
	 * 			false 	接続されていない
	 */
	public boolean isConnected() {
		Log.d("MethodCall", "ToServer.isConnected();");
		return (sock != null) ? true:false;
	}
	
	/**
	 * サーバーに接続する
	 * @return	true	接続完了
	 * 			false	接続失敗
	 */
	public Map<String,String> connect() {
		Log.d("MethodCall", "ToServer.Connect();");
		Map<String,String> tem_rd = new HashMap<String,String>();
		//ホストとポートが設定されていなかったら終了
		if(host == "" || port == 0) {
			Log.e("error", "ホストとポートを設定してください。");			
			tem_rd.put("TYPE", DataType.DT_ERROR.getType());
			tem_rd.put("MESSAGE", "ホストとポートが設定されていません");
			return tem_rd;
		}
		
		if(isConnected()) {			
			tem_rd.put("TYPE", DataType.DT_ERROR.getType());
			tem_rd.put("MESSAGE", "既にサーバとの接続は確立しているようです。\n" +
								  "再接続する場合は一度接続の終了をしてください");
			return tem_rd;
		}
		//Threadの生成
		Thread th;
    	th = new Thread(new Runnable() {
    			@Override
    			public void run() {
    				//sshトンネル経由でpoststoreに接続する
    				try {
    					sock = new Socket();
    					sock.connect(new InetSocketAddress(host, port));    					
    					Log.i("log", "ToServer.connect - サーバに接続");
    				} catch (IOException e) {
    					sock = null;
    					Log.e("error", "ToServer.connect - 接続失敗");
    					e.printStackTrace();
    				}
    			}
    		});
    	//実行スタート
    	th.start();    	
    	//実行終了を待つ
    	try {
    		th.join(5000);        	
    	} catch(InterruptedException e) {
    		e.printStackTrace();
    	}
    	Log.i("log","接続スレッド終了");    		    		    
    	if(sock == null) {    		    		
			Log.i("log","Tosever.Connect() - sock is null");
			SshTunnel.getInstance().disConnect();			
			tem_rd.put("TYPE", DataType.DT_ERROR.getType());
    		tem_rd.put("MESSAGE", "サーバ接続に失敗");
    		return tem_rd;
    	}
    	if(isConnected()) {    		
    		//ストリームを開く
    		try {
				is = sock.getInputStream();
				os = sock.getOutputStream();
			} catch (IOException e1) {
				Log.i("error","ToServer.connect() Fail to open socket stream.");
				e1.printStackTrace();
			}
    		//poststoreとコネクションを確立できているか確認する。
    		Map<String,Object> tem_sd = new HashMap<String,Object>();
			tem_sd.put("TYPE", DataType.DT_INST.getType());
			Map<String,String> data = new HashMap<String,String>();		
			data.put("INST_NUM",ClientInstruction.INST_CHECK_CONNECTION.getNum());
			tem_sd.put("DATA", data);
			Map<String,Object> response = new HashMap<String,Object>();			
			//送信データキューに入れる
			DataCommunications.getInstance().addSendData(tem_sd,response);			
			if(((String)response.get("TYPE")).equals(DataType.DT_CHECK_CONNECTION.getType())) {
				Log.i("log","Toserver.connect() - サーバ接続成功");
				tem_rd.put("TYPE", DataType.DT_CONNECTION_SUCCESS.getType());			
			} else {						
				Log.i("log","Toserver.connect() - サーバ接続失敗");				
				tem_rd.put("TYPE", DataType.DT_ERROR.getType());
				tem_rd.put("MESSAGE", "サーバ接続に失敗");									  
				try {
					sock.close();					
				} catch(IOException e) {
					e.printStackTrace();					
				}
				sock = null;
			}				
    	} else {    		
    		Log.i("log","Toserver.connect() - サーバ接続失敗");				
    		tem_rd.put("TYPE", DataType.DT_ERROR.getType());
    		tem_rd.put("MESSAGE", "サーバ接続に失敗");
			try {
				sock.close();
			} catch(IOException e) {
				e.printStackTrace();					
			}			    
			sock = null;
    	}	
    	Log.i("log","rd = " + tem_rd);    	
    	return tem_rd;
	}
	/**
	 * サーバーとの接続を終了する
	 */
	public boolean disConnect() {
		Log.d("MethodCall", "ToServer.disConnect();");
		if(!isConnected()) return true;		
		//sendDataOnThreadで接続終了要求を送る。
		Map<String,Object> send = new HashMap<String,Object>();
		Map<String,Object> response = new HashMap<String,Object>();
		send.put("TYPE", DataType.DT_INST.getType());
		Map<String,String> data = new HashMap<String,String>();
		data.put("INST_NUM", ClientInstruction.INST_QUIT.getNum());	
		send.put("DATA",data);		
		DataCommunications.getInstance().addSendData(send, response);
		if((response.get("TYPE")).equals(DataType.DT_DIS_CONNECTION.getType())) {			
			//接続終了成功
			try {
				host = "";
				port = 0;
				sock.close();
				sock = null;
				Log.i("log","接続終了成功");
			} catch(IOException e) {
				Log.e("error","Socketのクローズに失敗");
				e.printStackTrace();
			}					
		} else {
			//poststoreに接続終了要求の送信、もしくはレスポンスの受信に失敗した場合、
			//errorの場合、ssh接続はされていたが、poststoreとは接続していなかったとみなす。			
			host = "";
			port = 0;			
			try {
				sock.close();				
			} catch(Exception e) {
				Log.e("error","Socketのクローズに失敗");
				e.printStackTrace();
			}			
			Log.i("log","接続終了成功。ただし元々サーバとは接続出来ていなかったと思われます");
			sock = null;
		}
		Log.i("log", "サーバーとの接続を終了");
		return (sock == null) ? true:false;
	}
	
	//非同期処理
	//通信スレッドは同時には一つしか立てない。
	//取得したデータ,エラーメッセージはrdに入れる。	
	//DataCommunicationsを通して実行する
	synchronized public void sendDataOnThread(final Map<String, Map<String, Object>> srd) {					
		Log.d("MethodCall", "ToServer.sendData;");
		
		if(!isConnected() || sock.isClosed()) {
			Log.i("log","サーバ未接続");
			InputDisConnectionMessage(m_rd);
			return;
		}
		//既に通信スレッドが動いている場合はreturnする
		if(rcsd_lock == true) {
			Log.i("log","通信スレッドが使用中");
			InputDataCommunicationsThreadActiveMessage(m_rd);
			return;
		}
		
		m_sd = (Map<String,Object>)srd.get(DataCommunications.getInstance().getTagSd());
		m_rd = (Map<String,Object>)srd.get(DataCommunications.getInstance().getTagRd());
		
		if(m_sd == null || m_rd == null) {
			Log.i("log","送信オブジェクト、受信オブジェクトがセットされていない");
			InputNoSendDataMessage(m_rd);
			return;
		}
		//ロック
		rcsd_lock = true;
		
		//Threadの生成
		Thread th;
    	th = new Thread(new Runnable() {
    			@SuppressWarnings("unchecked")
				@Override
    			public void run() {
    				ByteArrayOutputStream baos = null;
    				ByteArrayInputStream bais = null;
    	            ObjectOutputStream oos = null;
    	            ObjectInputStream ois = null;    	            
    	            byte[] ReciveByte = null;        	            
    				try {
    					baos = new ByteArrayOutputStream();
    		            oos = new ObjectOutputStream(baos);    		            
    		            oos.writeObject(m_sd);
    		            byte[] send_byte = baos.toByteArray();
    		            //Log.i("log","Send to poststore :" + m_sd);
    		            os.write(send_byte);	
    		            os.flush();
    		            long start = System.currentTimeMillis();
    		            while(true) {
    		            	if(System.currentTimeMillis() - start > SDOT_TIME) {
    		            		InputResponseTimeoutMessage(m_rd);
    		            		break;
    		            	}
    		            	if(is.available() == 0) {
    		            		continue;
    		            	}
    		            	ReciveByte = new byte[is.available()];    		            	
    		            	is.read(ReciveByte, 0, is.available());
    		            	bais = new ByteArrayInputStream(ReciveByte);
    		            	ois = new ObjectInputStream(bais); 				
    		            	Map<String,Object> work = (Map<String,Object>)ois.readObject();
    		            	for(Map.Entry<String,Object> e : work.entrySet()) {
    		            		m_rd.put(e.getKey(), e.getValue());
    		            	}
    		            	Log.i("log","SendDataOnThread : recive_data : " + m_rd);    	                        
    		            	Log.i("log","end SendDataThread");    		            	
    		            	break;
    		            }	    				    		            
		           } catch(IOException e) {
		        	   InputExceptionErrorMessage(m_rd);
		        	   e.printStackTrace();
    		       } catch (ClassNotFoundException e) {
    		    	   e.printStackTrace();
    		       } finally {    					
    		    	   try {
    		    		   Log.i("log","sendDataOnThread finally");
    		    		   //データ送信に失敗したとき、ゴミが残っているとclose()出来ないっぽい。
    		    		   oos.reset();    						
    		    		   oos.close();
    		    		   if(ois != null) {
    		    			   ois.close();
    		    		   }
    		    		   if(bais != null) {
    		    			   bais.close();     						
    		    		   }
    		    		   baos.close();
    		    		   //ロック解除
    		    		   rcsd_lock = false;
    		    	   } catch(IOException e) {
    						e.printStackTrace();
    		    	   }     					
    		       }
    			}
    	});
    	//実行スタート
    	th.start();    	    	
	}
	
	public void InputResponseTimeoutMessage(Map<String,Object> m) {
		m.put("TYPE", DataType.DT_RESPONSE_TIMEOUT.getType());
		m.put("MESSAGE", "サーバからのレスポンスの受信に失敗しました");		
	}
	
	public void InputDisConnectionMessage(Map<String,Object> m) {
		m.put("TYPE", DataType.DT_DIS_CONNECTION.getType());
		m.put("MESSAGE", "サーバに接続されていません");		
	}
	
	public void InputNoSendDataMessage(Map<String,Object> m) {
		m.put("TYPE", DataType.DT_NO_SEND_DATA.getType());
		m.put("MESSAGE", "送信データがセットされていません");		
	}
	
	public void InputDataCommunicationsThreadActiveMessage(Map<String,Object> m) {
		m.put("TYPE", DataType.DT_DATA_COMMUNICATIONS_THREAD_ACTIVE.getType());
		m.put("MESSAGE", "通信スレッドが既に使用されています");		
	}
	
	public void InputExceptionErrorMessage(Map<String,Object> m) {
		m.put("TYPE", DataType.DT_DATA_COMMUNICATIONS_THREAD_ACTIVE.getType());
		m.put("MESSAGE", "例外エラーが発生しました");
	}
}
