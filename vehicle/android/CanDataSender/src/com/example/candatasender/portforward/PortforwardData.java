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

package com.example.candatasender.portforward;

import java.io.Serializable;

public class PortforwardData implements Serializable {
	private static final long serialVersionUID = 5734877824824L;
	
	private String[] data = new String[PortforwardDataID.ID_COUNT.toInt()];
		
	//接続中の設定を保存する。
	private static String connected_setting = "";	
	
	public static String GetConnectedSetting() {
		return connected_setting;
	}
	public static void SetConnectedSetting(String scs) {
		connected_setting = scs;
	}
	public PortforwardData() {
		initialize();
	}
	
	/**
	 * コンストラクタ
	 * @param str	接続名
	 * @param sp	sshポート
	 * @param sh	sshホスト名
	 * @param su	sshユーザ名
	 * @param spss	sshパスワード
	 * @param lp	ローカルポート
	 * @param ch	接続ホスト名
	 * @param fp	接続ポート
	 */
	public PortforwardData(String str, String sp, String sh, String su, String spss, String lp, String fh, String fp) {
		//接続名がない場合、「ユーザ名@sshホスト名」にする
		if(str.equals("")) {
			str = su + "@" + sh;
		}
		setName(str);
		setSshPort(sp);
		setSshHost(sh);
		setSshUser(su);
		setSshPass(spss);
		setLocalPort(lp);
		setFwdHost(fh);
		setFwdPort(fp);
	}
	

	private void initialize() {
		setName("");
		setSshPort("");
		setSshHost("");
		setSshUser("");
		setSshPass("");
		setLocalPort("");
		setFwdHost("");
		setFwdPort("");
	}
	
	/**
	 * メンバを一行の文字列に変換する（CSV形式）
	 * @return	変換した文字列
	 */
	public final String getString() {
		StringBuffer str = new StringBuffer(getName());
		for(int i=1; i<PortforwardDataID.ID_COUNT.toInt(); i++) {
			str.append("," + data[i]);
		}
		return str.toString();
	}
	
	/**
	 * getString()で生成される形式の文字列から復元する
	 * @param str	getString()で得られるものと同じ形式のStringデータ
	 */
	public void parseString(String str) {
		String[] dt = str.split(",");
		for(int i=0; i<PortforwardDataID.ID_COUNT.toInt(); i++) {
			data[i] = dt[i];
		}
	}
	
	//getter
	public final String getName() 		{ return data[PortforwardDataID.ID_NAME.toInt()]; }
	public final String getSshPort() 	{ return data[PortforwardDataID.ID_SSHPORT.toInt()]; }
	public final String getSshHost() 	{ return data[PortforwardDataID.ID_SSHHOST.toInt()]; }
	public final String getSshUser() 	{ return data[PortforwardDataID.ID_SSHUSER.toInt()]; }
	public final String getSshPass() 	{ return data[PortforwardDataID.ID_SSHPASS.toInt()]; }
	public final String getLocalPort()	{ return data[PortforwardDataID.ID_LOCALPORT.toInt()]; }
	public final String getFwdHost() 	{ return data[PortforwardDataID.ID_FWDHOST.toInt()]; }
	public final String getFwdPort() 	{ return data[PortforwardDataID.ID_FWDPORT.toInt()]; }
	
	//setter
	public final void setName(final String str) 		{ data[PortforwardDataID.ID_NAME.toInt()] 		= str;  }
	public final void setSshPort(final String port)		{ data[PortforwardDataID.ID_SSHPORT.toInt()] 	= port; }
	public final void setSshHost(final String host) 	{ data[PortforwardDataID.ID_SSHHOST.toInt()] 	= host; }
	public final void setSshUser(final String user) 	{ data[PortforwardDataID.ID_SSHUSER.toInt()] 	= user; }
	public final void setSshPass(final String pass) 	{ data[PortforwardDataID.ID_SSHPASS.toInt()] 	= pass; }
	public final void setLocalPort(final String port) 	{ data[PortforwardDataID.ID_LOCALPORT.toInt()] 	= port; }
	public final void setFwdHost(final String host) 	{ data[PortforwardDataID.ID_FWDHOST.toInt()] 	= host; }
	public final void setFwdPort(final String port) 	{ data[PortforwardDataID.ID_FWDPORT.toInt()] 	= port; }
}
