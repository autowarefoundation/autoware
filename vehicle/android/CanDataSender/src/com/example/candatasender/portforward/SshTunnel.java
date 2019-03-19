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

import java.util.Properties;

import android.util.Log;

import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;

public class SshTunnel {
    //シングルトン
    private static SshTunnel ins = new SshTunnel();
    private static final JSch jsch = new JSch();
    private static Session session = null;
	
    private static PortforwardData pfData = new PortforwardData();
	
    private SshTunnel() {}
    
    public Session GetSession() {
    	return session;
    }
    
    /**
     * トンネルを掘る
     * @return	true	ポートフォワード成功
     * 			false	失敗
     */
    public boolean connect(PortforwardData pf) {
		Log.d("MethodCall", "SshTunnel.conect(PortforwardData pf);");

    	pfData = pf;
    	if(session != null) return true;
    	
    	Thread th;
    	th = new Thread(new Runnable() {
    		@Override
    		public void run() {
    			try {
    	    		//セッションの作成    	    		
    	    		session = jsch.getSession( pfData.getSshUser(), pfData.getSshHost(), Integer.parseInt(pfData.getSshPort()) );
    				session.setPassword( pfData.getSshPass() );
    	    		Log.i("log", "セッション設定");
    	    		
    			    //鍵は使わない
    			    final Properties config = new Properties();
    			    config.put( "StrictHostKeyChecking", "no" );
    			    session.setConfig( config );
    			    Log.i("log", "鍵の設定");

    			    //keep alive
    			    session.setServerAliveInterval(5000);
    			    //接続
    			    session.connect(5000);
    			    Log.i("log", "ssh接続しました。");
    			    if(session != null) {
    			    	//ポートフォワード
    			    	session.setPortForwardingL(Integer.parseInt(pfData.getLocalPort()), pfData.getFwdHost(), Integer.parseInt(pfData.getFwdPort()));
    			    	Log.i("log", "ポートフォワーディング完了。");
    			    }
    	    	} catch(JSchException e) {
    	       		e.printStackTrace();
    	    		session = null;
    	    	}
    		}
    	});
    	th.start();
    	
    	try {
    		th.join();
    	} catch(InterruptedException e) {
    		e.printStackTrace();
    	}

    	return (session != null) ? true:false;
    }
    
    /**
     * 掘ったトンネルを閉じる
     * @return	true	成功
     * 			false	失敗
     */
    public boolean disConnect() {
		Log.d("MethodCall", "SshTunnel.disConnect();");
    	
    	if(session == null) {
        	Log.i("log", "セッションが存在しません。");
    		return false;
    	}
    	session.disconnect();
    	session = null;

    	Log.i("log", "セッションを終了しました。");

    	return true;
    }
    
    public static SshTunnel getInstance() {
    	return ins;
    }
}
