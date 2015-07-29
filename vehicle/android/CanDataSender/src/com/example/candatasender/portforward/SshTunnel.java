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
