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

import com.example.candatasender.main.PrivateAppData;
import com.example.candatasender.R;
import com.example.useful.ShortAlertDialog;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.text.Editable;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView.BufferType;

public class ActivityPortforwardSetting extends Activity {

	@Override
	protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        initActivity();
	}
	
	/**
	 * アクティビティの初期設定
	 */
	private void initActivity() {
		Log.d("MethodCall", "ActivityPortforwardSetting.initActivity();");
		
		//レイアウトの設定
		setContentView(R.layout.activity_prtfwd_setting);

		//デフォルト設定
		Intent g_intent = getIntent();		
		PortforwardData pfData = (PortforwardData)g_intent.getSerializableExtra("pfData");
		if(pfData == null) {		
			setEditableSshPort("22");	
			setEditableLocalPort("5558");
			setEditableFwdHost("127.0.0.1");
			setEditableFwdPort("5555");
		} else {
			setEditableCnctName(pfData.getName());
			setEditableSshHost(pfData.getSshHost());
			setEditableSshPort(pfData.getSshPort());
			setEditableSshUser(pfData.getSshUser());
			setEditableSshPass(pfData.getSshPass());
			setEditableLocalPort(pfData.getLocalPort());
			setEditableFwdHost(pfData.getFwdHost());
			setEditableFwdPort(pfData.getFwdPort());						
		}
		//「追加」ボタンのリスナーの設定
		Button btn = (Button)findViewById(R.id.prtfwd_setting_btn_add);
		btn.setOnClickListener(new OnClickListener() {
			@Override
			public void onClick(View view){
				onClkBtnAdd();
			}
		});
		
		//「キャンセル」ボタンのリスナーの設定
		btn = (Button)findViewById(R.id.prtfwd_setting_btn_cancel);
		btn.setOnClickListener(new OnClickListener() {
			@Override
			public void onClick(View view) {
				onClkBtnCancel();
			}
		});
	}
	
	/**
	 * 「追加」ボタンがクリックされたとき
	 */
	private void onClkBtnAdd() {
		Log.d("MethodCall", "ActivityPortforwardSetting.onClkBtnAdd();");

		//値の取得
		String name = getCnctName();
		String sshport = getSshPort();
		String sshhost = getSshHost();
		String sshuser = getSshUser();
		String sshpass = getSshPass();
		String localport = getLocalPort();
		String fwdhost = getFwdHost();
		String fwdport = getFwdPort();
		
		if(sshport.equals("")
				|| sshhost.equals("")
				|| sshuser.equals("")
				|| sshpass.equals("")
				|| localport.equals("")
				|| fwdhost.equals("")
				|| fwdport.equals("")) {
			ShortAlertDialog.sad("Notice", "入力データが不足しています", this);
		} else {
			//入力がOKだったら登録して、アクティビティを終了する
			PortforwardData pfData = new PortforwardData(name, sshport, sshhost, sshuser, sshpass, localport, fwdhost, fwdport);
			PrivateAppData.storePortforwardData(this, pfData);
			finish();
		}
	}
	
	/**
	 * 「キャンセル」ボタンがクリックされたとき
	 */
	private void onClkBtnCancel() {
		Log.d("MethodCall", "ActivityPortforwardSetting.onClkBtnCancel();");

		//アクティビティを終了する
		finish();
	}
	
	//TextEditの値を取得する
	private final String getCnctName()	 {	return getEditTextData(R.id.prtfwd_setting_et_cnctName);	}
	private final String getSshPort()	 {	return getEditTextData(R.id.prtfwd_setting_et_sshport);		}
	private final String getSshHost()	 {	return getEditTextData(R.id.prtfwd_setting_et_sshhost); 	}
	private final String getSshUser()	 {	return getEditTextData(R.id.prtfwd_setting_et_sshuser); 	}
	private final String getSshPass()	 {	return getEditTextData(R.id.prtfwd_setting_et_sshpass); 	}
	private final String getLocalPort()	 {	return getEditTextData(R.id.prtfwd_setting_et_localport); 	}
	private final String getFwdHost()	 {	return getEditTextData(R.id.prtfwd_setting_et_fwdhost); 	}
	private final String getFwdPort()	 {	return getEditTextData(R.id.prtfwd_setting_et_fwdport); 	}
	
	/**
	 * 引数で与えられたIDのEditTextから現在の文字列を取得する
	 * @param 	editTxtID	EditTextのID
	 * @return	指定されたEditTextの現在の文字列
	 */
	private final String getEditTextData(int editTxtID) {
	    EditText et = (EditText)findViewById(editTxtID);
	    Editable editable = et.getText();
	    return editable.toString();
	}

	//TextEditに編集可能な文字列を設定する
	private final void setEditableCnctName(String str)	{	setEditTextEditable(R.id.prtfwd_setting_et_cnctName, str);	}
	private final void setEditableSshPort(String str)	{	setEditTextEditable(R.id.prtfwd_setting_et_sshport, str);	}
	private final void setEditableSshHost(String str)	{	setEditTextEditable(R.id.prtfwd_setting_et_sshhost, str);	}
	private final void setEditableSshUser(String str)	{	setEditTextEditable(R.id.prtfwd_setting_et_sshuser, str);	}
	private final void setEditableSshPass(String str)	{	setEditTextEditable(R.id.prtfwd_setting_et_sshpass, str);	}
	private final void setEditableLocalPort(String str)	{	setEditTextEditable(R.id.prtfwd_setting_et_localport, str);	}
	private final void setEditableFwdHost(String str)	{	setEditTextEditable(R.id.prtfwd_setting_et_fwdhost, str);	}
	private final void setEditableFwdPort(String str)	{	setEditTextEditable(R.id.prtfwd_setting_et_fwdport, str);	}
	
	/**
	 * 引数で与えられたIDのEditTextに文字列を設定する
	 * 		設定した文字列は編集可能（Editable）
	 * @param editTxtID	EditTextのID
	 * @param str		設定する文字列
	 */
	private final void setEditTextEditable(int editTxtID, String str) {
		EditText et = (EditText)findViewById(editTxtID);
		et.setText(str, BufferType.EDITABLE);
	}
}
