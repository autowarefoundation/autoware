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

package com.ghostagent;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;

import com.ghostagent.R;

import android.os.Bundle;
import android.os.Environment;
import android.app.Activity;
import android.content.Intent;
import android.util.Log;
import android.view.Menu;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

public class StartActivity extends Activity implements OnClickListener {
	/**
	 * Start button
	 */
	Button startButton;
	/**
	 *  address box
	 */
	EditText addressBox;
	/**
	 * port number box
	 */
	EditText portNumberBox;
	/**
	 *  address string
	 */
	String address;
	/**
	 * port string
	 */
	String port;
	/**
	 * connect info file
	 */
	String path = "/autowarerider.txt";
	/**
	 * input stream
	 */
	BufferedInputStream istream;
	/**
	 * text for read file
	 */
	String text;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_start);

		try {
			istream = new BufferedInputStream(new FileInputStream(
					Environment.getExternalStorageDirectory().getPath() + path));
			byte[] buffer = new byte[256]; 
			istream.read(buffer);
			text = new String(buffer).trim();
			istream.close();
		} catch (FileNotFoundException e) {
			// don't care
		} catch (Exception e) {
			e.printStackTrace();
		}

		if(text != null) {
			String[] strAry = text.split(",");
			Log.v("log", "" + strAry.length);

			if(!strAry[0].equals("null") && !strAry[1].equals("null")){
				// pass intent to GhostEyeActivity
				Intent intent = new Intent(getApplication(), SoundManagementActivity.class);

				// address and port
				intent.putExtra("address", strAry[0]);
				intent.putExtra("port", Integer.parseInt(strAry[1]));

				// start
				startActivity(intent);
			}
		}

		// start button
		startButton = (Button)findViewById(R.id.startButton);
		startButton.setOnClickListener(this);
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.start, menu);
		return true;
	}

	private boolean validatePortNumer(final String portString) {
		int port;
		boolean isValid;

		try {
			port = Integer.parseInt(portString);
			if (port >= 0 && port <= 65535)
				isValid = true;
			else
				isValid = false;
		} catch (NumberFormatException e) {
			isValid = false;
		}

		return isValid;
	}

	public void onClick(View v) {
		// push button
		if(v == startButton){
			// address box
			addressBox = (EditText) findViewById(R.id.addressBox);
			address = addressBox.getText().toString();
			if (address == null || address.length() == 0) {
				Toast.makeText(this, "Please input address", Toast.LENGTH_LONG).show();
				return;
			}

			// port number box
			portNumberBox = (EditText) findViewById(R.id.portNumberBox);
			port = portNumberBox.getText().toString();
			if (port == null || port.length() == 0) {
				Toast.makeText(this, "Please input port", Toast.LENGTH_LONG).show();
				return;
			}

			if (!validatePortNumer(port)) {
				Toast.makeText(this, "Please input port less than equal 65535", Toast.LENGTH_LONG).show();
				return;
			}

			// pass intent to GhostEyeActivity
			Intent intent = new Intent(getApplication(), SoundManagementActivity.class);

			// address and port
			intent.putExtra("address", address);
			intent.putExtra("port", port);

			// start
			startActivity(intent);
		}
	}
}
