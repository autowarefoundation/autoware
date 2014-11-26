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
	String path = "/autowaretouch.txt";
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