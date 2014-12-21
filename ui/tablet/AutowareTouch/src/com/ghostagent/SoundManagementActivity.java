package com.ghostagent;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.Random;
import java.util.StringTokenizer;

import com.design.DrawCenterView;
import com.design.DrawLeftView;
import com.design.DrawRightView;
import com.ghostagent.R;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.ImageButton;
import android.widget.Toast;

public class SoundManagementActivity extends Activity implements OnClickListener {
	/**
	 * server address
	 */
	String address;
	/**
	 * server port
	 */
	int port;
	/**
	 * view width
	 */
	public static int viewWidth;
	/**
	 * view height
	 */
	public static int viewHeight;
	/**
	 * knight rider left view
	 */
	DrawLeftView drawLeftView;
	/**
	 * knight rider right view
	 */
	DrawRightView drawRightView;
	/**
	 * knight rider center view
	 */
	DrawCenterView drawCenterView;
	/**
	 * size flag
	 */
	public static boolean getSizeFlag = false;
	/**
	 * Image buttons
	 */
	ImageButton vision, voice, s1, s2, p1, p2, p3, p4, autoCruise, normalCruise, pursuit;
	/**
	 * flag for knight riding
	 */
	boolean bIsKnightRiding = false;

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.knight_rider);

		// get parameters
		address = getIntent().getExtras().getString("address");
		if (address == null || address.length() == 0) {
			Toast.makeText(this, "Bad address", Toast.LENGTH_LONG).show();
			Log.v("Log", "bad address");
			finish();
		}
		String portStr = getIntent().getExtras().getString("port");
		if (portStr == null || portStr.length() == 0) {
			Toast.makeText(this, "Bad port", Toast.LENGTH_LONG).show();
			Log.v("Log", "bad port");
			finish();
		}
		port = Integer.parseInt(portStr);
		Log.v("Log", "address: " + address + ", port: " + port);

		//connect server
		if (SoundManagementNative.connect(address, port) < 0) {
			Toast.makeText(this, "Cannot connect", Toast.LENGTH_LONG).show();
			Log.w("Log", "cannot connect");
			finish();
		}

		// center expression
		drawLeftView = (DrawLeftView) findViewById(R.id.leftView);
		drawRightView = (DrawRightView) findViewById(R.id.rightView);
		drawCenterView = (DrawCenterView) findViewById(R.id.centerView);

		// set buttons
		vision = (ImageButton)findViewById(R.id.air);
		findViewById(R.id.air).setOnClickListener(this);

		voice = (ImageButton)findViewById(R.id.oil);
		findViewById(R.id.oil).setOnClickListener(this);

		s1 = (ImageButton)findViewById(R.id.s1);
		findViewById(R.id.s1).setOnClickListener(this);

		s2 = (ImageButton)findViewById(R.id.s2);
		findViewById(R.id.s2).setOnClickListener(this);

		p1 = (ImageButton)findViewById(R.id.p1);
		findViewById(R.id.p1).setOnClickListener(this);

		p2 = (ImageButton)findViewById(R.id.p2);
		findViewById(R.id.p2).setOnClickListener(this);

		p3 = (ImageButton)findViewById(R.id.p3);
		findViewById(R.id.p3).setOnClickListener(this);

		p4 = (ImageButton)findViewById(R.id.p4);
		findViewById(R.id.p4).setOnClickListener(this);

		autoCruise = (ImageButton)findViewById(R.id.autoCruise);
		findViewById(R.id.autoCruise).setOnClickListener(this);

		normalCruise = (ImageButton)findViewById(R.id.normalCruise);
		findViewById(R.id.normalCruise).setOnClickListener(this);

		pursuit = (ImageButton)findViewById(R.id.pursuit);
		findViewById(R.id.pursuit).setOnClickListener(this);

		// start recording
		bIsKnightRiding = true;
		startKnightRiding();

	}

	public void startKnightRiding() {
		// Recoding Thread
		new Thread(new Runnable() {
			public void run() {
				while (bIsKnightRiding) {
					Random rnd = new Random();
					drawLeftView.drawView(rnd.nextInt(10));
					drawRightView.drawView(rnd.nextInt(10));
					drawCenterView.drawView(rnd.nextInt(10) + 4);
					try {
						Thread.sleep(100);
					} catch (Exception e) {
						e.printStackTrace();
						bIsKnightRiding = false;
					}
				}
			}
		}).start();
	}

	@Override
	protected void onDestroy() {
		super.onDestroy();
		bIsKnightRiding = false;
		SoundManagementNative.close();

	}

	@Override
	public void onRestart() {
		Toast.makeText(this, "onRestart", Toast.LENGTH_LONG).show();
		File file = new File(
				Environment.getExternalStorageDirectory().getPath() + "/MapRoute.txt");
		try {
			FileReader reader = new FileReader(file);
			BufferedReader breader = new BufferedReader(reader);
			String line;
			StringTokenizer token;
			for (int i = 1; (line = breader.readLine()) != null; i++) {
				token = new StringTokenizer(line, ",");
				if (token.countTokens() == 2) {
					Double lat = Double.parseDouble(token.nextToken());
					Double lon = Double.parseDouble(token.nextToken());
					Toast.makeText(this, "[" + i + "] " + lat + "," + lon, Toast.LENGTH_SHORT).show();
				}
			}
			breader.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
		super.onStart();
	}

	@Override
	public void onWindowFocusChanged(boolean hasFocus) {
		if (hasFocus) {
			viewWidth = drawLeftView.getWidth();
			viewHeight = drawLeftView.getHeight();
			getSizeFlag = true;
			Log.v("View", "width: " + viewWidth + ", height: " + viewHeight);
		}
		super.onWindowFocusChanged(hasFocus);
	}

	@Override
	public void onClick(View v) {
		if(v == vision){
			Log.v("Button", "vision");
		}
		else if(v == voice){
			Log.v("Button", "voice");
			Intent intent = new Intent(Intent.ACTION_MAIN);
			intent.setClassName("com.example.sampleroute", "com.example.sampleroute.MainActivity");
			startActivity(intent);
		}
		else if(v == s1){
			Log.v("Button", "s1");
			SoundManagementNative.send(1, 1); // GEAR:1
		}
		else if(v == s2){
			Log.v("Button", "s2");
			SoundManagementNative.send(1, 2); // GEAR:2
		}
		else if(v == p1){
			Log.v("Button", "p1");
		}
		else if(v == p2){
			Log.v("Button", "p2");
		}
		else if(v == p3){
			Log.v("Button", "p3");
			SoundManagementNative.send(1, 3); // GEAR:3
		}
		else if(v == p4){
			Log.v("Button", "p4");
			SoundManagementNative.send(1, 4); // GEAR:4
		}
		else if(v == autoCruise){
			Log.v("Button", "autoCruise");
			SoundManagementNative.send(2, 1); // MODE:1
		}
		else if(v == normalCruise){
			Log.v("Button", "normalCruise");
			SoundManagementNative.send(2, 0); // MODE:0
		}
		else if(v == pursuit){
			Log.v("Button", "pursuit");
			SoundManagementNative.send(0, 0); // Disconnect- test
			finish();
		}
	}
}
