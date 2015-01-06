package com.ghostagent;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Random;
import java.util.StringTokenizer;

import com.design.DrawCenterView;
import com.design.DrawLeftView;
import com.design.DrawRightView;
import com.ghostagent.R;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.view.Window;
import android.view.View.OnClickListener;
import android.widget.EditText;
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
	/**
	 * flag for server connecting
	 */
	public static boolean bIsServerConnecting = false;
	/**
	 * menu item id
	 */
	private static final int MENU_ID_SETTINGS = 0;

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		getWindow().requestFeature(Window.FEATURE_ACTION_BAR);
		setContentView(R.layout.knight_rider);

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

		String text = null;
		try {
			BufferedInputStream stream = new BufferedInputStream(
				new FileInputStream(Environment.getExternalStorageDirectory().getPath() +
						    "/autowaretouch.txt"));

			byte[] buffer = new byte[256];
			stream.read(buffer);

			text = new String(buffer).trim();

			stream.close();
		} catch (FileNotFoundException e) {
		} catch (Exception e) {
			e.printStackTrace();
		}

		if (text != null) {
			String[] settings = text.split(",");
			if (validateIpAddress(settings[0]) && validatePortNumber(settings[1])) {
				address = settings[0];
				port = Integer.parseInt(settings[1]);
				if (SoundManagementNative.connect(address, port) == 0) {
					bIsServerConnecting = true;
				}
			}
		}

		// start recording
		bIsKnightRiding = true;
		startKnightRiding();
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		menu.add(Menu.NONE, MENU_ID_SETTINGS, Menu.NONE, "Settings");
		return true;
	}

	private boolean validateIpAddress(final String addressString) {
		if (addressString == null || addressString.isEmpty()) {
			Toast.makeText(this, "Please input IP Address",
				       Toast.LENGTH_LONG).show();
			return false;
		}

		return true;
	}

	private boolean validatePortNumber(final String portString) {
		if (portString == null || portString.isEmpty()) {
			Toast.makeText(this, "Please input Port Number",
				       Toast.LENGTH_LONG).show();
			return false;
		}

		boolean isValid;
		try {
			int port = Integer.parseInt(portString);
			if (port >= 0 && port <= 65535)
				isValid = true;
			else
				isValid = false;
		} catch (NumberFormatException e) {
			isValid = false;
		}

		if (!isValid)
			Toast.makeText(this, "Please input Port Number " +
				       "less than equal 65535",
				       Toast.LENGTH_LONG).show();

		return isValid;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		switch (item.getItemId()) {
		case MENU_ID_SETTINGS:
			LayoutInflater inflater = getLayoutInflater();

			final View view = inflater.inflate(
				R.layout.settings,
				(ViewGroup)findViewById(R.id.settingsLayout));

			if (address != null) {
				EditText addressEdit = (EditText)view.findViewById(R.id.ipAddress);
				addressEdit.setText(address);
			}

			if (port != 0) {
				EditText portEdit = (EditText)view.findViewById(R.id.portNumber);
				portEdit.setText(String.valueOf(port));
			}

			AlertDialog.Builder builder = new AlertDialog.Builder(this);
			builder.setTitle("Settings");
			builder.setView(view);
			builder.setPositiveButton(
				"OK",
				new DialogInterface.OnClickListener () {
					public void onClick(DialogInterface dialog, int which) {
						EditText addressEdit = (EditText)view.findViewById(R.id.ipAddress);
						String addressString = addressEdit.getText().toString();
						if (!validateIpAddress(addressString))
							return;

						EditText portEdit = (EditText)view.findViewById(R.id.portNumber);
						String portString = portEdit.getText().toString();
						if (!validatePortNumber(portString))
							return;

						String text = addressString + "," + portString;
						try {
							BufferedOutputStream stream = new BufferedOutputStream(
								new FileOutputStream(
									Environment.getExternalStorageDirectory().getPath() +
									"/autowaretouch.txt"));

							stream.write(text.getBytes("UTF-8"));

							stream.close();
						} catch (Exception e) {
							e.printStackTrace();
						}

						if (bIsServerConnecting) {
							bIsServerConnecting = false;
							SoundManagementNative.close();
						}

						address = addressString;
						port = Integer.parseInt(portString);
						if (SoundManagementNative.connect(address, port) == 0) {
							bIsServerConnecting = true;
						}
					}
				});
			builder.setNegativeButton(
				"Cancel",
				new DialogInterface.OnClickListener () {
					public void onClick(DialogInterface dialog, int which) {
					}
				});

			builder.create().show();

			return true;
		}

		return false;
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
		if (bIsServerConnecting) {
			bIsServerConnecting = false;
			SoundManagementNative.close();
		}
	}

	private int sendWayPoints(ArrayList<Double> wayPoints) {
		double[] way = new double[wayPoints.size()];

		for (int i = 0; i < wayPoints.size(); i++)
			way[i] = wayPoints.get(i).doubleValue();

		return SoundManagementNative.sendDoubleArray(3, way); // ROUTE
	}

	@Override
	public void onRestart() {
		Log.v("Log", "restart");
		File file = new File(
				Environment.getExternalStorageDirectory().getPath() + "/MapRoute.txt");
		try {
			FileReader reader = new FileReader(file);
			BufferedReader breader = new BufferedReader(reader);
			String line;
			ArrayList<Double> wayPoints = new ArrayList<Double>();
			while ((line = breader.readLine()) != null) {
				StringTokenizer token = new StringTokenizer(line, ",");
				if (token.countTokens() == 2) {
					wayPoints.add(Double.parseDouble(token.nextToken()));
					wayPoints.add(Double.parseDouble(token.nextToken()));
				}
			}
			breader.close();
			sendWayPoints(wayPoints);
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
			SoundManagementNative.send(2, 1); // RUN:1
		}
		else if(v == normalCruise){
			Log.v("Button", "normalCruise");
			SoundManagementNative.send(2, 2); // RUN:2
		}
		else if(v == pursuit){
			Log.v("Button", "pursuit");
			SoundManagementNative.send(0, 0); // Disconnect- test
			finish();
		}
	}
}
