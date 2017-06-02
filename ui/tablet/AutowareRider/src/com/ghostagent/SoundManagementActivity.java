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
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.StringTokenizer;

import com.design.DrawCenterView;
import com.design.DrawLeftView;
import com.design.DrawRightView;
import com.ghostagent.R;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.GpsStatus;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
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
	abstract class RadioButton {
		static final int INVALID = -1;

		private int mode = INVALID;

		int getMode() {
			return mode;
		}

		void updateMode(int mode) {
			if (mode == this.mode)
				mode = INVALID;
			this.mode = mode;

			refresh();
		}

		abstract void refresh();
	}

	class GearButton extends RadioButton {
		static final int DRIVE = 1;
		static final int REVERSE = 2;
		static final int BRAKE = 3;
		static final int NEUTRAL = 4;

		ImageButton drive;
		ImageButton reverse;
		ImageButton brake;
		ImageButton neutral;

		GearButton(OnClickListener listener) {
			drive = (ImageButton)findViewById(R.id.p3);
			drive.setOnClickListener(listener);
			reverse = (ImageButton)findViewById(R.id.p4);
			reverse.setOnClickListener(listener);
			brake = (ImageButton)findViewById(R.id.p1);
			brake.setOnClickListener(listener);
			neutral = (ImageButton)findViewById(R.id.p2);
			neutral.setOnClickListener(listener);

			refresh();
		}

		@Override
		void refresh() {
			drive.setImageResource(R.drawable.gear_d);
			reverse.setImageResource(R.drawable.gear_r);
			brake.setImageResource(R.drawable.gear_b);
			neutral.setImageResource(R.drawable.gear_n);

			switch (getMode()) {
			case DRIVE:
				drive.setImageResource(R.drawable.pressed_gear_d);
				break;
			case REVERSE:
				reverse.setImageResource(R.drawable.pressed_gear_r);
				break;
			case BRAKE:
				brake.setImageResource(R.drawable.pressed_gear_b);
				break;
			case NEUTRAL:
				neutral.setImageResource(R.drawable.pressed_gear_n);
				break;
			}
		}
	}

	class DriveButton extends RadioButton {
		static final int NORMAL = 0;
		static final int STR_AUTO = 1;
		static final int DRV_AUTO = 2;
		static final int AUTO = 3;
		static final int PURSUIT = 2;

		ImageButton auto;
		ImageButton normal;
		ImageButton pursuit;

		DriveButton(OnClickListener listener) {
			auto = (ImageButton)findViewById(R.id.autoCruise);
			auto.setOnClickListener(listener);
			normal = (ImageButton)findViewById(R.id.normalCruise);
			normal.setOnClickListener(listener);
			pursuit = (ImageButton)findViewById(R.id.pursuit);
			pursuit.setOnClickListener(listener);

			refresh();
		}

		@Override
		void refresh() {
			auto.setImageResource(R.drawable.autocruise);
			normal.setImageResource(R.drawable.normalcruise);
			pursuit.setImageResource(R.drawable.pursuit);

			switch (getMode()) {
			case AUTO:
				auto.setImageResource(R.drawable.pressed_autocruise);
				break;
			case NORMAL:
				normal.setImageResource(R.drawable.pressed_normalcruise);
				break;
			case PURSUIT:
				pursuit.setImageResource(R.drawable.pressed_pursuit);
				break;
			}
		}
	}

	class ApplicationButton extends RadioButton {
		static final int NAVIGATION = 1;
		static final int MAP = 2;

		ImageButton navigation;
		ImageButton map;

		ApplicationButton(OnClickListener listener) {
			navigation = (ImageButton)findViewById(R.id.air);
			navigation.setOnClickListener(listener);
			map = (ImageButton)findViewById(R.id.oil);
			map.setOnClickListener(listener);

			refresh();
		}
		@Override
		void refresh() {
			navigation.setImageResource(R.drawable.app_navi);
			map.setImageResource(R.drawable.app_map);

			switch (getMode()) {
			case NAVIGATION:
				navigation.setImageResource(R.drawable.pressed_app_navi);
				break;
			case MAP:
				map.setImageResource(R.drawable.pressed_app_map);
				break;
			}
		}
	}

	class S1Button extends RadioButton {
		static final int NG = 1;
		static final int OK = 2;

		ImageButton s1;

		S1Button(OnClickListener listener) {
			s1 = (ImageButton)findViewById(R.id.s1);
			s1.setOnClickListener(listener);

			refresh();
		}

		@Override
		void refresh() {
			s1.setImageResource(R.drawable.app_s1);

			switch (getMode()) {
			case NG:
				s1.setImageResource(R.drawable.pressed_app_s1_ng);
				break;
			case OK:
				s1.setImageResource(R.drawable.pressed_app_s1_ok);
				break;
			}
		}
	}

	class S2Button extends RadioButton {
		static final int NG = 1;
		static final int OK = 2;

		ImageButton s2;

		S2Button(OnClickListener listener) {
			s2 = (ImageButton)findViewById(R.id.s2);
			s2.setOnClickListener(listener);

			refresh();
		}

		@Override
		void refresh() {
			s2.setImageResource(R.drawable.app_s2);

			switch (getMode()) {
			case NG:
				s2.setImageResource(R.drawable.pressed_app_s2_ng);
				break;
			case OK:
				s2.setImageResource(R.drawable.pressed_app_s2_ok);
				break;
			}
		}
	}

	abstract class Client {
		private static final int TIMEOUT = 5;

		private int sockfd = -1;

		boolean isClosed() {
			if (sockfd < 0)
				return true;
			else
				return false;
		}

		synchronized boolean connect(String address, int port) {
			sockfd = SoundManagementNative.socket();
			if (sockfd < 0)
				return false;

			if (SoundManagementNative.connect(sockfd, TIMEOUT, address, port) < 0) {
				close();
				return false;
			}

			return true;
		}

		synchronized void close() {
			SoundManagementNative.close(sockfd);
			sockfd = -1;
		}

		void sendInt(int arg0) {
			SoundManagementNative.sendInt(sockfd, TIMEOUT, arg0);
		}

		void sendIntTuple(int arg0, int arg1) {
			SoundManagementNative.sendIntTuple(sockfd, TIMEOUT, arg0, arg1);
		}

		void sendDoubleArray(double arg0[]) {
			SoundManagementNative.sendDoubleArray(sockfd, TIMEOUT, arg0);
		}

		int recvInt() {
			return SoundManagementNative.recvInt(sockfd, TIMEOUT);
		}

		int recvNDT() {
			return SoundManagementNative.recvNDT(sockfd, TIMEOUT);
		}
	}

	class CommandClient extends Client {
		static final int EXIT = 0;
		static final int GEAR = 1;
		static final int MODE = 2;
		static final int ROUTE = 3;
		static final int S1 = 4;
		static final int S2 = 5;
		static final int POSE = 6;

		static final int EXIT_DESTROY_ACTIVITY = 0;
		static final int EXIT_UPDATE_CONFIGURATION = 1;
		static final int EXIT_EXCEED_ERROR_LIMIT = -1;
		static final int EXIT_RECEIVE_BROKEN_PACKET = -2;

		synchronized int send(int type, int command) {
			if (isClosed())
				return -1;

			sendIntTuple(type, command);

			return recvInt();
		}

		synchronized int sendRoute(double[] latlong) {
			if (isClosed())
				return -1;

			sendIntTuple(ROUTE, latlong.length * (Double.SIZE / 8));

			sendDoubleArray(latlong);

			return recvInt();
		}

		synchronized int sendPose(double[] pose) {
			if (isClosed())
				return -1;

			sendIntTuple(POSE, pose.length * (Double.SIZE / 8));

			sendDoubleArray(pose);

			return recvInt();
		}
	}

	class InformationClient extends Client {
		static final int BEACON = 0;
		static final int ERROR = 1;
		static final int CAN = 2;
		static final int MODE = 3;
		static final int NDT = 4;
		static final int LF = 5;

		static final int MISS_BEACON_LIMIT = 10;

		static final int CAN_SHIFT_BRAKE = 0x00;
		static final int CAN_SHIFT_DRIVE = 0x10;
		static final int CAN_SHIFT_NEUTRAL = 0x20;
		static final int CAN_SHIFT_REVERSE = 0x40;

		synchronized int[] recv(int response) {
			int[] data = new int[2];

			if (isClosed()) {
				data[0] = -1;
				data[1] = -1;
				return data;
			}

			data[0] = recvInt();
			if (data[0] < 0) {
				data[1] = -1;
				return data;
			}

			if (data[0] == NDT)
				data[1] = recvNDT();
			else
				data[1] = recvInt();
			if (data[1] < 0)
				return data;

			sendInt(response);

			return data;
		}
	}

	class CanDataSender {
		private boolean isRunning = false;

		private String table;
		private String terminal;
		private String str;
		private String sp;
		private String sh;
		private String su;
		private String spss;
		private String lp;
		private String fh;
		private String fp;
		private Intent intent;

		CanDataSender(String table, String terminal, String str, String sp, String sh,
			      String su, String spss, String lp, String fh, String fp) {
			this.table = table;
			this.terminal = terminal;
			this.str = str;
			this.sp = sp;
			this.sh = sh;
			this.su = su;
			this.spss = spss;
			this.lp = lp;
			this.fh = fh;
			this.fp = fp;

			intent = new Intent(Intent.ACTION_MAIN);
			intent.setClassName("com.example.candatasender",
					    "com.example.candatasender.service.CanDataAutoSend");
		}

		boolean isRunning() {
			return isRunning;
		}

		void start() {
			intent.putExtra("table", table);
			intent.putExtra("terminal", terminal);
			intent.putExtra("pfd", true);
			intent.putExtra("str", str);
			intent.putExtra("sp", sp);
			intent.putExtra("sh", sh);
			intent.putExtra("su", su);
			intent.putExtra("spss", spss);
			intent.putExtra("lp", lp);
			intent.putExtra("fh", fh);
			intent.putExtra("fp", fp);

			startService(intent);
			isRunning = true;
		}

		void stop() {
			isRunning = false;
			stopService(intent);
		}

		String getTable() {
			return table;
		}

		String getTerminal() {
			return terminal;
		}

		String getStr() {
			return str;
		}

		String getSp() {
			return sp;
		}

		String getSh() {
			return sh;
		}

		String getSu() {
			return su;
		}

		String getSpss() {
			return spss;
		}

		String getLp() {
			return lp;
		}

		String getFh() {
			return fh;
		}

		String getFp() {
			return fp;
		}
	}

	class CanDataGather {
		static final int CAN_GATHER = 0;
		static final int CAR_LINK_BLUETOOTH = 1;
		static final int CAR_LINK_USB = 2;

		private Intent intent;

		CanDataGather(int type) {
			intent = new Intent(Intent.ACTION_MAIN);
			switch (type) {
			case CAN_GATHER:
				intent.setClassName("com.ecsgr.android.cangather",
						    "com.ecsgr.android.cangather.MainActivity");
				break;
			case CAR_LINK_BLUETOOTH:
				intent.setClassName("com.metaprotocol.android.carlinkcan232",
						    "com.metaprotocol.android.carlinkcan232.CarLinkMainActivity");
				break;
			case CAR_LINK_USB:
				intent.setClassName("com.metaprotocol.android.carlinkcanusbaccessory",
						    "com.metaprotocol.android.carlinkcanusbaccessory.CarLinkMainActivity");
				break;
			}
		}

		void start() {
			startActivity(intent);
		}
	}

	GearButton gearButton;
	DriveButton driveButton;
	ApplicationButton applicationButton;
	S1Button s1Button;
	S2Button s2Button;

	Handler buttonHandler;

	CommandClient commandClient;
	InformationClient informationClient;

	CanDataSender canDataSender;

	/**
	 * server address
	 */
	String address;
	/**
	 * server command port
	 */
	int commandPort;
	/**
	 * server information port
	 */
	int informationPort;
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
	 * flag for knight riding
	 */
	boolean bIsKnightRiding = false;
	/**
	 * flag for server connecting
	 */
	boolean bIsServerConnecting = false;
	/**
	 * menu item id
	 */
	private static final int MENU_ID_SETTINGS = 0;
	private static final int MENU_ID_DATA_GATHERING = 1;
	/**
	 * location manager
	 */
	private LocationManager locationManager;
	private LocationListener locationListener;
	private GpsStatus.NmeaListener nmeaListener;
	private double[] locationLatLong = new double[2];
	private double[] locationHeight = new double[1];
	private boolean bUsesGPS = false;
	private boolean bUsesNetwork = false;
	private boolean bExistsLatLong = false;
	private boolean bExistsHeight = false;
	/**
	 * sensor manager
	 */
	private SensorManager sensorManager;
	private SensorEventListener sensorListener;
	private float[] sensorGravity = new float[3];
	private float[] sensorGeomagnetic = new float[3];
	private boolean bUsesGravity = false;
	private boolean bUsesGeomagnetic = false;
	private boolean bExistsGravity = false;
	private boolean bExistsGeomagnetic = false;
	/**
	 * meter color
	 */
	private static final int COLOR_DARK_RED = 0xff5b1100;
	private static final int COLOR_RED = 0xffb62200;
	private static final int COLOR_BLUE = 0xff0000fb;
	private static final int COLOR_YELLOW = 0xfffffb00;

	private String getMacAddress() {
		WifiManager wifiManager = (WifiManager)getSystemService(Context.WIFI_SERVICE);
		WifiInfo wifiInfo = wifiManager.getConnectionInfo();
		return wifiInfo.getMacAddress();
	}

	private boolean startServerConnecting() {
		bIsServerConnecting = true;

		drawLeftView.setColor(COLOR_RED);
		drawRightView.setColor(COLOR_RED);
		drawCenterView.setColor(COLOR_RED);

		if (commandClient.send(CommandClient.GEAR, gearButton.getMode()) < 0)
			return false;
		if (commandClient.send(CommandClient.MODE, driveButton.getMode()) < 0)
			return false;
		if (commandClient.send(CommandClient.S1, s1Button.getMode()) < 0)
			return false;
		if (commandClient.send(CommandClient.S2, s2Button.getMode()) < 0)
			return false;

		return true;
	}

	private void stopServerConnecting() {
		drawLeftView.setColor(COLOR_DARK_RED);
		drawRightView.setColor(COLOR_DARK_RED);
		drawCenterView.setColor(COLOR_DARK_RED);

		bIsServerConnecting = false;

		if (!commandClient.isClosed())
			commandClient.close();
		if (!informationClient.isClosed())
			informationClient.close();
	}

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		getWindow().requestFeature(Window.FEATURE_ACTION_BAR);
		setContentView(R.layout.knight_rider);

		// center expression
		drawLeftView = (DrawLeftView) findViewById(R.id.leftView);
		drawLeftView.setColor(COLOR_DARK_RED);
		drawRightView = (DrawRightView) findViewById(R.id.rightView);
		drawRightView.setColor(COLOR_DARK_RED);
		drawCenterView = (DrawCenterView) findViewById(R.id.centerView);
		drawCenterView.setColor(COLOR_DARK_RED);

		// set buttons
		gearButton = new GearButton(this);
		driveButton = new DriveButton(this);
		applicationButton = new ApplicationButton(this);
		s1Button = new S1Button(this);
		s2Button = new S2Button(this);

		buttonHandler = new Handler();

		commandClient = new CommandClient();
		informationClient = new InformationClient();

		// default settings
		address = "";
		commandPort = 5666;
		informationPort = 5777;
		canDataSender = new CanDataSender(
			"",
			getMacAddress(),
			"AutowareRider",
			"22",
			"",
			"",
			"",
			"5558",
			"127.0.0.1",
			"5555");

		String text = null;
		try {
			BufferedInputStream stream = new BufferedInputStream(
				new FileInputStream(Environment.getExternalStorageDirectory().getPath() +
						    "/autowarerider.txt"));

			byte[] buffer = new byte[1024];
			stream.read(buffer);

			text = new String(buffer).trim();

			stream.close();
		} catch (FileNotFoundException e) {
		} catch (Exception e) {
			e.printStackTrace();
		}

		if (text != null) {
			String[] settings = text.split("\n");
			if (settings.length == 11) {
				if (validateIpAddress(settings[0]) &&
				    validatePortNumber(settings[1]) &&
				    validatePortNumber(settings[2])) {
					address = settings[0];
					commandPort = Integer.parseInt(settings[1]);
					informationPort = Integer.parseInt(settings[2]);

					if (commandClient.connect(address, commandPort) &&
					    informationClient.connect(address, informationPort)) {
						if (!startServerConnecting())
							stopServerConnecting();
					} else
						stopServerConnecting();
				}
				if (validatePortNumber(settings[5]) &&
				    validatePortNumber(settings[7]) &&
				    validatePortNumber(settings[9])) {
					canDataSender = new CanDataSender(
						settings[3],
						getMacAddress(),
						"AutowareRider",
						settings[5],
						settings[4],
						settings[6],
						"",
						settings[7],
						settings[8],
						settings[9]);
				}
			}
		}

		// register location listener
		locationManager = (LocationManager)getSystemService(Context.LOCATION_SERVICE);
		locationListener = new LocationListener() {
				@Override
				public void onLocationChanged(Location location) {
					locationLatLong[0] = location.getLatitude();
					locationLatLong[1] = location.getLongitude();
					bExistsLatLong = true;
				}
				@Override
				public void onProviderDisabled(String provider) {
				}
				@Override
				public void onProviderEnabled(String provider) {
				}
				@Override
				public void onStatusChanged(String provider, int status, Bundle extras) {
				}
			};
		nmeaListener = new GpsStatus.NmeaListener() {
				@Override
				public void onNmeaReceived(long timestamp, String nmea) {
					String[] data = nmea.split(",");
					if (data[0].equals("$GPGGA") && !data[9].isEmpty()) {
						locationHeight[0] = Double.parseDouble(data[9]);
						bExistsHeight = true;
					}
				}
			};
		if (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
			locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, locationListener);
			locationManager.addNmeaListener(nmeaListener);
			bUsesGPS = true;
		} else if (locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
			locationManager.requestLocationUpdates(LocationManager.NETWORK_PROVIDER, 0, 0, locationListener);
			bUsesNetwork = true;
		}
		if (!bUsesGPS) {
			locationHeight[0] = 0;
			bExistsHeight = true;
		}
		if (!bUsesGPS && !bUsesNetwork) {
			locationLatLong[0] = 0;
			locationLatLong[1] = 0;
			bExistsLatLong = true;
		}

		// register sensor listener
		sensorManager = (SensorManager)getSystemService(Context.SENSOR_SERVICE);
		sensorListener = new SensorEventListener() {
				@Override
				public void onAccuracyChanged(Sensor sensor, int accuracy) {
				}
				@Override
				public void onSensorChanged(SensorEvent event) {
					if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
						sensorGravity = event.values.clone();
						bExistsGravity = true;
					}
					if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
						sensorGeomagnetic = event.values.clone();
						bExistsGeomagnetic = true;
					}
				}
			};
		List<Sensor> sensors = sensorManager.getSensorList(Sensor.TYPE_ALL);
		for (Sensor sensor : sensors) {
			if (sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
				sensorManager.registerListener(sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
				bUsesGravity = true;
			}
			if (sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
				sensorManager.registerListener(sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
				bUsesGeomagnetic = true;
			}
		}
		if (!bUsesGravity) {
			sensorGravity[0] = 0;
			sensorGravity[1] = 0;
			sensorGravity[2] = 0;
			bExistsGravity = true;
		}
		if (!bUsesGeomagnetic) {
			sensorGeomagnetic[0] = 0;
			sensorGeomagnetic[1] = 0;
			sensorGeomagnetic[2] = 0;
			bExistsGeomagnetic = true;
		}

		bIsKnightRiding = true;

		// start recording
		startKnightRiding();

		startCommandSender();
		startInformationReceiver();
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		menu.add(Menu.NONE, MENU_ID_SETTINGS, Menu.NONE, "設定");
		menu.add(Menu.NONE, MENU_ID_DATA_GATHERING, Menu.NONE, "データ収集");
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
		LayoutInflater inflater = getLayoutInflater();
		AlertDialog.Builder builder = new AlertDialog.Builder(this);
		final View view;

		switch (item.getItemId()) {
		case MENU_ID_SETTINGS:
			view = inflater.inflate(
				R.layout.settings,
				(ViewGroup)findViewById(R.id.settingsLayout));

			if (address != null) {
				EditText addressEdit = (EditText)view.findViewById(R.id.rosIpAddress);
				addressEdit.setText(address);
			}

			if (commandPort != 0) {
				EditText portEdit = (EditText)view.findViewById(R.id.rosCommandPortNumber);
				portEdit.setText(String.valueOf(commandPort));
			}

			if (informationPort != 0) {
				EditText portEdit = (EditText)view.findViewById(R.id.rosInformationPortNumber);
				portEdit.setText(String.valueOf(informationPort));
			}

			if (canDataSender.getTable() != null) {
				EditText tableEdit = (EditText)view.findViewById(R.id.gatheringTableName);
				tableEdit.setText(String.valueOf(canDataSender.getTable()));
			}

			if (canDataSender.getSh() != null) {
				EditText hostEdit = (EditText)view.findViewById(R.id.sshHostName);
				hostEdit.setText(String.valueOf(canDataSender.getSh()));
			}

			if (canDataSender.getSp() != null) {
				EditText portEdit = (EditText)view.findViewById(R.id.sshPortNumber);
				portEdit.setText(String.valueOf(canDataSender.getSp()));
			}

			if (canDataSender.getSu() != null) {
				EditText userEdit = (EditText)view.findViewById(R.id.sshUserName);
				userEdit.setText(String.valueOf(canDataSender.getSu()));
			}

			if (canDataSender.getSpss() != null) {
				EditText passwordEdit = (EditText)view.findViewById(R.id.sshPassword);
				passwordEdit.setText(String.valueOf(canDataSender.getSpss()));
			}

			if (canDataSender.getLp() != null) {
				EditText portEdit = (EditText)view.findViewById(R.id.forwardingLocalPortNumber);
				portEdit.setText(String.valueOf(canDataSender.getLp()));
			}

			if (canDataSender.getFh() != null) {
				EditText hostEdit = (EditText)view.findViewById(R.id.forwardingRemoteHostName);
				hostEdit.setText(String.valueOf(canDataSender.getFh()));
			}

			if (canDataSender.getFp() != null) {
				EditText portEdit = (EditText)view.findViewById(R.id.forwardingRemotePortNumber);
				portEdit.setText(String.valueOf(canDataSender.getFp()));
			}

			builder.setTitle("設定");
			builder.setView(view);
			builder.setPositiveButton(
				"OK",
				new DialogInterface.OnClickListener () {
					public void onClick(DialogInterface dialog, int which) {
						EditText addressEdit = (EditText)view.findViewById(R.id.rosIpAddress);
						String addressString = addressEdit.getText().toString();
						if (!validateIpAddress(addressString))
							return;

						EditText commandPortEdit = (EditText)view.findViewById(R.id.rosCommandPortNumber);
						String commandPortString = commandPortEdit.getText().toString();
						if (!validatePortNumber(commandPortString))
							return;

						EditText informationPortEdit = (EditText)view.findViewById(R.id.rosInformationPortNumber);
						String informationPortString = informationPortEdit.getText().toString();
						if (!validatePortNumber(informationPortString))
							return;

						EditText gatheringTableEdit = (EditText)view.findViewById(R.id.gatheringTableName);
						String gatheringTableString = gatheringTableEdit.getText().toString();

						EditText sshHostEdit = (EditText)view.findViewById(R.id.sshHostName);
						String sshHostString = sshHostEdit.getText().toString();

						EditText sshPortEdit = (EditText)view.findViewById(R.id.sshPortNumber);
						String sshPortString = sshPortEdit.getText().toString();
						if (!validatePortNumber(sshPortString))
							return;

						EditText sshUserEdit = (EditText)view.findViewById(R.id.sshUserName);
						String sshUserString = sshUserEdit.getText().toString();

						EditText sshPasswordEdit = (EditText)view.findViewById(R.id.sshPassword);
						String sshPasswordString = sshPasswordEdit.getText().toString();

						EditText forwardingLocalPortEdit = (EditText)view.findViewById(R.id.forwardingLocalPortNumber);
						String forwardingLocalPortString = forwardingLocalPortEdit.getText().toString();
						if (!validatePortNumber(forwardingLocalPortString))
							return;

						EditText forwardingRemoteHostEdit = (EditText)view.findViewById(R.id.forwardingRemoteHostName);
						String forwardingRemoteHostString = forwardingRemoteHostEdit.getText().toString();

						EditText forwardingRemotePortEdit = (EditText)view.findViewById(R.id.forwardingRemotePortNumber);
						String forwardingRemotePortString = forwardingRemotePortEdit.getText().toString();
						if (!validatePortNumber(forwardingRemotePortString))
							return;

						// don't save ssh password
						String text =
							addressString + "\n" +
							commandPortString + "\n" +
							informationPortString + "\n" +
							gatheringTableString + "\n" +
							sshHostString + "\n" +
							sshPortString + "\n" +
							sshUserString + "\n" +
							forwardingLocalPortString + "\n" +
							forwardingRemoteHostString + "\n" +
							forwardingRemotePortString + "\n" +
							"end\n";
						try {
							BufferedOutputStream stream = new BufferedOutputStream(
								new FileOutputStream(
									Environment.getExternalStorageDirectory().getPath() +
									"/autowarerider.txt"));

							stream.write(text.getBytes("UTF-8"));

							stream.close();
						} catch (Exception e) {
							e.printStackTrace();
						}

						if (bIsServerConnecting) {
							commandClient.send(
								CommandClient.EXIT,
								CommandClient.EXIT_UPDATE_CONFIGURATION);
							stopServerConnecting();
						}

						address = addressString;
						commandPort = Integer.parseInt(commandPortString);
						informationPort = Integer.parseInt(informationPortString);

						if (commandClient.connect(address, commandPort) &&
						    informationClient.connect(address, informationPort)) {
							if (!startServerConnecting())
								stopServerConnecting();
						} else
							stopServerConnecting();

						canDataSender = new CanDataSender(
							gatheringTableString,
							getMacAddress(),
							"AutowareRider",
							sshPortString,
							sshHostString,
							sshUserString,
							sshPasswordString,
							forwardingLocalPortString,
							forwardingRemoteHostString,
							forwardingRemotePortString);
					}
				});
			builder.setNegativeButton(
				"キャンセル",
				new DialogInterface.OnClickListener () {
					public void onClick(DialogInterface dialog, int which) {
					}
				});
			builder.create().show();

			return true;
		case MENU_ID_DATA_GATHERING:
			view = inflater.inflate(
				R.layout.data_gathering,
				(ViewGroup)findViewById(R.id.dataGatheringLayout));

			builder.setTitle("データ収集");
			builder.setView(view);
			builder.setNegativeButton(
				"キャンセル",
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
					drawLeftView.drawView(rnd.nextInt(4) + 1);
					drawRightView.drawView(rnd.nextInt(4) + 1);
					drawCenterView.drawView(rnd.nextInt(7) + 1);
					try {
						Thread.sleep(100);
					} catch (Exception e) {
					}
				}
			}
		}).start();
	}

	public void startCommandSender() {
		new Thread(new Runnable() {
			public void run() {
				while (bIsKnightRiding) {
					if (applicationButton.getMode() == ApplicationButton.MAP) {
						if (bExistsLatLong && bExistsHeight && bExistsGravity && bExistsGeomagnetic) {
							float[] inR = new float[16];
							float[] I = new float[16];
							SensorManager.getRotationMatrix(inR, I, sensorGravity, sensorGeomagnetic);

							float[] outR = new float[16];
							SensorManager.remapCoordinateSystem(inR, SensorManager.AXIS_X, SensorManager.AXIS_Y, outR);

							float[] attitude = new float[3];
							SensorManager.getOrientation(outR, attitude);

							double[] pose = new double[6];
							pose[0] = locationLatLong[0]; // north latitude (degrees)
							pose[1] = locationLatLong[1]; // east longitude (degrees)
							pose[2] = locationHeight[0];  // height above sea level (m)
							pose[3] = -(double)attitude[0]; // azimuth (rad)
							pose[4] = (double)attitude[1]; // pitch (rad)
							pose[5] = (double)attitude[2]; // roll (rad)
							int data = commandClient.sendPose(pose);
							if (data < 0)
								stopServerConnecting();
						}
					}
					try {
						Thread.sleep(1000);
					} catch (Exception e) {
					}
				}
			}
		}).start();
	}

	public void startInformationReceiver() {
		new Thread(new Runnable() {
			public void run() {
				int missBeacon = 0;

				while (bIsKnightRiding) {
					int[] data = informationClient.recv(0);

					if (data[0] < 0) {
						if (bIsServerConnecting) {
							if (missBeacon < InformationClient.MISS_BEACON_LIMIT)
								missBeacon++;
							else {
								commandClient.send(
									CommandClient.EXIT,
									CommandClient.EXIT_EXCEED_ERROR_LIMIT);
								stopServerConnecting();
								missBeacon = 0;
							}
						}
						continue;
					}

					if (data[1] < 0) {
						commandClient.send(
							CommandClient.EXIT,
							CommandClient.EXIT_RECEIVE_BROKEN_PACKET);
						stopServerConnecting();
						missBeacon = 0;
						continue;
					}

					switch (data[0]) {
					case InformationClient.BEACON:
						missBeacon = 0;
						break;
					case InformationClient.ERROR:
						if (data[1] == 0) {
							drawLeftView.setColor(COLOR_RED);
							drawRightView.setColor(COLOR_RED);
							drawCenterView.setColor(COLOR_RED);
						} else {
							drawLeftView.setColor(COLOR_YELLOW);
							drawRightView.setColor(COLOR_YELLOW);
							drawCenterView.setColor(COLOR_YELLOW);
						}
						break;
					case InformationClient.CAN:
						switch (data[1]) {
						case InformationClient.CAN_SHIFT_BRAKE:
							if (gearButton.getMode() != GearButton.BRAKE) {
								buttonHandler.post(new Runnable() {
									public void run() {
										gearButton.updateMode(GearButton.BRAKE);
									}
								});
							}
							break;
						case InformationClient.CAN_SHIFT_DRIVE:
							if (gearButton.getMode() != GearButton.DRIVE) {
								buttonHandler.post(new Runnable() {
									public void run() {
										gearButton.updateMode(GearButton.DRIVE);
									}
								});
							}
							break;
						case InformationClient.CAN_SHIFT_NEUTRAL:
							if (gearButton.getMode() != GearButton.NEUTRAL) {
								buttonHandler.post(new Runnable() {
									public void run() {
										gearButton.updateMode(GearButton.NEUTRAL);
									}
								});
							}
							break;
						case InformationClient.CAN_SHIFT_REVERSE:
							if (gearButton.getMode() != GearButton.REVERSE) {
								buttonHandler.post(new Runnable() {
									public void run() {
										gearButton.updateMode(GearButton.REVERSE);
									}
								});
							}
							break;
						}
						break;
					case InformationClient.MODE:
						switch (data[1]) {
						case DriveButton.NORMAL:
							if (driveButton.getMode() != DriveButton.NORMAL) {
								buttonHandler.post(new Runnable() {
									public void run() {
										driveButton.updateMode(DriveButton.NORMAL);
									}
								});
							}
							if (s1Button.getMode() == S1Button.NG) {
								buttonHandler.post(new Runnable() {
									public void run() {
										s1Button.updateMode(S1Button.NG);
									}
								});
							}
							if (s2Button.getMode() == S2Button.NG) {
								buttonHandler.post(new Runnable() {
									public void run() {
										s2Button.updateMode(S2Button.NG);
									}
								});
							}
							drawLeftView.setColor(COLOR_RED);
							drawRightView.setColor(COLOR_RED);
							drawCenterView.setColor(COLOR_RED);
							break;
						case DriveButton.STR_AUTO:
							if (driveButton.getMode() != DriveButton.AUTO) {
								buttonHandler.post(new Runnable() {
									public void run() {
										driveButton.updateMode(DriveButton.AUTO);
									}
								});
							}
							if (s1Button.getMode() != S1Button.NG) {
								buttonHandler.post(new Runnable() {
									public void run() {
										s1Button.updateMode(S1Button.NG);
									}
								});
							}
							if (s2Button.getMode() == S2Button.NG) {
								buttonHandler.post(new Runnable() {
									public void run() {
										s2Button.updateMode(S2Button.NG);
									}
								});
							}
							drawLeftView.setColor(COLOR_BLUE);
							drawRightView.setColor(COLOR_BLUE);
							drawCenterView.setColor(COLOR_BLUE);
							break;
						case DriveButton.DRV_AUTO:
							if (driveButton.getMode() != DriveButton.AUTO) {
								buttonHandler.post(new Runnable() {
									public void run() {
										driveButton.updateMode(DriveButton.AUTO);
									}
								});
							}
							if (s1Button.getMode() == S1Button.NG) {
								buttonHandler.post(new Runnable() {
									public void run() {
										s1Button.updateMode(S1Button.NG);
									}
								});
							}
							if (s2Button.getMode() != S2Button.NG) {
								buttonHandler.post(new Runnable() {
									public void run() {
										s2Button.updateMode(S2Button.NG);
									}
								});
							}
							drawLeftView.setColor(COLOR_BLUE);
							drawRightView.setColor(COLOR_BLUE);
							drawCenterView.setColor(COLOR_BLUE);
							break;
						case DriveButton.AUTO:
							if (driveButton.getMode() != DriveButton.AUTO) {
								buttonHandler.post(new Runnable() {
									public void run() {
										driveButton.updateMode(DriveButton.AUTO);
									}
								});
							}
							if (s1Button.getMode() != S1Button.NG) {
								buttonHandler.post(new Runnable() {
									public void run() {
										s1Button.updateMode(S1Button.NG);
									}
								});
							}
							if (s2Button.getMode() != S2Button.NG) {
								buttonHandler.post(new Runnable() {
									public void run() {
										s2Button.updateMode(S2Button.NG);
									}
								});
							}
							drawLeftView.setColor(COLOR_BLUE);
							drawRightView.setColor(COLOR_BLUE);
							drawCenterView.setColor(COLOR_BLUE);
							break;
						default:
							drawLeftView.setColor(COLOR_YELLOW);
							drawRightView.setColor(COLOR_YELLOW);
							drawCenterView.setColor(COLOR_YELLOW);
							break;
						}
						break;
					}
				}
			}
		}).start();
	}

	public void startCanGather(View v) {
		canDataSender.start();

		CanDataGather gather = new CanDataGather(CanDataGather.CAN_GATHER);
		gather.start();
	}

	public void startCarLinkBluetooth(View v) {
		canDataSender.start();

		CanDataGather gather = new CanDataGather(CanDataGather.CAR_LINK_BLUETOOTH);
		gather.start();
	}

	public void startCarLinkUSB(View v) {
		canDataSender.start();

		CanDataGather gather = new CanDataGather(CanDataGather.CAR_LINK_USB);
		gather.start();
	}

	@Override
	protected void onDestroy() {
		super.onDestroy();
		bIsKnightRiding = false;

		if (bUsesGravity || bUsesGeomagnetic)
			sensorManager.unregisterListener(sensorListener);
		bUsesGravity = false;
		bUsesGeomagnetic = false;
		bExistsGravity = false;
		bExistsGeomagnetic = false;

		if (bUsesGPS)
			locationManager.removeNmeaListener(nmeaListener);
		if (bUsesGPS || bUsesNetwork)
			locationManager.removeUpdates(locationListener);
		bUsesGPS = false;
		bUsesNetwork = false;
		bExistsLatLong = false;
		bExistsHeight = false;

		if (bIsServerConnecting) {
			commandClient.send(
				CommandClient.EXIT,
				CommandClient.EXIT_DESTROY_ACTIVITY);
			stopServerConnecting();
		}
	}

	private int sendWayPoints(ArrayList<Double> wayPoints) {
		double[] way = new double[wayPoints.size()];

		for (int i = 0; i < wayPoints.size(); i++)
			way[i] = wayPoints.get(i).doubleValue();

		return commandClient.sendRoute(way);
	}

	@Override
	public void onRestart() {
		if (applicationButton.getMode() == ApplicationButton.NAVIGATION) {
			applicationButton.updateMode(ApplicationButton.NAVIGATION);
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
				if (sendWayPoints(wayPoints) < 0)
					stopServerConnecting();
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		if (canDataSender.isRunning())
			canDataSender.stop();
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
		int data = -1;

		if (v == gearButton.drive) {
			data = commandClient.send(CommandClient.GEAR, GearButton.DRIVE);
		} else if (v == gearButton.reverse) {
			data = commandClient.send(CommandClient.GEAR, GearButton.REVERSE);
		} else if (v == gearButton.brake) {
			data = commandClient.send(CommandClient.GEAR, GearButton.BRAKE);
		} else if (v == gearButton.neutral) {
			data = commandClient.send(CommandClient.GEAR, GearButton.NEUTRAL);
		} else if (v == driveButton.auto) {
			data = commandClient.send(CommandClient.MODE, DriveButton.AUTO);
		} else if (v == driveButton.normal) {
			data = commandClient.send(CommandClient.MODE, DriveButton.NORMAL);
		} else if (v == driveButton.pursuit) {
			finish();
			data = 0;
		} else if (v == applicationButton.navigation) {
			applicationButton.updateMode(ApplicationButton.NAVIGATION);
			Intent intent = new Intent(Intent.ACTION_MAIN);
			intent.setClassName("com.example.autowareroute",
					    "com.example.autowareroute.MainActivity");
			startActivity(intent);
			data = 0;
		} else if (v == applicationButton.map) {
			if (applicationButton.getMode() == ApplicationButton.MAP)
				applicationButton.updateMode(ApplicationButton.MAP);
			else {
				if (bExistsLatLong && bExistsHeight && bExistsGravity && bExistsGeomagnetic)
					applicationButton.updateMode(ApplicationButton.MAP);
				else
					Toast.makeText(this, "Sensor data has not been received yet", Toast.LENGTH_LONG).show();
			}
			data = 0;
		} else if (v == s1Button.s1) {
			if (s1Button.getMode() != S1Button.NG) {
				if (s2Button.getMode() != S2Button.NG)
					data = commandClient.send(CommandClient.MODE, DriveButton.STR_AUTO);
				else
					data = commandClient.send(CommandClient.MODE, DriveButton.AUTO);
			} else {
				if (s2Button.getMode() != S2Button.NG)
					data = commandClient.send(CommandClient.MODE, DriveButton.NORMAL);
				else
					data = commandClient.send(CommandClient.MODE, DriveButton.DRV_AUTO);
			}
		} else if (v == s2Button.s2) {
			if (s2Button.getMode() != S2Button.NG) {
				if (s1Button.getMode() != S1Button.NG)
					data = commandClient.send(CommandClient.MODE, DriveButton.DRV_AUTO);
				else
					data = commandClient.send(CommandClient.MODE, DriveButton.AUTO);
			} else {
				if (s1Button.getMode() != S1Button.NG)
					data = commandClient.send(CommandClient.MODE, DriveButton.NORMAL);
				else
					data = commandClient.send(CommandClient.MODE, DriveButton.STR_AUTO);
			}
		}

		if (data < 0)
			stopServerConnecting();
	}
}
