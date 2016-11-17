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
import android.view.View.OnClickListener;
import android.view.ViewGroup;
import android.view.Window;
import android.widget.EditText;
import android.widget.Toast;

import com.client.CanDataGather;
import com.client.CanDataSender;
import com.client.CommandClient;
import com.client.InformationClient;
import com.design.DrawCenterView;
import com.design.DrawLeftView;
import com.design.DrawRightView;
import com.ui.ApplicationButton;
import com.ui.DriveButton;
import com.ui.GearButton;
import com.ui.S1Button;
import com.ui.S2Button;

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

public class SoundManagementActivity extends Activity implements OnClickListener {
	static final String LOG_TAG = SoundManagementActivity.class.getSimpleName();

	static final String AUTOWAREHOST = "192.168.0.197";

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
			Log.v(LOG_TAG, "send(CommandClient.GEAR, invalid failed");
//			return false;
		if (commandClient.send(CommandClient.MODE, driveButton.getMode()) < 0)
			Log.v(LOG_TAG, "send(CommandClient.MODE, invalid failed");
//			return false;
		if (commandClient.send(CommandClient.S1, s1Button.getMode()) < 0)
			Log.v(LOG_TAG, "send(CommandClient.S1, invalid failed");
//			return false;
		if (commandClient.send(CommandClient.S2, s2Button.getMode()) < 0)
			Log.v(LOG_TAG, "send(CommandClient.S2, invalid failed");
//			return false;
		return true;
	}
	void initiateConnections(final String address, final int commandPort, final int informationPort) {
		new Thread(new Runnable() {
			@Override
			public void run() {
				commandClient.connect(address, commandPort);
				informationClient.connect(address, informationPort);
				if (!startServerConnecting())
					stopServerConnecting();
			}
		}).start();
	}
	void sendExitCommand() {
		new Thread(new Runnable() {
			@Override
			public void run() {
				sendExitCommand();
			}
		}).start();
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
//		address = "";
		address = AUTOWAREHOST ;
		commandPort = 5666;
		informationPort = 5777;
		canDataSender = new CanDataSender( SoundManagementActivity.this,
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
			text = address + "\n" +
					commandPort + "\n" +
					informationPort + "\n" +
					"" + "\n" +   		//?gatheringTableString for upload to table@db3.ertl.jp
					"localhost" + "\n" +	//? update db3.ertl.jp table via ssh Tunnel Host?
					22 + "\n" +
					"ming" + "\n" +
					"5558" + "\n" +
					address		 + "\n" +                    //? forwardingRemoteHostString
					"5555"		 + "\n" +                    //? forwardingRemotePortString
					"end\n";
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

					initiateConnections(address, commandPort, informationPort);
				}
				if (validatePortNumber(settings[5]) &&
				    validatePortNumber(settings[7]) &&
				    validatePortNumber(settings[9])) {
					canDataSender = new CanDataSender(SoundManagementActivity.this,
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

//			builder.setTitle("設定");
			builder.setTitle("Setting");
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
							sendExitCommand();
						}

						address = addressString;
						commandPort = Integer.parseInt(commandPortString);
						informationPort = Integer.parseInt(informationPortString);

						initiateConnections(address, commandPort, informationPort);

						canDataSender = new CanDataSender( SoundManagementActivity.this,
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
				"Configuration",
//					"キャンセル",
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

			builder.setTitle("Gethering");
//			builder.setTitle("データ収集");
			builder.setView(view);
			builder.setNegativeButton(
					"Configuration",
//				"キャンセル",
				new DialogInterface.OnClickListener () {
					public void onClick(DialogInterface dialog, int which) {
					}
				});
			builder.create().show();

			return true;
		}

		return false;
	}
//MR: draw the middle part
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
// MR: get GPS info from SensorManager and send to Autoware port 5666
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
//MR InformationClient receive. d[0] < 0: error
// d[0] =BEACON; d[1]
// d[0] =ERROR; d[1]=0 RED, <>0 Yellow
// d[0] =CAN; d[1]= {DRIVE, BREAK, NEUTRAL, REVERSE}
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
								sendExitCommand();
								missBeacon = 0;
							}
						}
						continue;
					}

					if (data[1] < 0) {
						sendExitCommand();
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
							drawLeftView.setColor(COLOR_RED);
							drawRightView.setColor(COLOR_RED);
							drawCenterView.setColor(COLOR_RED);
							break;
						case DriveButton.AUTO:
							if (driveButton.getMode() != DriveButton.AUTO) {
								buttonHandler.post(new Runnable() {
									public void run() {
										driveButton.updateMode(DriveButton.AUTO);
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
					case InformationClient.NDT:
						switch (data[1]) {
						case 0:
							if (s1Button.getMode() == S1Button.OK) {
								buttonHandler.post(new Runnable() {
									public void run() {
										s1Button.updateMode(S1Button.NG);
									}
								});
							}
							break;
						case 1:
							if (s1Button.getMode() == S1Button.NG) {
								buttonHandler.post(new Runnable() {
									public void run() {
										s1Button.updateMode(S1Button.OK);
									}
								});
							}
							break;
						}
						break;
					case InformationClient.LF:
						switch (data[1]) {
						case 0:
							if (s2Button.getMode() == S2Button.OK) {
								buttonHandler.post(new Runnable() {
									public void run() {
										s2Button.updateMode(S2Button.NG);
									}
								});
							}
							break;
						case 1:
							if (s2Button.getMode() == S2Button.NG) {
								buttonHandler.post(new Runnable() {
									public void run() {
										s2Button.updateMode(S2Button.OK);
									}
								});
							}
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

		CanDataGather gather = new CanDataGather(SoundManagementActivity.this, CanDataGather.CAN_GATHER);
		gather.start();
	}

	public void startCarLinkBluetooth(View v) {
		canDataSender.start();

		CanDataGather gather = new CanDataGather(SoundManagementActivity.this, CanDataGather.CAR_LINK_BLUETOOTH);
		gather.start();
	}

	public void startCarLinkUSB(View v) {
		canDataSender.start();

		CanDataGather gather = new CanDataGather(SoundManagementActivity.this, CanDataGather.CAR_LINK_USB);
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
			sendExitCommand();
		}
	}

	private void sendWayPoints(final ArrayList<Double> wayPoints) {
		new Thread(new Runnable() {
			@Override
			public void run() {
				double[] way = new double[wayPoints.size()];

				for (int i = 0; i < wayPoints.size(); i++)
					way[i] = wayPoints.get(i).doubleValue();

				if (commandClient.sendRoute(way) < 0) {
					stopServerConnecting();
				}
			}
		}).start(); ;
	}

	@Override
	public void onRestart() {
		super.onRestart();
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
				sendWayPoints(wayPoints);
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
	int sendCommand(final int cmdType, final int cmd) {
		new Thread(new Runnable() {
			@Override
			public void run() {
				int data = commandClient.send(cmdType, cmd);
				if (data < 0)
					stopServerConnecting();
			}
		}).start();
		return 0;
	}
	@Override
	public void onClick(View v) {
		int data = -1;

		if (v == gearButton.drive) {
			data = sendCommand(CommandClient.GEAR, GearButton.DRIVE);
		} else if (v == gearButton.reverse) {
			data = sendCommand(CommandClient.GEAR, GearButton.REVERSE);
		} else if (v == gearButton.brake) {
			data = sendCommand(CommandClient.GEAR, GearButton.BRAKE);
		} else if (v == gearButton.neutral) {
			data = sendCommand(CommandClient.GEAR, GearButton.NEUTRAL);
		} else if (v == driveButton.auto) {
			data = sendCommand(CommandClient.MODE, DriveButton.AUTO);
		} else if (v == driveButton.normal) {
			data = sendCommand(CommandClient.MODE, DriveButton.NORMAL);
		} else if (v == driveButton.pursuit) {
			Toast.makeText(this, "Pursuit not supported yet", Toast.LENGTH_LONG).show();
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
			if (s1Button.getMode() == S1Button.OK)
				s1Button.updateMode(S1Button.OK);
			else
				s1Button.updateMode(S1Button.NG);
			data = sendCommand(CommandClient.S1, s1Button.getMode());
		} else if (v == s2Button.s2) {
			if (s2Button.getMode() == S2Button.OK)
				s2Button.updateMode(S2Button.OK);
			else
				s2Button.updateMode(S2Button.NG);
			data = sendCommand(CommandClient.S2, s2Button.getMode());
		}
		if (data < 0)
			stopServerConnecting();
	}
}
