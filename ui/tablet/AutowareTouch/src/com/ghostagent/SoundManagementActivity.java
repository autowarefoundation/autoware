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
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.graphics.Color;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
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
	abstract class RadioButton {
		static final int NONE = 0;

		private int mode = NONE;

		int getMode() {
			return mode;
		}

		void updateMode(int mode) {
			if (mode == this.mode)
				mode = NONE;
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
			drive = (ImageButton)findViewById(R.id.s1);
			drive.setOnClickListener(listener);
			reverse = (ImageButton)findViewById(R.id.s2);
			reverse.setOnClickListener(listener);
			brake = (ImageButton)findViewById(R.id.p3);
			brake.setOnClickListener(listener);
			neutral = (ImageButton)findViewById(R.id.p4);
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
		static final int AUTO = 1;
		static final int NORMAL = 2;
		static final int PURSUIT = 3;

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
			auto.setImageResource(R.drawable.autodrive);
			normal.setImageResource(R.drawable.normaldrive);
			pursuit.setImageResource(R.drawable.pursuit);

			switch (getMode()) {
			case AUTO:
				auto.setImageResource(R.drawable.pressed_autodrive);
				break;
			case NORMAL:
				normal.setImageResource(R.drawable.pressed_normaldrive);
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
		static final int DISPLAY = 3;
		static final int INFORMATION = 4;

		ImageButton navigation;
		ImageButton map;
		ImageButton display;
		ImageButton information;

		ApplicationButton(OnClickListener listener) {
			navigation = (ImageButton)findViewById(R.id.air);
			navigation.setOnClickListener(listener);
			map = (ImageButton)findViewById(R.id.oil);
			map.setOnClickListener(listener);
			display = (ImageButton)findViewById(R.id.p1);
			display.setOnClickListener(listener);
			information = (ImageButton)findViewById(R.id.p2);
			information.setOnClickListener(listener);

			refresh();
		}

		@Override
		void refresh() {
			navigation.setImageResource(R.drawable.app_navi);
			map.setImageResource(R.drawable.app_map);
			display.setImageResource(R.drawable.app_disp);
			information.setImageResource(R.drawable.app_info);

			switch (getMode()) {
			case NAVIGATION:
				navigation.setImageResource(R.drawable.pressed_app_navi);
				break;
			case MAP:
				map.setImageResource(R.drawable.pressed_app_map);
				break;
			case DISPLAY:
				display.setImageResource(R.drawable.pressed_app_disp);
				break;
			case INFORMATION:
				information.setImageResource(R.drawable.pressed_app_info);
				break;
			}
		}
	}

	abstract class Client {
		private static final int TIMEOUT = 3;

		private int sockfd = -1;

		boolean isClosed() {
			if (sockfd < 0)
				return true;
			else
				return false;
		}

		boolean connect(String address, int port) {
			sockfd = SoundManagementNative.socket();
			if (sockfd < 0)
				return false;

			if (SoundManagementNative.connect(sockfd, TIMEOUT, address, port) < 0) {
				close();
				return false;
			}

			return true;
		}

		void close() {
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
	}

	class CommandClient extends Client {
		static final int EXIT = 0;
		static final int GEAR = 1;
		static final int MODE = 2;
		static final int ROUTE = 3;

		int send(int type, int command) {
			if (isClosed())
				return -1;

			sendIntTuple(type, command);

			return recvInt();
		}

		int sendRoute(double[] latlong) {
			if (isClosed())
				return -1;

			sendIntTuple(ROUTE, latlong.length * (Double.SIZE / 8));

			sendDoubleArray(latlong);

			return recvInt();
		}
	}

	class InformationClient extends Client {
		static final int BEACON = 0;
		static final int ERROR = 1;
		static final int CAN = 2;
		static final int MODE = 3;

		static final int MISS_BEACON_LIMIT = 10;

		int[] recv(int response) {
			int[] data = new int[2];

			if (isClosed()) {
				data[0] = -1;
				data[1] = -1;
				return data;
			}

			data[0] = recvInt();
			data[1] = recvInt();

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
	public static boolean bIsServerConnecting = false;
	/**
	 * menu item id
	 */
	private static final int MENU_ID_SETTINGS = 0;
	private static final int MENU_ID_DATA_GATHERING = 1;

	private String getMacAddress() {
		WifiManager wifiManager = (WifiManager)getSystemService(Context.WIFI_SERVICE);
		WifiInfo wifiInfo = wifiManager.getConnectionInfo();
		return wifiInfo.getMacAddress();
	}

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
		gearButton = new GearButton(this);
		driveButton = new DriveButton(this);
		applicationButton = new ApplicationButton(this);

		commandClient = new CommandClient();
		informationClient = new InformationClient();

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
			if (settings.length == 3 && validateIpAddress(settings[0]) &&
			    validatePortNumber(settings[1]) && validatePortNumber(settings[2])) {
				address = settings[0];
				commandPort = Integer.parseInt(settings[1]);
				informationPort = Integer.parseInt(settings[2]);

				if (commandClient.connect(address, commandPort) &&
				    informationClient.connect(address, informationPort)) {
					bIsServerConnecting = true;

					commandClient.send(CommandClient.GEAR, gearButton.getMode());
					commandClient.send(CommandClient.MODE, driveButton.getMode());
				} else {
					bIsServerConnecting = false;

					if (!commandClient.isClosed())
						commandClient.close();
					if (!informationClient.isClosed())
						informationClient.close();
				}
			}
		}

		canDataSender = new CanDataSender("",
						  getMacAddress(),
						  "AutowareTouch",
						  "22",
						  "",
						  "",
						  "",
						  "5558",
						  "127.0.0.1",
						  "5555");

		bIsKnightRiding = true;

		// start recording
		startKnightRiding();

		// start informationReceiver
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

						String text = addressString + "," + commandPortString + "," + informationPortString;
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

							commandClient.send(CommandClient.EXIT, 0);

							commandClient.close();
							informationClient.close();
						}

						address = addressString;
						commandPort = Integer.parseInt(commandPortString);
						informationPort = Integer.parseInt(informationPortString);

						if (commandClient.connect(address, commandPort) &&
						    informationClient.connect(address, informationPort)) {
							bIsServerConnecting = true;

							commandClient.send(CommandClient.GEAR, gearButton.getMode());
							commandClient.send(CommandClient.MODE, driveButton.getMode());
						} else {
							bIsServerConnecting = false;

							if (!commandClient.isClosed())
								commandClient.close();
							if (!informationClient.isClosed())
								informationClient.close();
						}
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
					drawLeftView.drawView(rnd.nextInt(10));
					drawRightView.drawView(rnd.nextInt(10));
					drawCenterView.drawView(rnd.nextInt(10) + 4);
					try {
						Thread.sleep(100);
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

					switch (data[0]) {
					case InformationClient.BEACON:
						missBeacon = 0;
						break;
					case InformationClient.ERROR:
						switch (data[1]) {
						case 0:
							drawLeftView.setColor(Color.RED);
							drawRightView.setColor(Color.RED);
							drawCenterView.setColor(Color.RED);
							break;
						case 1: // should evaluate mode information
							drawLeftView.setColor(Color.BLUE);
							drawRightView.setColor(Color.BLUE);
							drawCenterView.setColor(Color.BLUE);
							break;
						default:
							drawLeftView.setColor(Color.YELLOW);
							drawRightView.setColor(Color.YELLOW);
							drawCenterView.setColor(Color.YELLOW);
						}
						break;
					default:
						if (bIsServerConnecting) {
							if (missBeacon < InformationClient.MISS_BEACON_LIMIT) {
								missBeacon++;
							} else {
								bIsServerConnecting = false;

								commandClient.send(CommandClient.EXIT, 0);

								commandClient.close();
								informationClient.close();

								missBeacon = 0;
							}
						}
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
		if (bIsServerConnecting) {
			bIsServerConnecting = false;

			commandClient.send(CommandClient.EXIT, 0);

			commandClient.close();
			informationClient.close();
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

	@Override
	public void onClick(View v) {
		if (v == gearButton.drive) {
			gearButton.updateMode(GearButton.DRIVE);
			commandClient.send(CommandClient.GEAR, gearButton.getMode());
		} else if (v == gearButton.reverse) {
			gearButton.updateMode(GearButton.REVERSE);
			commandClient.send(CommandClient.GEAR, gearButton.getMode());
		} else if (v == gearButton.brake) {
			gearButton.updateMode(GearButton.BRAKE);
			commandClient.send(CommandClient.GEAR, gearButton.getMode());
		} else if (v == gearButton.neutral) {
			gearButton.updateMode(GearButton.NEUTRAL);
			commandClient.send(CommandClient.GEAR, gearButton.getMode());
		} else if (v == driveButton.auto) {
			driveButton.updateMode(DriveButton.AUTO);
			commandClient.send(CommandClient.MODE, driveButton.getMode());
		} else if (v == driveButton.normal) {
			driveButton.updateMode(DriveButton.NORMAL);
			commandClient.send(CommandClient.MODE, driveButton.getMode());
		} else if (v == driveButton.pursuit) {
			driveButton.updateMode(DriveButton.PURSUIT);
			finish();
		} else if (v == applicationButton.navigation) {
			applicationButton.updateMode(ApplicationButton.NAVIGATION);
			Intent intent = new Intent(Intent.ACTION_MAIN);
			intent.setClassName("com.example.sampleroute",
					    "com.example.sampleroute.MainActivity");
			startActivity(intent);
		} else if (v == applicationButton.map) {
			applicationButton.updateMode(ApplicationButton.MAP);
		} else if (v == applicationButton.display) {
			applicationButton.updateMode(ApplicationButton.DISPLAY);
		} else if (v == applicationButton.information) {
			applicationButton.updateMode(ApplicationButton.INFORMATION);
		}
	}
}
