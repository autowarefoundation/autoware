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
	private static final int COMMAND_EXIT = 0;
	private static final int COMMAND_GEAR = 1;
	private static final int COMMAND_RUN = 2;
	private static final int COMMAND_ROUTE = 3;

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

	GearButton gearButton;
	DriveButton driveButton;
	ApplicationButton applicationButton;

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
		gearButton = new GearButton(this);
		driveButton = new DriveButton(this);
		applicationButton = new ApplicationButton(this);

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
					SoundManagementNative.send(COMMAND_GEAR, gearButton.getMode());
					SoundManagementNative.send(COMMAND_RUN, driveButton.getMode());
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
							SoundManagementNative.send(COMMAND_EXIT, 0);
							SoundManagementNative.close();
						}

						address = addressString;
						port = Integer.parseInt(portString);
						if (SoundManagementNative.connect(address, port) == 0) {
							bIsServerConnecting = true;
							SoundManagementNative.send(COMMAND_GEAR, gearButton.getMode());
							SoundManagementNative.send(COMMAND_RUN, driveButton.getMode());
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
			SoundManagementNative.send(COMMAND_EXIT, 0);
			SoundManagementNative.close();
		}
	}

	private int sendWayPoints(ArrayList<Double> wayPoints) {
		double[] way = new double[wayPoints.size()];

		for (int i = 0; i < wayPoints.size(); i++)
			way[i] = wayPoints.get(i).doubleValue();

		return SoundManagementNative.sendDoubleArray(COMMAND_ROUTE, way);
	}

	@Override
	public void onRestart() {
		if (applicationButton.getMode() == ApplicationButton.MAP) {
			applicationButton.updateMode(ApplicationButton.MAP);
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
			SoundManagementNative.send(COMMAND_GEAR, gearButton.getMode());
		} else if (v == gearButton.reverse) {
			gearButton.updateMode(GearButton.REVERSE);
			SoundManagementNative.send(COMMAND_GEAR, gearButton.getMode());
		} else if (v == gearButton.brake) {
			gearButton.updateMode(GearButton.BRAKE);
			SoundManagementNative.send(COMMAND_GEAR, gearButton.getMode());
		} else if (v == gearButton.neutral) {
			gearButton.updateMode(GearButton.NEUTRAL);
			SoundManagementNative.send(COMMAND_GEAR, gearButton.getMode());
		} else if (v == driveButton.auto) {
			driveButton.updateMode(DriveButton.AUTO);
			SoundManagementNative.send(COMMAND_RUN, driveButton.getMode());
		} else if (v == driveButton.normal) {
			driveButton.updateMode(DriveButton.NORMAL);
			SoundManagementNative.send(COMMAND_RUN, driveButton.getMode());
		} else if (v == driveButton.pursuit) {
			driveButton.updateMode(DriveButton.PURSUIT);
			finish();
		} else if (v == applicationButton.navigation) {
			applicationButton.updateMode(ApplicationButton.NAVIGATION);
		} else if (v == applicationButton.map) {
			applicationButton.updateMode(ApplicationButton.MAP);
			Intent intent = new Intent(Intent.ACTION_MAIN);
			intent.setClassName("com.example.sampleroute",
					    "com.example.sampleroute.MainActivity");
			startActivity(intent);
		} else if (v == applicationButton.display) {
			applicationButton.updateMode(ApplicationButton.DISPLAY);
		} else if (v == applicationButton.information) {
			applicationButton.updateMode(ApplicationButton.INFORMATION);
		}
	}
}
