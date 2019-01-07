/*
 * Copyright 2015 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.candatasender.main;

import java.io.*;
import java.util.*;
import java.text.SimpleDateFormat;
import java.text.ParsePosition;
import android.os.AsyncTask;
import android.app.ProgressDialog;
import android.content.Context;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.util.Log;
import com.example.candatasender.CanGatherData.*;
import com.example.candatasender.sender.ClientInstruction;
import com.example.candatasender.sender.DataType;
import com.example.candatasender.toserver.DataCommunications;
import com.example.useful.ShortAlertDialog;


public class CanGatherDataSend extends AsyncTask<String, String,  Boolean> {
	private Context context;
	private ProgressDialog progressDialog = null;
	private String tableName = null;
	private String terminal;
	private Date   startup;
	final int SEND_SIZE = 200;

	CanGatherDataSend(Context context) {
		this.context = context;

		WifiManager wifiManager = (WifiManager)context.getSystemService(Context.WIFI_SERVICE);
		WifiInfo wifiInfo = wifiManager.getConnectionInfo();
		terminal = wifiInfo.getMacAddress();
	}

	@Override
	protected void onPreExecute() {
		Log.d("MethodCall", "CanGatherDataSend.onPreExecute");

		if (context == null) return;
		progressDialog = new ProgressDialog(context);
		progressDialog.setTitle("データ送信中");
		progressDialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
		progressDialog.show();
	}

	@Override
	protected Boolean doInBackground(String... args) {
		publishProgress("送信準備");

		String src = args[0];
		String dst = args[1];
		String sd  = args[2];
		tableName  = args[3];
		String logDir = sd + "/" + src;
		startup = getStartup(logDir);

		ArrayList<String> jpgList = new ArrayList<String>();
		jpgList.add(dst);
		jpgList.add(sd);

		boolean r = true;
		File log = new File(logDir);
		String[] files = log.list();
		for (String f: files) {
			Log.d("CanGatherDataSend", "send: " + f);
			if (f.startsWith("itsecu_") && f.endsWith(".csv")) {
				publishProgress("送信：" + f);
				r &= sendItsecu(logDir + "/" + f);
			} else if (f.startsWith("sensor_") && f.endsWith(".csv")) {
				publishProgress("送信：" + f);
				r &= sendSensor(logDir + "/" + f);
			} else if (f.startsWith("gps_") && f.endsWith(".csv")) {
				publishProgress("送信：" + f);
				r &= sendGps(logDir + "/" + f);
			} else if (f.startsWith("env_") && f.endsWith(".csv")) {
				publishProgress("送信：" + f);
				r &= sendEnv(logDir + "/" + f);
			} else if (f.endsWith(".jpg")) {
				jpgList.add(src + "/" + f);
			}
		}
		r &= sendImage(jpgList);

		return Boolean.valueOf(r);
	}

	@Override
	protected void onProgressUpdate(String... args) {
		if (progressDialog != null) {
			progressDialog.setMessage(args[0]);
		}
	}

	@Override
	protected void onPostExecute(Boolean result) {
		if (progressDialog != null) {
			progressDialog.dismiss();
			progressDialog = null;
		}
		if (!result && (context != null)) {
			ShortAlertDialog.sad("Error message", "送信中にエラーが発生しました。", context);
		}
	}

	private Date getStartup(String dirname) {
		File dir = new File(dirname);
		String name = dir.getName();
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");
		ParsePosition pos = new ParsePosition(0);
		Date date = sdf.parse(name, pos);
		return date;
	}

	private boolean sendItsecu(String filename) {
		String line = null;
		boolean r = true;
		try {
			FileReader in = new FileReader(filename);
			BufferedReader br = new BufferedReader(in);
			ArrayList<ItsecuData> itsecuList = new ArrayList<ItsecuData>();
			while (true) {
				line = br.readLine();
				if (line == null) break;

				ItsecuData iteecu = ItsecuData.parse(line);
				itsecuList.add(iteecu);
				if (!br.ready() || (SEND_SIZE < itsecuList.size())) {
					r &= sendData(ItsecuData.name, itsecuList);
					itsecuList = new ArrayList<ItsecuData>();
				}
			}
			br.close();
			in.close();
			if (0 != itsecuList.size()) {
				r &= sendData(ItsecuData.name, itsecuList);
			}
		} catch (NumberFormatException e) {
			Log.e("CanGatherDataSend", e.toString());
			Log.e("CanGatherDataSend", line);
			r = false;
		} catch (FileNotFoundException e) {
			Log.e("CanGatherDataSend", e.toString());
			r = false;
		} catch (IOException e) {
			Log.e("CanGatherDataSend", e.toString());
			r = false;
		}
		return r;
	}

	private boolean sendSensor(String filename) {
		final int SEND_SIZE = 30;
		String line = null;
		boolean r = true;
		try {
			FileReader in = new FileReader(filename);
			BufferedReader br = new BufferedReader(in);
			ArrayList<SensorData> sensorList = new ArrayList<SensorData>();
			while (true) {
				line = br.readLine();
				if (line == null) break;

				SensorData sensor = SensorData.parse(line);
				sensorList.add(sensor);
				if (!br.ready() || (SEND_SIZE < sensorList.size())) {
					r &= sendData(SensorData.name, sensorList);
					sensorList = new ArrayList<SensorData>();
				}
			}
			br.close();
			in.close();
			if (sensorList.size() != 0) {
				r &= sendData(SensorData.name, sensorList);
			}
		} catch (NumberFormatException e) {
			Log.e("CanGatherDataSend", e.toString());
			Log.e("CanGatherDataSend", line);
			r = false;
		} catch (FileNotFoundException e) {
			Log.e("CanGatherDataSend", e.toString());
			r = false;
		} catch (IOException e) {
			Log.e("CanGatherDataSend", e.toString());
			r = false;
		}
		return r;
	}

	private boolean sendGps(String filename) {
		String line = null;
		boolean r = true;
		try {
			FileReader in = new FileReader(filename);
			BufferedReader br = new BufferedReader(in);
			ArrayList<GpsData> gpsList = new ArrayList<GpsData>();
			while (true) {
				line = br.readLine();
				if (line == null) break;

				GpsData gps = GpsData.parse(line);
				gpsList.add(gps);
				if (!br.ready() || (SEND_SIZE < gpsList.size())) {
					r &= sendData(GpsData.name, gpsList);
					gpsList = new ArrayList<GpsData>();
				}
			}
			br.close();
			in.close();
			if (gpsList.size() != 0) {
				r &= sendData(GpsData.name, gpsList);
			}
		} catch (NumberFormatException e) {
			Log.e("CanGatherDataSend", e.toString());
			Log.e("CanGatherDataSend", line);
			r = false;
		} catch (FileNotFoundException e) {
			Log.e("CanGatherDataSend", e.toString());
			r = false;
		} catch (IOException e) {
			Log.e("CanGatherDataSend", e.toString());
			r = false;
		}
		return r;
	}

	private boolean sendEnv(String filename) {
		String line = null;
		boolean r = true;
		try {
			FileReader in = new FileReader(filename);
			BufferedReader br = new BufferedReader(in);
			ArrayList<EnvData> envList = new ArrayList<EnvData>();
			while (true) {
				line = br.readLine();
				if (line == null) break;

				EnvData env = EnvData.parse(line);
				envList.add(env);
				if (!br.ready() || (SEND_SIZE < envList.size())) {
					r &= sendData(EnvData.name, envList);
					envList = new ArrayList<EnvData>();
				}
			}
			br.close();
			in.close();
			if (envList.size() != 0) {
				r &= sendData(EnvData.name, envList);
			}
		} catch (NumberFormatException e) {
			Log.e("CanGatherDataSend", e.toString());
			Log.e("CanGatherDataSend", line);
			r = false;
		} catch (FileNotFoundException e) {
			Log.e("CanGatherDataSend", e.toString());
			r = false;
		} catch (IOException e) {
			Log.e("CanGatherDataSend", e.toString());
			r = false;
		}
		return r;
	}

	private boolean sendImage(ArrayList<String> jpgList) {
		final int SEND_SIZE = 180;
		boolean r = true;
		ArrayList<ImageData> imageList = new ArrayList<ImageData>();
		for (int i=2; i<jpgList.size(); ++i) {
			ImageData image = ImageData.parse(jpgList.get(i));
			imageList.add(image);
			if ((SEND_SIZE < imageList.size())) {
				r &= sendData(ImageData.name, imageList);
				imageList  = new ArrayList<ImageData>();
			}
		}
		if (imageList.size() != 0) {
			r &= sendData(ImageData.name, imageList);
		}

		RawDataScp rds = new RawDataScp(context, progressDialog);
		progressDialog = null;
		rds.execute(jpgList.toArray(new String[jpgList.size()]));

		return r;
	}

    private boolean sendData(String name, Object obj) {
		Map<String, Object> data = new HashMap<String, Object>();
		data.put("INST_NUM",   ClientInstruction.INST_INSERT_CG_DATA.getNum());
		data.put("TABLE_NAME", tableName);
		data.put("STARTUP",    startup);
		data.put("DATA_NAME",  name);
		data.put("DATA_OBJ",   obj);
		data.put("TERMINAL",   terminal);

		Map<String, Object> send = new HashMap<String, Object>();
		send.put("TYPE", DataType.DT_INST.getType());
		send.put("DATA", data);

		Map<String, Object> response = new HashMap<String, Object>();
		DataCommunications.getInstance().addSendDataNoWait(send, response);

		return true; /* TODO  */
	}
}
