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
import com.example.candatasender.CarLinkData.*;
import com.example.candatasender.sender.ClientInstruction;
import com.example.candatasender.sender.DataType;
import com.example.candatasender.toserver.DataCommunications;
import com.example.useful.ShortAlertDialog;


public class CarLinkDataSend extends AsyncTask<String, String,  Boolean> {
	private Context context;
	private ProgressDialog progressDialog = null;
	private String tableName = null;
	private Date   startup;
	final int SEND_SIZE = 200;

	CarLinkDataSend(Context context) {
		this.context = context;
	}

	@Override
	protected void onPreExecute() {
		Log.d("MethodCall", "CarLinkDataSend.onPreExecute");

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
		tableName  = args[4];
		CanData.UsbBt usbbt;
		if (args[3].equalsIgnoreCase("usb")) {
			usbbt = CanData.UsbBt.USB;
		} else if (args[3].equalsIgnoreCase("bt")) {
			usbbt = CanData.UsbBt.BT;
		} else {
			return Boolean.FALSE;
		}
		String logDir = sd + "/" + src;
		startup = getStartup(logDir);

		ArrayList<String> videoList = new ArrayList<String>();
		videoList.add(dst);
		videoList.add(sd);

		boolean r = true;
		File log = new File(logDir);
		String[] files = log.list();
		for (String f: files) {
			Log.d("CarLinkDataSend", "send: " + f);
			if (f.startsWith("cl_can_") && f.endsWith(".csv")) {
				publishProgress("送信：" + f);
				r &= sendCan(logDir + "/" + f, usbbt);
			} else if (f.startsWith("cl_acc_") && f.endsWith(".csv")) {
				publishProgress("送信：" + f);
				r &= sendAcc(logDir + "/" + f);
			} else if (f.startsWith("cl_pose_") && f.endsWith(".csv")) {
				publishProgress("送信：" + f);
				r &= sendPose(logDir + "/" + f);
			} else if (f.startsWith("cl_loc_") && f.endsWith(".csv")) {
				publishProgress("送信：" + f);
				r &= sendLoc(logDir + "/" + f);
			} else if (f.startsWith("cl_video_") && f.endsWith(".mp4")) {
				videoList.add(src + "/" + f);
			}
		}
		r &= sendVideo(videoList);

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
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-M-d-H-m-s");
		ParsePosition pos = new ParsePosition(0);
		Date date = sdf.parse(name, pos);
		return date;
	}

	private boolean sendCan(String filename, CanData.UsbBt usbbt) {
		boolean r = true;
		String line = null;
		try {
			FileReader in = new FileReader(filename);
			BufferedReader br = new BufferedReader(in);
			ArrayList<CanData> canList = new ArrayList<CanData>();
			br.readLine(); /* 1行目空読み */
			while (true) {
				line = br.readLine();
				if (line == null) break;

				CanData can = CanData.parse(line, usbbt);
				if (can.type != CanData.CanDataType.unknown) {
					canList.add(can);
				}
				if (!br.ready() || (SEND_SIZE < canList.size())) {
					r &= sendData(CanData.name, canList);
					canList = new ArrayList<CanData>();
				}
			}
			br.close();
			in.close();
			if (canList.size() != 0) {
				r &= sendData(CanData.name, canList);
			}
		} catch (NumberFormatException e) {
			Log.e("CarLinkDataSend", e.toString());
			Log.e("CarLinkDataSend", line);
			r = false;
		} catch (FileNotFoundException e) {
			Log.e("CarLinkDataSend", e.toString());
			r = false;
		} catch (IOException e) {
			Log.e("CarLinkDataSend", e.toString());
			r = false;
		}
		return r;
	}

	private boolean sendAcc(String filename) {
		String line = null;
		boolean r = true;
		try {
			FileReader in = new FileReader(filename);
			BufferedReader br = new BufferedReader(in);
			ArrayList<AccData> accList = new ArrayList<AccData>();
			br.readLine(); /* 1行目空読み */
			while (true) {
				line = br.readLine();
				if (line == null) break;

				AccData acc = AccData.parse(line);
				accList.add(acc);
				if (!br.ready() || (SEND_SIZE < accList.size())) {
					r &= sendData(AccData.name, accList);
					accList = new ArrayList<AccData>();
				}
			}
			br.close();
			in.close();
			if (0 != accList.size()) {
				r &= sendData(AccData.name, accList);
			}
		} catch (NumberFormatException e) {
			Log.e("CarLinkDataSend", e.toString());
			Log.e("CarLinkDataSend", line);
			r = false;
		} catch (FileNotFoundException e) {
			Log.e("CarLinkDataSend", e.toString());
			r = false;
		} catch (IOException e) {
			Log.e("CarLinkDataSend", e.toString());
			r = false;
		}
		return r;
	}

	private boolean sendPose(String filename) {
		String line = null;
		boolean r = true;
		try {
			FileReader in = new FileReader(filename);
			BufferedReader br = new BufferedReader(in);
			ArrayList<PoseData> poseList = new ArrayList<PoseData>();
			br.readLine(); /* 1行目空読み */
			while (true) {
				line = br.readLine();
				if (line == null) break;

				PoseData pose = PoseData.parse(line);
				poseList.add(pose);
				if (!br.ready() || (SEND_SIZE < poseList.size())) {
					r &= sendData(PoseData.name, poseList);
					poseList = new ArrayList<PoseData>();
				}
			}
			br.close();
			in.close();
			if (poseList.size() != 0) {
				r &= sendData(PoseData.name, poseList);
			}
		} catch (NumberFormatException e) {
			Log.e("CarLinkDataSend", e.toString());
			Log.e("CarLinkDataSend", line);
			r = false;
		} catch (FileNotFoundException e) {
			Log.e("CarLinkDataSend", e.toString());
			r = false;
		} catch (IOException e) {
			Log.e("CarLinkDataSend", e.toString());
			r = false;
		}
		return r;
	}

	private boolean sendLoc(String filename) {
		String line = null;
		boolean r = true;
		try {
			FileReader in = new FileReader(filename);
			BufferedReader br = new BufferedReader(in);
			ArrayList<LocData> locList = new ArrayList<LocData>();
			br.readLine(); /* 1行目空読み */
			while (true) {
				line = br.readLine();
				if (line == null) break;

				LocData loc = LocData.parse(line);
				locList.add(loc);
				if (!br.ready() || (SEND_SIZE < locList.size())) {
					r &= sendData(LocData.name, locList);
					locList = new ArrayList<LocData>();
				}
			}
			br.close();
			in.close();
			if (locList.size() != 0) {
				r &= sendData(LocData.name, locList);
			}
		} catch (NumberFormatException e) {
			Log.e("CarLinkDataSend", e.toString());
			Log.e("CarLinkDataSend", line);
			r = false;
		} catch (FileNotFoundException e) {
			Log.e("CarLinkDataSend", e.toString());
			r = false;
		} catch (IOException e) {
			Log.e("CarLinkDataSend", e.toString());
			r = false;
		}
		return r;
	}

	private boolean sendVideo(ArrayList<String> videoList) {
		File videoFile = new File(videoList.get(2));
		String videoSplit = "${HOME}/bin/nohup-video_split.sh " +
				"\"" + videoList.get(0) + "/" + videoList.get(2) + "\" " +
				tableName + " " +
				getBaseTimestamp(videoFile.getName()) + " " +
				dateToSqlTimestamp(startup) + " " +
				getTerminal() + " " +
				videoFile.getParent();
		RawDataScp rds = new RawDataScp(context, progressDialog, null, videoSplit);
		progressDialog = null;
		rds.execute(videoList.toArray(new String[videoList.size()]));

		return true;
	}

	private String getBaseTimestamp(String filename) {
		int start = filename.indexOf("(") + 1;
		int end   = filename.indexOf(")", start);
		if (start <= 0) {
			start = filename.indexOf("_tstamp_") + 8;
			end = filename.indexOf(".mp4", start);
		}
		Log.d("CarLinkDataSend", "getBaseTimestamp filename=" + filename + ", start=" + start + ", end=" + end);
		// TODO: range check!
		return filename.substring(start, end);
	}

	private String dateToSqlTimestamp(Date date) {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
		String str = sdf.format(date);
		return "\"" + str + "\"";
	}

	private boolean sendData(String name, Object obj) {
		Map<String, Object> data = new HashMap<String, Object>();
		data.put("INST_NUM",   ClientInstruction.INST_INSERT_CL_DATA.getNum());
		data.put("TABLE_NAME", tableName);
		data.put("STARTUP",    startup);
		data.put("DATA_NAME",  name);
		data.put("DATA_OBJ",   obj);
		data.put("TERMINAL",   getTerminal());

		Map<String, Object> send = new HashMap<String, Object>();
		send.put("TYPE", DataType.DT_INST.getType());
		send.put("DATA", data);

		Map<String, Object> response = new HashMap<String, Object>();
		DataCommunications.getInstance().addSendDataNoWait(send, response);

		return true; /* TODO  */
	}

	private String getTerminal() {
		WifiManager wifiManager = (WifiManager)context.getSystemService(Context.WIFI_SERVICE);
		WifiInfo wifiInfo = wifiManager.getConnectionInfo();
		return wifiInfo.getMacAddress();
	}
}
