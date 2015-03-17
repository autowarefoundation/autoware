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

package com.example.candatasender.service;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.text.ParsePosition;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;

import com.example.candatasender.R;
import com.example.candatasender.CanGatherData.EnvData;
import com.example.candatasender.CanGatherData.GpsData;
import com.example.candatasender.CanGatherData.ItsecuData;
import com.example.candatasender.CanGatherData.SensorData;
import com.example.candatasender.CarLinkData.AccData;
import com.example.candatasender.CarLinkData.CanData;
import com.example.candatasender.CarLinkData.LocData;
import com.example.candatasender.CarLinkData.PoseData;
import com.example.candatasender.sender.ClientInstruction;
import com.example.candatasender.sender.DataType;
import com.example.candatasender.portforward.PortforwardData;
import com.example.candatasender.portforward.SshTunnel;
import com.example.candatasender.toserver.DataCommunications;
import com.example.candatasender.toserver.ToServer;
import com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverService;
import com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverServiceCallbackListener;

import android.app.Notification;
import android.app.NotificationManager;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.net.LocalServerSocket;
import android.net.LocalSocket;
import android.net.LocalSocketAddress;
import android.os.Environment;
import android.os.IBinder;
import android.os.RemoteException;
import android.util.Log;

/**
 * A service which read from UNIX domain socket and write to RDBMS.
 */
public class CanDataAutoSend extends Service {
	/**
	 * the UNIX domain socket name.
	 */
	public final String SERVER_SOCKET_NAME = "com.metaprotocol.android.carlink_sample_receiver_service.server";
	/**
	 * the maximum number of inserting records.
	 */
	private final int CAN_INSERT_MAX = 256;
	private final int ACC_INSERT_MAX = 32;
	/**
	 * the accept thread.
	 */
	private AcceptThread acceptThread;
	/**
	 * the MAC addres.
	 */
	private String terminal;
	/**
	 * the table name of RDBMS.
	 */
	private String tableName;
	/**
	 * the flag to enable use of extra port forward data.
	 */
	private Boolean isPortforwardData;
	/**
	 * the callback listener of CarLink.
	 */
	ICarLinkDataReceiverServiceCallbackListener viewUpdateEventListener;
	/**
	 * debug output (see dbgLog())
	 */
	private static boolean isDebugOut = false;
	/**
	 * do dummy test for CanGather
	 */
	private boolean isDummyTest = false;
	
	@Override
	public void onCreate() {
		Log.d("CanDataAutoSend", "onCreate()");
		super.onCreate();
	}

	@Override
	public int onStartCommand(Intent intent, int flags, int start_id) {
		Log.d("CanDataAutoSend", "onStartCommand()");
		tableName = intent.getStringExtra("table");
		terminal = intent.getStringExtra("terminal");
		dbgLog("table=" + tableName + ",terminal=" + terminal);
		isPortforwardData = intent.getBooleanExtra("pfd", false);
		if (isPortforwardData) {
			if (!PortforwardData.GetConnectedSetting().equals("")) {
				ToServer.getInstance().disConnect();
				SshTunnel.getInstance().disConnect();
				PortforwardData.SetConnectedSetting("");
			}

			PortforwardData pfd = new PortforwardData(intent.getStringExtra("str"),
								  intent.getStringExtra("sp"),
								  intent.getStringExtra("sh"),
								  intent.getStringExtra("su"),
								  intent.getStringExtra("spss"),
								  intent.getStringExtra("lp"),
								  intent.getStringExtra("fh"),
								  intent.getStringExtra("fp"));
			if (SshTunnel.getInstance().connect(pfd)) {
				String host = "127.0.0.1";
				int port = Integer.valueOf(pfd.getLocalPort()).intValue();
				ToServer.getInstance().setServerData(host, port);

				Map<String, String> rd = ToServer.getInstance().connect();
				if (((String)rd.get("TYPE")).equals(DataType.DT_CONNECTION_SUCCESS.getType()))
					PortforwardData.SetConnectedSetting(pfd.getString());
				else {
					ToServer.getInstance().disConnect();
					SshTunnel.getInstance().disConnect();
					return START_STICKY;
				}
			}
		}
		try {
			acceptThread = new AcceptThread(SERVER_SOCKET_NAME);
			acceptThread.start();
			if (isDummyTest) {
				Thread dummyThread = new DummySendThread();
				dummyThread.start();
			}
		} catch (IOException e) {
			e.printStackTrace();
			acceptThread = null;
		}
		Notification notification = new Notification.Builder(this)
        	.setContentTitle("CanDataAutoSend")
        	.setSmallIcon(R.drawable.ic_launcher)
        	.build();
		NotificationManager manager = (NotificationManager)getSystemService(Context.NOTIFICATION_SERVICE);
		manager.notify(1, notification);
		startForeground(1, notification);
		dbgLog("onStartCommand() success");

		return START_STICKY;
	}

	@Override
	public void onDestroy() {
		Log.d("CanDataAutoSend", "onDestroy()");
		dbgLog("onDestroy() called");
		if (acceptThread != null) {
			acceptThread.close();
			acceptThread = null;
		}
		if (isPortforwardData && !PortforwardData.GetConnectedSetting().equals("")) {
			ToServer.getInstance().disConnect();
			SshTunnel.getInstance().disConnect();
			PortforwardData.SetConnectedSetting("");
		}
		super.onDestroy();
	}
	
	@Override
	public IBinder onBind(Intent arg0) {
		return (IBinder)dataRcvServiceIfImpl;
	}

	@Override
	public boolean onUnbind(Intent arg0) {
		viewUpdateEventListener = null;
		return true;
	}

	ICarLinkDataReceiverService dataRcvServiceIfImpl = new ICarLinkDataReceiverService.Stub() {
		@Override
		public int setEventListener(ICarLinkDataReceiverServiceCallbackListener listener)
				throws RemoteException {
			viewUpdateEventListener = listener;
			return 0;
		}
		
		@Override
		public int close() throws RemoteException {
			Log.d("CanDataAutoSend", "close() called.");
			acceptThread.close();
			return 0;
		}
	};

	/**
	 * An accept thread.
	 */
	class AcceptThread extends Thread {
		/**
		 * the UNIX domain socket.
		 */
		private LocalServerSocket localSrvSocket;
		/**
		 * the exit flag.
		 */
		private boolean exitFlag;
		/**
		 * the list of data receiving/sending threads.
		 */
		private ArrayList<DataReceiveThread> dataRcvThreadList;

		/**
		 * Constructor.
		 * @param sockName the UNIX domain socket name.
		 * @param tableName the table name of RDBMS.
		 * @throws IOException
		 */
		public AcceptThread(String sockName) throws IOException {
			Log.d("CanDataAutoSend", "AcceptThread tableName:" + tableName);
			dataRcvThreadList = new ArrayList<DataReceiveThread>();
			localSrvSocket = new LocalServerSocket(sockName);
			exitFlag = false;
		}
		
		public void close() {
			exitFlag = true;
			for(DataReceiveThread rcvThread : dataRcvThreadList) {
				rcvThread.close();
			}
		}
		
		@Override
		public void run() {
			Log.d("CanDataAutoSend", "AcceptThread run.");
    		while((!Thread.currentThread().isInterrupted()) && (!exitFlag)) {
        		try {
        			LocalSocket localSocket = localSrvSocket.accept();
        			Log.d("CanDataAutoSend", "AcceptThread accepted.");
        			dbgLog("AcceptThread accepted");
        			DataReceiveThread dataRcvThread = new DataReceiveThread(localSocket);
        			dataRcvThreadList.add(dataRcvThread);
        			dataRcvThread.start();
        		} catch (IOException e) {
    				e.printStackTrace();
    				break;
    			}
			}

    		try {
				localSrvSocket.close();
    			Log.d("CanDataAutoSend", "AcceptThread closed localSrvSocket.");
			} catch (IOException e) {
				e.printStackTrace();
			} finally {
    			Log.d("CanDataAutoSend", "AcceptThread exited.");
    		}
		}

		/**
		 * A data sending/receiving thread.
		 */
		class DataReceiveThread extends Thread {
			private final int FILENAME_LEN_MAX = 512;
			private LocalSocket localSocket;
			private boolean exitFlag;

			/**
			 * Constructor.
			 * @param localSocket the accepted UNIX domain socket.
			 * @param tableName the table name of RDBMS.
			 */
			public DataReceiveThread(LocalSocket localSocket) {
				this.localSocket = localSocket;
				exitFlag = false;
			}
			
			public void close() {
				exitFlag = true;
				interrupt();
			}
			
			/**
			 * Returns the "startup" date.
			 * @param fileName the file name of CSV.
			 * @return the "Startup" date.
			 */
			private Date getStartup(String fileName) {
				int fIndex = fileName.indexOf("20"); // 20XX
				int tIndex = fileName.indexOf(".");
				if (fIndex < 0) {
					return null;
				}
				String dateName = (tIndex < 0) ? fileName.substring(fIndex):fileName.substring(fIndex, tIndex);
				SimpleDateFormat sdf = new SimpleDateFormat("yyyy-M-d-H-m-s");
                ParsePosition pos = new ParsePosition(0);
                Date date = sdf.parse(dateName, pos);
                return date;
			}
			
			@Override
			public void run() {
    			String dataName = "*unset*";
				Log.d("CanDataAutoSend", "DataReceiveThread run.");
    			dbgLog("DataReceiveThread run:" + this.getName());
	    		try {
	    			// at first, receive the file name of CSV or MPEG.
	    			BufferedInputStream bIn = new BufferedInputStream(localSocket.getInputStream());
	    			byte[] headerBytes = new byte[FILENAME_LEN_MAX];
	    			int i = 0;
	    			while((!Thread.currentThread().isInterrupted()) && (!exitFlag)) {
	    				int val = bIn.read();
	    				if (val == -1) {
	    					exitFlag = true;
	    					break;
	    				} else if (val == 0x0d) {  // terminator
	    					break;
	    				}
	    				headerBytes[i++] = (byte)val;
    	    			//dbgLog("  i=" + i + ", val=" + val + ":" + this.getName());
	    				if (i >= headerBytes.length) {
	    	    			dbgLog("DataReceiveThread filename too long:" + i);
	    					exitFlag = true;
	    					break;
	    				}
	    			}

	    			if (!exitFlag) {
	    				dataName = new String(headerBytes, 0, i, "UTF-8");  // file name
	    				dbgLog("DataReceiveThread filename=" + dataName);
	    				// CarLink
	    				if (dataName.startsWith("cl_can_") && dataName.endsWith(".csv")) {
	    					sendCan(bIn, dataName);
	    				} else if (dataName.startsWith("cl_acc_") && dataName.endsWith(".csv")) {
	    					sendAcc(bIn, dataName);
	    				} else if (dataName.startsWith("cl_pose_") && dataName.endsWith(".csv")) {
	    					sendPose(bIn, dataName);
	    				} else if (dataName.startsWith("cl_loc_") && dataName.endsWith(".csv")) {
	    					sendLoc(bIn, dataName);
	    				}
	    				// CanGather
	    				else if (dataName.startsWith("itsecu_") && dataName.endsWith(".csv")) {
	    					sendItsecu(bIn, dataName);
	    				} else if (dataName.startsWith("sensor_") && dataName.endsWith(".csv")) {
	    					sendSensor(bIn, dataName);
	    				} else if (dataName.startsWith("gps_") && dataName.endsWith(".csv")) {
	    					sendGps(bIn, dataName);
	    				} else if (dataName.startsWith("env_") && dataName.endsWith(".csv")) {
	    					sendEnv(bIn, dataName);
	    				} else {
	    					recvOther(bIn, dataName); // TODO: mpeg, jpeg
	    				}
	    			}
	    			// TODO: I think we can collect send* methods to one method.
    			
					bIn.close();
	    			localSocket.close();
				} catch (IOException e) {
					e.printStackTrace();
				} finally {
	    			Log.d("CanDataAutoSend", "DataReceiveThread exited / dataName:" + dataName);
    				dbgLog("DataReceiveThread exited / filename=" + dataName);
	    			dataRcvThreadList.remove(this);
	    		}
			}
			
			/**
			 * receive can data and send to RDBMS.
			 * @param bIn BufferedInputStream of local socket.
			 * @param dataName the data name (file name).
			 * @return true if success.
			 * @throws IOException
			 */
			private boolean sendCan(BufferedInputStream bIn, String dataName)
					throws IOException {
				boolean ret = true;
				int total = 0;
				Date startup = getStartup(dataName);
				localSocket.getOutputStream().write(0x0d); // send ACK
				Log.d("CanDataAutoSend", "DataReceiveThread sent ACK / dataName:" + dataName);
				dbgLog("DataReceiveThread send ACK / filename=" + dataName + " / startup=" + startup);
				
				BufferedReader bReader = new BufferedReader(new InputStreamReader(bIn));
				String header = bReader.readLine();
				CanData.UsbBt usbbt = CanData.parseUsbBt(header);
				ArrayList<CanData> list = new ArrayList<CanData>();
				while ((!Thread.currentThread().isInterrupted()) && bReader.ready()) {
					String line = bReader.readLine();
					if (line == null || line.length() == 0) {
						break;
					}
					CanData data = CanData.parse(line, usbbt);
					if (data != null && data.type != CanData.CanDataType.unknown) {
						list.add(data);
					}
					if (!bReader.ready() || list.size() >= CAN_INSERT_MAX) {
						ret &= sendData(CanData.name, startup, list);
						total += list.size();
						list = new ArrayList<CanData>();
					}
				}
				if (list.size() > 0) {
					ret &= sendData(AccData.name, startup, list);
					total += list.size();
				}

				Log.d("CanDataAutoSend", "DataReceiveThread total:" + total + " / dataName:" + dataName);
				dbgLog("DataReceiveThread can total=" + total + " / filename=" + dataName + " / startup=" + startup);
				return ret;
			}
			
			/**
			 * receive acc data and send to RDBMS.
			 * @param bIn BufferedInputStream of local socket.
			 * @param dataName the data name (file name).
			 * @return true if success.
			 * @throws IOException
			 */
			private boolean sendAcc(BufferedInputStream bIn, String dataName) throws IOException {
				boolean ret = true;
				int total = 0;
				Date startup = getStartup(dataName);
				localSocket.getOutputStream().write(0x0d); // send ACK
				Log.d("CanDataAutoSend", "DataReceiveThread sent ACK / dataName:" + dataName);
				dbgLog("DataReceiveThread send ACK / filename=" + dataName + " / startup=" + startup);
				
				BufferedReader bReader = new BufferedReader(new InputStreamReader(bIn));
				bReader.readLine(); // skip header
				ArrayList<AccData> list = new ArrayList<AccData>();
				while ((!Thread.currentThread().isInterrupted()) && bReader.ready()) {
					String line = bReader.readLine();
					if (line == null || line.length() == 0) {
						break;
					}
					AccData data = AccData.parse(line);
					list.add(data);
					if (!bReader.ready() || list.size() >= ACC_INSERT_MAX) {
						ret &= sendData(AccData.name, startup, list);
						total += list.size();
						list = new ArrayList<AccData>();
					}
				}
				if (list.size() > 0) {
					ret &= sendData(AccData.name, startup, list);
					total += list.size();
				}

				Log.d("CanDataAutoSend", "DataReceiveThread total:" + total + " / dataName:" + dataName);
				dbgLog("DataReceiveThread acc total=" + total + " / filename=" + dataName + " / startup=" + startup);
				return ret;
			}
			
			/**
			 * receive pose data and send to RDBMS.
			 * @param bIn BufferedInputStream of local socket.
			 * @param dataName the data name (file name).
			 * @return true if success.
			 * @throws IOException
			 */
			private boolean sendPose(BufferedInputStream bIn, String dataName) throws IOException {
				boolean ret = true;
				int total = 0;
				Date startup = getStartup(dataName);
				localSocket.getOutputStream().write(0x0d); // send ACK
				Log.d("CanDataAutoSend", "DataReceiveThread sent ACK / dataName:" + dataName);
				dbgLog("DataReceiveThread send ACK / filename=" + dataName + " / startup=" + startup);
				
				BufferedReader bReader = new BufferedReader(new InputStreamReader(bIn));
				bReader.readLine(); // skip header
				ArrayList<PoseData> list = new ArrayList<PoseData>();
				while ((!Thread.currentThread().isInterrupted()) && bReader.ready()) {
					String line = bReader.readLine();
					if (line == null || line.length() == 0) {
						break;
					}
					PoseData data = PoseData.parse(line);
					list.add(data);
					ret &= sendData(PoseData.name, startup, list);
					total++;
					list = new ArrayList<PoseData>();
				}

				Log.d("CanDataAutoSend", "DataReceiveThread total:" + total + " / dataName:" + dataName);
				dbgLog("DataReceiveThread pose total=" + total + " / filename=" + dataName + " / startup=" + startup);
				return ret;
			}
			
			/**
			 * receive loc data and send to RDBMS.
			 * @param bIn BufferedInputStream of local socket.
			 * @param dataName the data name (file name).
			 * @return true if success.
			 * @throws IOException
			 */
			private boolean sendLoc(BufferedInputStream bIn, String dataName) throws IOException {
				boolean ret = true;
				int total = 0;
				Date startup = getStartup(dataName);
				localSocket.getOutputStream().write(0x0d); // send ACK
				Log.d("CanDataAutoSend", "DataReceiveThread sent ACK / dataName:" + dataName);
				dbgLog("DataReceiveThread send ACK / filename=" + dataName + " / startup=" + startup);
				
				BufferedReader bReader = new BufferedReader(new InputStreamReader(bIn));
				bReader.readLine(); // skip header
				ArrayList<LocData> list = new ArrayList<LocData>();
				while ((!Thread.currentThread().isInterrupted()) && bReader.ready()) {
					String line = bReader.readLine();
					if (line == null || line.length() == 0) {
						break;
					}
					LocData data = LocData.parse(line);
					list.add(data);
					ret &= sendData(LocData.name, startup, list);
					total++;
					list = new ArrayList<LocData>();
				}

				Log.d("CanDataAutoSend", "DataReceiveThread total:" + total + " / dataName:" + dataName);
				dbgLog("DataReceiveThread loc total=" + total + " / filename=" + dataName + " / startup=" + startup);
				return ret;
			}
			
			/**
			 * receive itsecu data and send to RDBMS.
			 * @param bIn BufferedInputStream of local socket.
			 * @param dataName the data name (file name).
			 * @return true if success.
			 * @throws IOException
			 */
			private boolean sendItsecu(BufferedInputStream bIn, String dataName) throws IOException {
				boolean ret = true;
				int total = 0;
				Date startup = getStartup(dataName);
				Log.d("CanDataAutoSend", "DataReceiveThread dataName:" + dataName);
				dbgLog("DataReceiveThread / filename=" + dataName + " / startup=" + startup);
				
				BufferedReader bReader = new BufferedReader(new InputStreamReader(bIn));
				ArrayList<ItsecuData> list = new ArrayList<ItsecuData>();
				while ((!Thread.currentThread().isInterrupted()) && bReader.ready()) {
					String line = bReader.readLine();
					if (line == null || line.length() == 0) {
						break;
					}
					ItsecuData data = ItsecuData.parse(line);
					list.add(data);
					ret &= sendData(ItsecuData.name, startup, list);
					total++;
					list = new ArrayList<ItsecuData>();
				}

				Log.d("CanDataAutoSend", "DataReceiveThread total:" + total + " / dataName:" + dataName);
				dbgLog("DataReceiveThread itsecu total=" + total + " / filename=" + dataName + " / startup=" + startup);
				return ret;
			}

			/**
			 * receive sensor data and send to RDBMS.
			 * @param bIn BufferedInputStream of local socket.
			 * @param dataName the data name (file name).
			 * @return true if success.
			 * @throws IOException
			 */
			private boolean sendSensor(BufferedInputStream bIn, String dataName) throws IOException {
				boolean ret = true;
				int total = 0;
				Date startup = getStartup(dataName);
				Log.d("CanDataAutoSend", "DataReceiveThread dataName:" + dataName);
				dbgLog("DataReceiveThread / filename=" + dataName + " / startup=" + startup);
				
				BufferedReader bReader = new BufferedReader(new InputStreamReader(bIn));
				ArrayList<SensorData> list = new ArrayList<SensorData>();
				while ((!Thread.currentThread().isInterrupted()) && bReader.ready()) {
					String line = bReader.readLine();
					if (line == null || line.length() == 0) {
						break;
					}
					SensorData data = SensorData.parse(line);
					list.add(data);
					ret &= sendData(SensorData.name, startup, list);
					total++;
					list = new ArrayList<SensorData>();
				}

				Log.d("CanDataAutoSend", "DataReceiveThread total:" + total + " / dataName:" + dataName);
				dbgLog("DataReceiveThread sensor total=" + total + " / filename=" + dataName + " / startup=" + startup);
				return ret;
			}

			/**
			 * receive gps data and send to RDBMS.
			 * @param bIn BufferedInputStream of local socket.
			 * @param dataName the data name (file name).
			 * @return true if success.
			 * @throws IOException
			 */
			private boolean sendGps(BufferedInputStream bIn, String dataName) throws IOException {
				boolean ret = true;
				int total = 0;
				Date startup = getStartup(dataName);
				Log.d("CanDataAutoSend", "DataReceiveThread dataName:" + dataName);
				dbgLog("DataReceiveThread / filename=" + dataName + " / startup=" + startup);
				
				BufferedReader bReader = new BufferedReader(new InputStreamReader(bIn));
				ArrayList<GpsData> list = new ArrayList<GpsData>();
				while ((!Thread.currentThread().isInterrupted()) && bReader.ready()) {
					String line = bReader.readLine();
					if (line == null || line.length() == 0) {
						break;
					}
					GpsData data = GpsData.parse(line);
					list.add(data);
					ret &= sendData(GpsData.name, startup, list);
					total++;
					list = new ArrayList<GpsData>();
				}

				Log.d("CanDataAutoSend", "DataReceiveThread total:" + total + " / dataName:" + dataName);
				dbgLog("DataReceiveThread gps total=" + total + " / filename=" + dataName + " / startup=" + startup);
				return ret;
			}

			/**
			 * receive env data and send to RDBMS.
			 * @param bIn BufferedInputStream of local socket.
			 * @param dataName the data name (file name).
			 * @return true if success.
			 * @throws IOException
			 */
			private boolean sendEnv(BufferedInputStream bIn, String dataName) throws IOException {
				boolean ret = true;
				int total = 0;
				Date startup = getStartup(dataName);
				Log.d("CanDataAutoSend", "DataReceiveThread dataName:" + dataName);
				dbgLog("DataReceiveThread / filename=" + dataName + " / startup=" + startup);
				
				BufferedReader bReader = new BufferedReader(new InputStreamReader(bIn));
				ArrayList<EnvData> list = new ArrayList<EnvData>();
				while ((!Thread.currentThread().isInterrupted()) && bReader.ready()) {
					String line = bReader.readLine();
					if (line == null || line.length() == 0) {
						break;
					}
					EnvData data = EnvData.parse(line);
					list.add(data);
					ret &= sendData(EnvData.name, startup, list);
					total++;
					list = new ArrayList<EnvData>();
				}

				Log.d("CanDataAutoSend", "DataReceiveThread total:" + total + " / dataName:" + dataName);
				dbgLog("DataReceiveThread env total=" + total + " / filename=" + dataName + " / startup=" + startup);
				return ret;
			}

			/**
			 * receive other data and do nothing...
			 * @param bIn BufferedInputStream of local socket.
			 * @param dataName the data name (file name).
			 * @return true if success.
			 * @throws IOException
			 */
			private boolean recvOther(BufferedInputStream bIn, String dataName) throws IOException {
				boolean ret = true;
				byte[] bytes = new byte[4096];
				localSocket.getOutputStream().write(0x0d); // send ACK
				Log.d("CanDataAutoSend", "DataReceiveThread sent ACK, do nothing / dataName:" + dataName);
				dbgLog("DataReceiveThread send ACK, do nothing / filename=" + dataName);
				while (!Thread.currentThread().isInterrupted() && bIn.read(bytes) > 0) {
					// TODO: do nothing...
				}
				return ret;
			}
			
			/**
			 * Send CarLink data to RDBMS.
			 * @param name the data name.
			 * @param startup the "startup" date.
			 * @param obj the list of data.
			 * @return true if success.
			 */
			private boolean sendData(String name, Date startup, Object obj) {
				HashMap<String, Object> data = new HashMap<String, Object>();
				if (name.equals(CanData.name) || name.equals(AccData.name) ||
						name.equals(PoseData.name) || name.equals(LocData.name)) {
					data.put("INST_NUM",	ClientInstruction.INST_INSERT_CL_DATA.getNum());
				} else {
					data.put("INST_NUM",	ClientInstruction.INST_INSERT_CG_DATA.getNum());
				}
                data.put("TABLE_NAME",	tableName);
                data.put("STARTUP",		startup);
                data.put("DATA_NAME",	name);
                data.put("DATA_OBJ",	obj);
                data.put("TERMINAL",	terminal);
                
                HashMap<String, Object> send = new HashMap<String, Object>();
                send.put("TYPE",		DataType.DT_INST.getType());
                send.put("DATA",		data);

                HashMap<String, Object> response = new HashMap<String, Object>();
                DataCommunications.getInstance().addSendDataNoWait(send, response);
                
                return true;
			}
		}
	}
	
	/**
	 * outputs the debug message to file.
	 * @param message the debug message.
	 */
	public static void dbgLog(String message) {
		if (! isDebugOut) {
			return;
		}
		try {
			File file = new File(Environment.getExternalStorageDirectory().getPath() + "/hoge.txt");
			if (! file.exists()) {
				file.createNewFile();
			}
			PrintWriter writer = new PrintWriter(new OutputStreamWriter(new FileOutputStream(file, true)));
			writer.println((new Date()) + ":" + message);
			writer.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * test thread for CanGather
	 *
	 */
	public class DummySendThread extends Thread {
		@Override
		public void run() {
			try {
				Thread.sleep(2000);
				LocalSocket socket = new LocalSocket();
				socket.connect(new LocalSocketAddress(SERVER_SOCKET_NAME));
				OutputStream out = socket.getOutputStream();
				out.write("itsecu_2014-09-27-15-34-46.csv".getBytes());
				out.write(0x0d);
				out.write("1411799693210,0,0,0,0\n".getBytes());
				out.close();
				socket.close();
			} catch (Exception e) {}
		}
	}
}
