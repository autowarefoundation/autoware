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

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import android.os.AsyncTask;
import android.content.Context;
import android.app.ProgressDialog;
import android.util.Log;
import com.jcraft.jsch.*;
import com.example.useful.ShortAlertDialog;
import com.example.candatasender.portforward.SshTunnel;

public class RawDataScp extends AsyncTask<String, String, String> {
	private Context context;
	private ProgressDialog progressDialog = null;
	private String preExe = null;
	private String postExe = null;

	public RawDataScp(Context context) {
		this.context = context;
	}

	public RawDataScp(Context context, ProgressDialog progressDialog) {
		this.context = context;
		this.progressDialog = progressDialog;
	}

	public RawDataScp(Context context, ProgressDialog progressDialog, String preExe, String postExe) {
		this.context = context;
		this.progressDialog = progressDialog;
		this.preExe = preExe;
		this.postExe = postExe;
	}

	@Override
	protected void onPreExecute() {
		Log.d("MethodCall", "RawDataScp.onPreExecute");
		if ((context == null) || (progressDialog != null)) return;

		progressDialog = new ProgressDialog(context);
		progressDialog.setTitle("データ送信中");
		progressDialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
		progressDialog.show();
	}

	@Override
	protected String doInBackground(String... args) {
		Log.d("MethodCall", "RawDataScp.doInBackground");
		String dst = args[0];
		String sd  = args[1];

		publishProgress("送信準備");
		try {
			if (preExe != null) sshExe(preExe);

			Session session = SshTunnel.getInstance().GetSession();
			ChannelSftp channel = (ChannelSftp)session.openChannel("sftp");
			channel.connect();

			for (int i=2; i<args.length; ++i) {
				String src = args[i];
				File abstractSrcFile = new File(src);
				String parent = abstractSrcFile.getParent();
				if (parent == null) {
					parent = dst;
				} else {
					parent = dst + "/" + parent;
				}

				File file = new File(sd + "/" + src);
				if (file.isFile()) {
					String serverAbsPath = sshMkdir(parent);
					scpFile(channel, file, serverAbsPath);
				} else {
					scpDir(channel, file, parent);
				}
			}
			channel.disconnect();

			if (postExe != null) sshExe(postExe);
		} catch (JSchException e) {
			return e.toString();
		} catch (SftpException e) {
			return e.toString();
		} catch (IOException e) {
			return e.toString();
		}
		return null;
	}

	@Override
	protected void onProgressUpdate(String... values) {
		Log.d("MethodCall", "RawDataScp.onProgressUpdate");
		if (progressDialog != null) {
			progressDialog.setMessage(values[0]);
		}
	}

	@Override
	protected void onPostExecute(String result) {
		Log.d("MethodCall", "RawDataScp.onPostExecute");
		if (progressDialog != null) {
			progressDialog.dismiss();
			progressDialog = null;
		}
		if ((result != null) && (context != null)) {
			ShortAlertDialog.sad("Error message", result, context);
		}
	}

	/**
	 * ファイル送信
	 * @param channel チャンネル
	 * @param file 送信ファイル
	 * @param dst 送信先ディレクトリ
	 * @throws SftpException
	 * @throws IOException
	 */
	private void scpFile(ChannelSftp channel, File file, String dst) throws SftpException, IOException {
		if (!file.isFile()) return;

		Log.d("RawDataScp", "scpFile: " + file.getName() + ", " + dst);
		publishProgress("送信：" + file.getName());
		channel.cd(dst);
		channel.put(file.getAbsolutePath(), file.getName());
	}

	/**
	 * 
	 * @param channel チャンネル
	 * @param dir 送信元ディレクトリ
	 * @param dst 送信先ディレクトリ
	 * @throws SftpException
	 * @throws JSchException
	 * @throws IOException
	 */
	private void scpDir(ChannelSftp channel, File dir, String dst) throws SftpException, JSchException, IOException {
		if (!dir.isDirectory()) return;

		Log.d("RawDataScp", "scpDir: " + dir.getName() + ", " + dst);
		String serverAbsPath = sshMkdir(dst + "/" + dir.getName());
		File[] files = dir.listFiles();
		if (files != null) {
			for (File f: files) {
				if (f.isFile()) {
					scpFile(channel, f, serverAbsPath);
				} else {
					scpDir(channel, f, serverAbsPath);
				}
			}
		}
	}

	/**
	 * サーバに送信先のディレクトリを作成する
	 * @param dir 送信先ディレクトリ（サーバのディレクトリ）
	 * @return 送信先ディレクトリの絶対パス
	 * @throws JSchException
	 * @throws IOException
	 */
	private String sshMkdir(String dir) throws JSchException, IOException {
		return sshExe("mkdir -p " + dir + " && cd " + dir + " && pwd");
	}

	/**
	 * SSHコマンド実行
	 * @param command 実行するコマンド
	 * @return コマンド結果
	 * @throws JSchException
	 * @throws IOException
	 */
	private String sshExe(String command) throws JSchException, IOException {
		Session session = SshTunnel.getInstance().GetSession();
		ChannelExec channel = (ChannelExec)session.openChannel("exec");
		channel.setCommand(command);

		InputStream in = channel.getInputStream();
		InputStream er = channel.getErrStream();
		StringBuilder stdOutBuf = new StringBuilder();
		StringBuilder stdErrBuf = new StringBuilder();
		channel.connect();
		while (true) {
			readInputStream(in, stdOutBuf);
			readInputStream(er, stdErrBuf);
			if (channel.isClosed()) {
				int status = channel.getExitStatus();
				if (status != 0) {
					Log.i("CanDataSender", "sshExitStatus: " + status);
					Log.i("CanDataSender", "sshStderr: " + stdErrBuf.toString());
					throw new JSchException(stdErrBuf.toString());
				}
				break;
			}
			try { Thread.sleep(500); } catch (Exception e) {}
		}
		channel.disconnect();
		return stdOutBuf.toString();
	}

	private void readInputStream(InputStream is, StringBuilder sb) throws IOException {
		BufferedReader reader = new BufferedReader(new InputStreamReader(is));
		String str;
		while ((str = reader.readLine()) != null) {
			sb.append(str);
			/* 改行無視 */
		}
	}
}
