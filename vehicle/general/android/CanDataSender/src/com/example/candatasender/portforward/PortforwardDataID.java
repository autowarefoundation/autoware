package com.example.candatasender.portforward;

public enum PortforwardDataID {

	ID_NAME		(0,	"接続名"),
	ID_SSHPORT	(1, "sshポート"),
	ID_SSHHOST	(2, "sshホスト名"),
	ID_SSHUSER	(3, "sshユーザ名"),
	ID_SSHPASS	(4, "sshパスワード"),
	ID_LOCALPORT(5, "ローカルポート"),
	ID_FWDHOST	(6, "接続ホスト名"),
	ID_FWDPORT	(7, "接続ポート"),
	ID_COUNT	(8, "要素数");
	
	private int num;
	private String name;
	
	private PortforwardDataID(int n, String str) {
		num = n;
		name = str;
	}
	
	public int toInt() {
		return num;
	}
	
	public String toString() {
		return name;
	}
}