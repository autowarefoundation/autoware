package com.example.candatasender.sender;

public class SendCycle {
	//デフォルト値を3000msに設定
	private static int cycleTime = 3000;
	// 自動更新かどうか
	private static boolean isAutoUpdate = false;

	public static int getCycleTime() {
		return cycleTime;
	}

	public static void setCycleTime(int cycle_time) {
		SendCycle.cycleTime = cycle_time;
	}
	
	public static boolean getAutoUpdate() {
		return isAutoUpdate;
	}
	
	public static void setAutoUpdate(boolean autoUpdate) {
		isAutoUpdate = autoUpdate;
	}
	
}
