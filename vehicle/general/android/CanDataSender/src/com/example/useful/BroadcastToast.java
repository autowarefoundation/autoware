package com.example.useful;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.widget.Toast;

public class BroadcastToast extends BroadcastReceiver {
	@Override
	public void onReceive(Context context, Intent intent) {	    
		Bundle bundle = intent.getExtras();
		String message = bundle.getString("message");	    
		Toast.makeText(context, message, Toast.LENGTH_SHORT).show();
	}
}