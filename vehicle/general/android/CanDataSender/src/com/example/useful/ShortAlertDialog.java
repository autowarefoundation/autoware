package com.example.useful;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.DialogInterface.OnDismissListener;
import android.app.Activity;
public class ShortAlertDialog {
	
	public static void sad(String title,String message,Context con) {		
		AlertDialog.Builder alertDialogBuilder = new AlertDialog.Builder(con);		
		alertDialogBuilder.setTitle(title);			        
        alertDialogBuilder.setMessage(message);		        
        alertDialogBuilder.setCancelable(true);
        AlertDialog alertDialog = alertDialogBuilder.create();
        alertDialog.show();		   
	}
	
	public static void sad_finish(String title,String message,final Context con) {		
		AlertDialog.Builder alertDialogBuilder = new AlertDialog.Builder(con);		
		alertDialogBuilder.setTitle(title);			        
        alertDialogBuilder.setMessage(message);		        
        alertDialogBuilder.setCancelable(true);
        AlertDialog alertDialog = alertDialogBuilder.create();
        alertDialog.setOnDismissListener(new OnDismissListener() {
        	@Override
        	public void onDismiss(DialogInterface dialog) {
        		((Activity) con).finish();
        	}
        });
        alertDialog.show();		   
	}
}