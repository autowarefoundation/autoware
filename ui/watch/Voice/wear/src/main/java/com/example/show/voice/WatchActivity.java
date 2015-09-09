package com.example.show.voice;

import android.app.Activity;
import android.app.Notification;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.NotificationCompat;
import android.support.v4.app.NotificationManagerCompat;
import android.support.wearable.view.WatchViewStub;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

public class WatchActivity extends Activity {

    private TextView mTextView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_watch);
        final Context context = this;
        final WatchViewStub stub = (WatchViewStub) findViewById(R.id.watch_view_stub);
        stub.setOnLayoutInflatedListener(new WatchViewStub.OnLayoutInflatedListener() {
            @Override
            public void onLayoutInflated(WatchViewStub stub) {
                mTextView = (TextView) stub.findViewById(R.id.text);
                ((Button) findViewById(R.id.button_notification_to_open_activity)).setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        sendNotificationToOpenCustomActivity(context);
                    }
                });
                ((Button) findViewById(R.id.button_notification_display_intent)).setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        sendNotificationWithDisplayIntent(context);
                    }
                });
            }
        });
    }

    private void sendNotificationToOpenCustomActivity(Context context) {
        Intent viewIntent = new Intent(context, WatchActivity.class);
        PendingIntent pendingViewIntent = PendingIntent.getActivity(context, 0, viewIntent, 0);

        Notification notification = new NotificationCompat.Builder(context)
                .setSmallIcon(R.drawable.ic_launcher)
                .setContentTitle("Wearableから送信")
                .setContentText("Wearableから送信したNotificationです")
                .addAction(R.drawable.ic_launcher, "Open", pendingViewIntent)
                .setLocalOnly(true)
                .extend(new NotificationCompat.WearableExtender().setContentAction(0).setHintHideIcon(true))
                .build();

        NotificationManagerCompat notificationManager = NotificationManagerCompat.from(context);
        notificationManager.notify(3000, notification);
    }

    private void sendNotificationWithDisplayIntent(Context context) {
        Intent viewIntent = new Intent(context, NotificationEmbeddedActivity.class);
        PendingIntent pendingViewIntent = PendingIntent.getActivity(context, 0, viewIntent, 0);

        Notification notification = new NotificationCompat.Builder(context)
                .setSmallIcon(R.drawable.ic_launcher)
                .setContentTitle("Wearableから送信")
                .setContentText("Wearableから送信したNotificationです")
                .setLocalOnly(true)
                .extend(new NotificationCompat.WearableExtender().setDisplayIntent(pendingViewIntent))
                .build();

        NotificationManagerCompat notificationManager = NotificationManagerCompat.from(context);
        notificationManager.notify(4000, notification);
    }
}
