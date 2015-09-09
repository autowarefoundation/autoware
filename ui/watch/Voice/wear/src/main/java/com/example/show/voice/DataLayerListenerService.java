package com.example.show.voice;

import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.content.Intent;

import com.google.android.gms.wearable.DataEvent;
import com.google.android.gms.wearable.DataEventBuffer;
import com.google.android.gms.wearable.DataItem;
import com.google.android.gms.wearable.DataMap;
import com.google.android.gms.wearable.DataMapItem;
import com.google.android.gms.wearable.MessageEvent;
import com.google.android.gms.wearable.WearableListenerService;

/**
 * Created by poly on 7/6/14.
 */
public class DataLayerListenerService extends WearableListenerService {
    private static final String TAG = "DataLayerListenerService";
    private static final String COUNT_PATH = "/count";
    private static final String COUNT_KEY = "COUNT_KEY";
    private static final String START_ACTIVITY_PATH = "/start/MainActivity";

    @Override
    public void onMessageReceived(MessageEvent messageEvent) {
        if (START_ACTIVITY_PATH.equals(messageEvent.getPath())) {
            Intent intent = new Intent(this, WatchActivity.class);
            intent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
            startActivity(intent);
            return;
        }
    }

    @Override
    public void onDataChanged(DataEventBuffer dataEvents) {
        for (DataEvent event : dataEvents) {
            DataItem dataItem = event.getDataItem();
            if (COUNT_PATH.equals(dataItem.getUri().getPath())) {
                DataMap dataMap = DataMapItem.fromDataItem(dataItem).getDataMap();
                int count = dataMap.getInt(COUNT_KEY);

                // android:allowEmbedded="true" is required for target activity
                Intent intent = new Intent(this, NotificationEmbeddedActivity.class);
                intent.putExtra(NotificationEmbeddedActivity.EXTRA_KEY_COUNT, count);
                PendingIntent pendingIntent =
                        PendingIntent.getActivity(this, 0, intent, PendingIntent.FLAG_CANCEL_CURRENT);

                Notification notification = new Notification.Builder(this)
                        .setSmallIcon(R.drawable.ic_launcher)
                        .extend(
                                new Notification.WearableExtender()
                                        .setHintHideIcon(true)
                                        .setDisplayIntent(pendingIntent)
                        )
                        .build();

                NotificationManager notificationManager = (NotificationManager) getSystemService(NOTIFICATION_SERVICE);
                notificationManager.notify(1000, notification);

                break;
            }
        }
    }
}
