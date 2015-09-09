package com.example.show.voice;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.widget.TextView;

public class NotificationEmbeddedActivity extends Activity {

    public static final String EXTRA_KEY_COUNT = "extra_key_count";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_notification);

        TextView textView = (TextView) findViewById(R.id.text);
        Intent intent = getIntent();
        int count = intent.getIntExtra(EXTRA_KEY_COUNT, -1);
        if (count >= 0) {
            textView.setText(String.format("count is %d", count));
        } else {
            textView.setText(String.format("Hello world!"));
        }
    }
}
