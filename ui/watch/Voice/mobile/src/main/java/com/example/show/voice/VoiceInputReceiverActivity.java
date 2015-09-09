package com.example.show.voice;

import android.app.Activity;
import android.os.Bundle;
import android.support.v4.app.RemoteInput;
import android.widget.TextView;

public class VoiceInputReceiverActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_voice_input_receiver);
        Bundle remoteInput = RemoteInput.getResultsFromIntent(getIntent());
        CharSequence message = remoteInput.getCharSequence(NotificationActivity.EXTRA_VOICE_REPLY);
        ((TextView) findViewById(R.id.message)).setText(String.format("入力メッセージは「%s」です。", message));
    }

}
