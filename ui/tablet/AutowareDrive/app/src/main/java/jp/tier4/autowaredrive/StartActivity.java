package jp.tier4.autowaredrive;

import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.KeyEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.inputmethod.EditorInfo;
import android.widget.ArrayAdapter;
import android.widget.AutoCompleteTextView;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;

import static android.Manifest.permission.READ_CONTACTS;

/**
 * A login screen that offers login via email/password.
 */
public class StartActivity extends AppCompatActivity {
    private static final String TAG = "StartActivity";

    private static final String MQTT_ADDRESS = "192.168.1.1";
    private static final int MQTT_PORT = 1883;
    private static final int VEHICLE_ID = 1;

    // UI references.
    private AutoCompleteTextView mAdressView;
    private EditText mPortView;
    private EditText mVhicleIdView;
    private View mProgressView;
    private View mLoginFormView;
    private SharedPreferences mSharedPreferences;
    private SharedPreferences.Editor mEditor;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_start);

        mSharedPreferences = PreferenceManager.getDefaultSharedPreferences(this);
        mEditor = mSharedPreferences.edit();

        String address = mSharedPreferences.getString("Address", MQTT_ADDRESS);
        String port = String.valueOf(mSharedPreferences.getInt("Port", MQTT_PORT));
        String vehicleId = String.valueOf(mSharedPreferences.getInt("Id", VEHICLE_ID));

        // Set up the login form.
        mAdressView = (AutoCompleteTextView) findViewById(R.id.address);
        mAdressView.setText(address);

        mPortView = (EditText) findViewById(R.id.port);
        mPortView.setText(port);
        mPortView.setOnEditorActionListener(new TextView.OnEditorActionListener() {
            @Override
            public boolean onEditorAction(TextView textView, int id, KeyEvent keyEvent) {
                if (id == R.id.start || id == EditorInfo.IME_NULL) {
                    attemptStart();
                    return true;
                }
                return false;
            }
        });

        mVhicleIdView = (EditText) findViewById(R.id.vehicle_id);
        mVhicleIdView.setText(vehicleId);
        mVhicleIdView.setOnEditorActionListener(new TextView.OnEditorActionListener() {
            @Override
            public boolean onEditorAction(TextView textView, int id, KeyEvent keyEvent) {
                if (id == R.id.start || id == EditorInfo.IME_NULL) {
                    attemptStart();
                    return true;
                }
                return false;
            }
        });

        Button mStartButton = (Button) findViewById(R.id.start_button);
        mStartButton.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View view) {
                attemptStart();
            }
        });

        mLoginFormView = findViewById(R.id.login_form);
        mProgressView = findViewById(R.id.login_progress);
    }

    private void attemptStart() {
        // Store values at the time of the start attempt.
        String address = mAdressView.getText().toString();
        int port = Integer.parseInt(mPortView.getText().toString());
        int vehicle_id = Integer.parseInt(mVhicleIdView.getText().toString());

        boolean cancel = false;
        View focusView = null;

        if (cancel) {
            focusView.requestFocus();
        } else {
            Intent intent = new Intent(this, RemoteControl.class);
            Bundle bandle = new Bundle();

            bandle.putString("address", address);
            bandle.putInt("port", port);
            bandle.putInt("vehicle_id", vehicle_id);
            intent.putExtras(bandle);
            mEditor.putString("Address", address);
            mEditor.putInt("Port", port);
            mEditor.putInt("Id", vehicle_id);
            mEditor.commit();
            Log.i(TAG, address + ", " + port + ", " + vehicle_id);

            startActivity(intent);
        }
    }

    private boolean isEmailValid(String email) {
        //TODO: Replace this with your own logic
        return email.contains("@");
    }

    private boolean isPasswordValid(String password) {
        //TODO: Replace this with your own logic
        return password.length() > 4;
    }
}