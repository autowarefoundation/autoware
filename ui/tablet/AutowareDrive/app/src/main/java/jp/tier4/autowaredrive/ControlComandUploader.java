package jp.tier4.autowaredrive;

import android.os.AsyncTask;
import android.util.Log;

import org.eclipse.paho.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

/**
 * Created by yuki.iida on 2017/06/20.
 */

public class ControlComandUploader extends AsyncTask<Void,Integer,Boolean> {
    private final static String TAG = "ControlComandUploader";

    static boolean isActive = true;
    private MqttAndroidClient mqttAndroidClient;
    private MqttConnectOptions mqttConnectOptions;
    private String mqttTopic;
    /*** Upload Period (ms) ***/
    private static final int defaultPeriod = 100;
    /*** Control Comand ***/
    private ControlComand mControlComand;

    public ControlComandUploader(MqttAndroidClient mqttAndroidClient, MqttConnectOptions mqttConnectOptions, String mqttTopic) {
        mControlComand = ControlComand.getInstance();
        this.mqttAndroidClient = mqttAndroidClient;
        this.mqttConnectOptions = mqttConnectOptions;
        this.mqttTopic = mqttTopic;
        this.isActive = true;
    }

    private boolean publishMessage(String message) {
        /*** Publish ***/
        try {
            Log.i(TAG, "Publish " + message);
            mqttAndroidClient.publish(mqttTopic, new MqttMessage(message.getBytes()));
            return true;
        } catch (Exception e) {
            Log.e(TAG, e.toString());
            return false;
        }
    }

    public void setAutoMode() {
        if(mqttAndroidClient.isConnected() == false) {
            connectMqttBroker();
        }
        try {
            String message = mControlComand.accelCmd + "," + mControlComand.brakeCmd + "," +
                    mControlComand.steeringCmd + "," + mControlComand.gearCmd + "," +
                    mControlComand.modeCmd + "," + mControlComand.emergencyCmd;
            publishMessage(message);
        } catch (Exception e) {
            Log.e(TAG, e.toString());
        }

    }

    public void stopLogUpload() {
        /*** Stop Thread ***/
        this.isActive = false;
    }

    @Override
    protected Boolean doInBackground(Void... params) {
        while (isActive) {
            if(mqttAndroidClient.isConnected() == false) {
                connectMqttBroker();
            }
            try {
                String message = mControlComand.accelCmd + "," + mControlComand.brakeCmd + "," +
                        mControlComand.steeringCmd + "," + mControlComand.gearCmd + "," +
                        mControlComand.modeCmd + "," + mControlComand.emergencyCmd;
                publishMessage(message);
                Thread.sleep(defaultPeriod);
            } catch (Exception e) {
                Log.e(TAG, e.toString());
            }
        }

        return null;
    }

    /*** Connect MQTT Broker ***/
    private void connectMqttBroker() {
        try {
            mqttAndroidClient.connect(mqttConnectOptions, new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    Log.i(TAG, "Connection Success! isConnected: " + mqttAndroidClient.isConnected());
                }

                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    Log.e(TAG, "Connection Failure! isConnected: " + mqttAndroidClient.isConnected());
                }
            });
        } catch (MqttException ex) {
            Log.e(TAG, ex.toString());
        }
    }
}
