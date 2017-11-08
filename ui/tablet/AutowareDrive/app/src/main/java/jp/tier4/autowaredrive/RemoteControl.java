package jp.tier4.autowaredrive;

import android.content.Context;
import android.content.Intent;
import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Matrix;
import android.graphics.Point;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.Display;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.ImageButton;
import android.widget.ProgressBar;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import org.eclipse.paho.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

/**
 * Created by yuki.iida on 2017/06/20.
 */

public class RemoteControl extends AppCompatActivity implements View.OnTouchListener , CompoundButton.OnCheckedChangeListener {
    private final static String TAG = "RemoteControl";

    static private String CLIENTID_HEAD = "remote_";
    static private String TOPIC = "vehicle/";
    static private int REMOTE_MODE = 4;
    static private int AUTO_MODE = 3;
    static private int EMERGENCY_MODE = 1;
    static private int NONEMERGENCY_MODE = 0;
    static private int STEERING_MAX_VAL = 600;
    static private float STEERING_ANGLE_RATION = 1 / 1;

    private Context mContext;
    private int mVehicleId;
    /*** MQTT ***/
    private MqttAndroidClient mqttAndroidClient;
    private MqttConnectOptions mqttConnectOptions;
    private String mqttId;
    private String mqttHost;
    private int mqttPort;
    private String mqttBrokerURI;
    private String mqttPublishTopic;
    private String mqttSubscribeTopic;

    /*** UI ***/
    private ProgressBar accelBar, brakeBar;
    private Button accelButton, brakeButton;
    private ToggleButton remoteControlButton, emergencyButton;
    private int displayHeight, displayWidth;
    private SeekBar steeringBar;

    /*** Steering ***/
    private ImageButton steeringImageButton;
    private Bitmap steeringBitmap;
    private int steeringBitmapWidth;
    private int steeringBitmapHeight;

    private float baseTouchPointx = 0;
    private float baseTouchPointy = 0;
    private double currentSteeringAngle = 0;

    /*** VehicleInfo ***/
    private TextView spped_label;
    private TextView rpm_label;
    private TextView mode_label;
    private TextView fps_label;
    private TextView gear_preview;

    /*** Control Comand ***/
    private ControlComand mControlComand;
    private CanInfo mCanInfo;

    /*** Thread ***/
    ControlComandUploader mControlComandUploader = null;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_remote_control2);

        mControlComand = ControlComand.getInstance();
        mCanInfo = CanInfo.getInstance();
        mContext = getApplicationContext();

        Intent intent = getIntent();
        Bundle bundle = intent.getExtras();
        if (bundle != null) {
            this.mqttHost = bundle.getString("address");
            this.mqttPort = bundle.getInt("port");
            this.mqttBrokerURI = "tcp://" + mqttHost + ":" + mqttPort;
            this.mVehicleId = bundle.getInt("vehicle_id");
            this.mqttPublishTopic = TOPIC + mVehicleId + "/remote_cmd";
            this.mqttSubscribeTopic = TOPIC + mVehicleId + "/can_info";
            this.mqttId = CLIENTID_HEAD + mVehicleId + "_" + String.valueOf(System.currentTimeMillis());

            Log.i("RemoteControl", this.mqttBrokerURI + ", " + this.mqttPublishTopic + ", " + this.mqttId);
        }

        mqttAndroidClient = new MqttAndroidClient(this, mqttBrokerURI, mqttId);
        mqttConnectOptions = new MqttConnectOptions();

        mqttAndroidClient.setCallback(new MqttCallback() {
            @Override
            public void connectionLost(Throwable cause) {
                Log.i(TAG, "Connection was lost!");
                gear_preview.setText("P");
            }

            @Override
            public void messageArrived(String topic, MqttMessage message) throws Exception {
                String arrivedMessage = new String(message.getPayload());
                mCanInfo.parseCanInfo(arrivedMessage);

                // Set Vehiclenfo
                spped_label.setText("Speed: " + String.valueOf(mCanInfo.speed) + " km/h");
                rpm_label.setText("RPM: " + String.valueOf(mCanInfo.rpm));
                mode_label.setText("Mode: " + String.valueOf(mCanInfo.drvmode));
                gear_preview.setText("D");

                accelBar.setProgress((int) ((mCanInfo.drivepedal / 1000) * 100));
                brakeBar.setProgress((int) (((mCanInfo.brakepedal - 225) / 3000) * 100));
                Log.v(TAG, "VehicleInfo: " + mCanInfo.speed + ", " + mCanInfo.rpm + ", " + mCanInfo.drvmode + ", " + mCanInfo.drivepedal + ", " + mCanInfo.brakepedal);

                if (mControlComand.modeCmd != REMOTE_MODE) {
                    currentSteeringAngle = mCanInfo.angle * STEERING_ANGLE_RATION;
                    setCurrentSteeringAngle(currentSteeringAngle);
                    mControlComand.steeringCmd = (float) (0.5 + currentSteeringAngle / STEERING_MAX_VAL / 2);
                    steeringBar.setProgress((int) (mControlComand.steeringCmd * 100));
                }
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken token) {
                Log.i(TAG, "Delivery Complete!");
            }
        });

        /*** Connect MQTT Broker ***/
        connectMqttBroker();

        /*** UI ***/
        accelButton = (Button) this.findViewById(R.id.accel_button);
        accelButton.setOnTouchListener(this);
        brakeButton = (Button) this.findViewById(R.id.brake_button);
        brakeButton.setOnTouchListener(this);

        steeringImageButton = (ImageButton) findViewById(R.id.steering_image_button);

        accelBar = (ProgressBar) findViewById(R.id.accel_progressbar);
        accelBar.setMax(100);
        accelBar.setProgress(0);

        brakeBar = (ProgressBar) findViewById(R.id.brake_progressbar);
        brakeBar.setMax(100);
        brakeBar.setProgress(0);

        steeringBar = (SeekBar) findViewById(R.id.steering_seekbar);
        steeringBar.setMax(100);
        steeringBar.setProgress(50);

        WindowManager wm = (WindowManager) getSystemService(WINDOW_SERVICE);
        Display disp = wm.getDefaultDisplay();
        Point size = new Point();
        disp.getSize(size);
        displayWidth = size.x;
        displayHeight = size.y - getStatusBarHeight() * 8 / 9;

        remoteControlButton = (ToggleButton) findViewById(R.id.remote_control_button);
        remoteControlButton.setOnCheckedChangeListener(this);

        emergencyButton = (ToggleButton) findViewById(R.id.emergency_button);
        emergencyButton.setOnCheckedChangeListener(this);

        mControlComandUploader = new ControlComandUploader(mqttAndroidClient, mqttConnectOptions, mqttPublishTopic);

        setCurrentSteeringAngle(currentSteeringAngle);

        /*** VehicleInfo ***/
        spped_label = (TextView) findViewById(R.id.speed_label);
        rpm_label = (TextView) findViewById(R.id.rpm_label);
        mode_label = (TextView) findViewById(R.id.mode_label);
        fps_label = (TextView) findViewById(R.id.fps_label);
        gear_preview = (TextView) findViewById(R.id.gear_preview);

    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        int buttonWidth = findViewById(R.id.steering_image_button).getWidth();
        int buttonHeight = findViewById(R.id.steering_image_button).getHeight();

        setSteeringImageButtonSize(buttonWidth, buttonHeight);
    }

    public void setSteeringImageButtonSize(int buttonWidth, int buttonHeight) {
        Bitmap steeringBitmapRaw = BitmapFactory.decodeResource(getResources(), R.drawable.steering);
        float ratio = 0;
        Matrix matrix = new Matrix();
        if(buttonWidth > buttonHeight) {
            ratio = (float) buttonHeight / (float)steeringBitmapRaw.getHeight();
        }
        else {
            ratio = (float) buttonWidth / (float)steeringBitmapRaw.getWidth();
        }
        matrix.preScale(ratio, ratio);

        steeringBitmap = Bitmap.createBitmap(steeringBitmapRaw, 0, 0, steeringBitmapRaw.getWidth(), steeringBitmapRaw.getHeight(), matrix, true);
        steeringImageButton.setImageBitmap(steeringBitmap);
        steeringBitmapWidth = steeringBitmap.getWidth();
        steeringBitmapHeight = steeringBitmap.getHeight();
        steeringImageButton.setOnTouchListener(this);
    }

    public void setCurrentSteeringAngle(double angle) {
        try{
            Matrix matrix = new Matrix();
            matrix.setRotate((float)angle, steeringBitmapWidth/2, steeringBitmapHeight/2);
            Bitmap bitmap2 = Bitmap.createBitmap(steeringBitmap, 0, 0, steeringBitmapWidth, steeringBitmapHeight, matrix, true);
            steeringImageButton.setImageBitmap(bitmap2);
        }
        catch (Exception e) {
            Log.e(TAG, e.toString());
        }
    }

    public int getSteeringAngle(float x, float y) {
        double rad = getRadian(steeringBitmapWidth/2, steeringBitmapHeight/2, x, y);
        //convert radian to degree
        int target_angle = (int)(rad * 180d / Math.PI) + 90;

        return target_angle;
    }

    protected double getRadian(double x, double y, double x2, double y2) {
        double radian = Math.atan2(y2 - y,x2 - x);
        return radian;
    }

    public int getStatusBarHeight() {
        int result = 0;
        int resourceId = getResources().getIdentifier("status_bar_height", "dimen", "android");
        if (resourceId > 0) {
            result = getResources().getDimensionPixelSize(resourceId);
        }
        return result;
    }

    /*** Connect MQTT Broker ***/
    private void connectMqttBroker() {
        try {
            mqttAndroidClient.connect(mqttConnectOptions, new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    Log.i(TAG, "Connection Success! isConnected: " + mqttAndroidClient.isConnected());

                    try {
                        mqttAndroidClient.subscribe(mqttSubscribeTopic, 0);
                        Log.d(TAG, "subscribe");
                    } catch (MqttException e) {
                        Log.d(TAG, e.toString());
                    }
                }

                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    Log.i(TAG, "Connection Failure! isConnected: " + exception.getMessage() + ", " + mqttAndroidClient.isConnected());
                    Toast.makeText(mContext, "Connection Failure", Toast.LENGTH_LONG).show();
                }
            });
        } catch (MqttException ex) {
            Log.e(TAG, ex.toString());
        }
    }

    @Override
    public boolean onTouch(View v, MotionEvent event) {
        // Acc/Brake
        if(v == accelButton || v == brakeButton) {
            if(event.getAction() == MotionEvent.ACTION_DOWN ||
                    event.getAction() == MotionEvent.ACTION_MOVE) {
                float rate = 100 - event.getY() / displayHeight * 100;
                if(rate >= 50) {
                    if(v == accelButton) {
                        mControlComand.accelCmd = (rate - 50) / 50;
                        if(mControlComand.accelCmd > 1)
                            mControlComand.accelCmd = (float)1.0;
                        mControlComand.brakeCmd = 0;
                    }
                    else if(v == brakeButton) {
                        mControlComand.brakeCmd = (rate - 50) / 50;
                        if(mControlComand.brakeCmd > 1)
                            mControlComand.brakeCmd = (float)1.0;
                        mControlComand.accelCmd = 0;
                    }
                }
                else {
                    mControlComand.accelCmd = 0;
                    mControlComand.brakeCmd = 0;
                }
//                else {
//                    mControlComand.accelCmd = 0;
//                    mControlComand.brakeCmd = (50 - rate) / 50;
//                    if(mControlComand.accelCmd > 1)
//                        mControlComand.accelCmd = (float)1.0;
//                }
            }

            accelBar.setProgress(Math.round(mControlComand.accelCmd * 100));
            brakeBar.setProgress(Math.round(mControlComand.brakeCmd * 100));

        }
        else if(v == steeringImageButton) {
            if(event.getAction() == MotionEvent.ACTION_DOWN) {
                baseTouchPointx = event.getX();
                baseTouchPointy = event.getY();
            }
            else if(event.getAction() == MotionEvent.ACTION_MOVE) {
                double angle_diff = getVectorAngle(baseTouchPointx, baseTouchPointy, event.getX(), event.getY());
                currentSteeringAngle = currentSteeringAngle + angle_diff;

                if(currentSteeringAngle > STEERING_MAX_VAL) {
                    currentSteeringAngle = STEERING_MAX_VAL;
                }
                else if (currentSteeringAngle < -STEERING_MAX_VAL) {
                    currentSteeringAngle = -STEERING_MAX_VAL;
                }

                if(-STEERING_MAX_VAL < currentSteeringAngle && currentSteeringAngle < STEERING_MAX_VAL) {
                    setCurrentSteeringAngle(currentSteeringAngle);
                    baseTouchPointx = event.getX();
                    baseTouchPointy = event.getY();

                    mControlComand.steeringCmd = (float)(0.5 + currentSteeringAngle / STEERING_MAX_VAL / 2);
                    steeringBar.setProgress((int)(mControlComand.steeringCmd * 100));
                }
            }
        }
        return false;
    }

    public double getVectorAngle(float x1, float y1, float x2, float y2) {

        float v1x = x1 - steeringBitmapWidth/2;
        float v1y = y1 - steeringBitmapHeight/2;

        float v2x = x2 - steeringBitmapWidth/2;
        float v2y = y2 - steeringBitmapHeight/2;

        float babc = v1x * v2x + v1y * v2y;
        float ban = (v1x * v1x) + (v1y * v1y);
        float bcn = (v2x * v2x) + (v2y * v2y);
        double radian = Math.acos(babc / (Math.sqrt(ban * bcn)));

        float tmp = (x2 - steeringBitmapWidth/2) * (y1 - steeringBitmapHeight/2) - (x1 - steeringBitmapWidth/2) * (y2 - steeringBitmapHeight/2);

        double angle = 0;
        if(0 < tmp) {
            angle  = -radian * 180 / Math.PI;
        }
        else {
            angle  = radian * 180 / Math.PI;
        }

        return angle;
    }

    @Override
    public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
        if(buttonView == remoteControlButton) {
            if(mqttAndroidClient.isConnected() == false) {
                connectMqttBroker();
            }
           if(isChecked == true) {
               mControlComand.modeCmd = REMOTE_MODE;
               if(mControlComandUploader == null) {
                   mControlComandUploader = new ControlComandUploader(mqttAndroidClient, mqttConnectOptions, mqttPublishTopic);
               }
               mControlComandUploader.execute();
           }
           else {
               mControlComand.modeCmd = AUTO_MODE;
               mControlComandUploader.setAutoMode();
               mControlComandUploader.stopLogUpload();
               mControlComandUploader = null;
           }
        }
        else if(buttonView == emergencyButton) {
            if(isChecked == true) {
                mControlComand.emergencyCmd = EMERGENCY_MODE;
            }
            else {
                mControlComand.emergencyCmd = NONEMERGENCY_MODE;
            }
        }
        Log.i(TAG, "Toggle: modeCmd = " +  mControlComand.modeCmd + ", emegencyCmd = " + mControlComand.emergencyCmd);
    }

    @Override
    public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
        Log.i(TAG, "onConfigrationChanged()");
    }

    @Override
    public void onDestroy(){
        super.onDestroy();
        Log.i(TAG, "onDestroy()");
        /*** Stop Upload ***/
        if(mControlComandUploader != null) {
            mControlComandUploader.stopLogUpload();
            mControlComandUploader = null;
        }
        /*** Disconnect MQTT ***/
        try{
            if(mqttAndroidClient.isConnected()) {
                mqttAndroidClient.disconnect();
                Log.i(TAG, "MQTT disconnect");
            }
            mqttAndroidClient.unregisterResources();

        } catch(MqttException e) {
            Log.e(TAG, e.toString());
        }
    }
}
