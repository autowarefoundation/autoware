package jp.ogwork.freetransform;

import jp.ogwork.freetransform.fragment.MainFragment;
import jp.ogwork.gesturetransformableview.view.GestureTransformableImageView;

import com.lylc.widget.circularprogressbar.CircularProgressBar;
import com.lylc.widget.circularprogressbar.CircularProgressBar.ProgressAnimationListener;

import android.content.Intent;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.ToggleButton;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.StringTokenizer;

public class MainActivity extends RosActivity {

    private static final int MP = ViewGroup.LayoutParams.MATCH_PARENT;

    private ToggleButton set_btn;
    private ToggleButton drive_btn;
    private ToggleButton navi_btn;
    private ToggleButton map_btn;
    private ToggleButton view_btn;
    private ToggleButton info_btn;

    private ImageView compass_view;

    private ImageView first_digital_view;
    private ImageView second_digital_view;

    private CircularProgressBar gas_circle;
    private CircularProgressBar brake_circle;

    private GestureTransformableImageView handle_view;

    private AutowareTouch autoware_touch;

    public MainActivity() {
        super("AutowareTouch", "AutowareTouch");
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        set_btn = (ToggleButton) findViewById(R.id.set_btn);
        drive_btn = (ToggleButton) findViewById(R.id.drive_btn);
        navi_btn = (ToggleButton) findViewById(R.id.navi_btn);
        map_btn = (ToggleButton) findViewById(R.id.map_btn);
        view_btn = (ToggleButton) findViewById(R.id.view_btn);
        info_btn = (ToggleButton) findViewById(R.id.info_btn);

        compass_view = (ImageView) findViewById(R.id.compass_view);

        first_digital_view = (ImageView) findViewById(R.id.first_digital_view);
        second_digital_view = (ImageView) findViewById(R.id.second_digital_view);

        gas_circle = (CircularProgressBar) findViewById(R.id.circularprogressbar1);
        gas_circle.setTitle("Gas");
        brake_circle = (CircularProgressBar) findViewById(R.id.circularprogressbar2);
        brake_circle.setTitle("Brake");

        handle_view = new GestureTransformableImageView(getApplicationContext(),
            GestureTransformableImageView.GESTURE_ROTATABLE);
        handle_view.setScaleType(ImageView.ScaleType.FIT_CENTER);
        handle_view.setImageResource(R.drawable.handle_dark);
        RelativeLayout.LayoutParams params = new RelativeLayout.LayoutParams(MP, MP);
        params.addRule(RelativeLayout.CENTER_IN_PARENT, 1);
        RelativeLayout rlGesture = (RelativeLayout) findViewById(R.id.parent_handle_view);
        rlGesture.addView(handle_view, params);

        handle_view.setOnTouchListener(new OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                handle_view.onTouch(v, event);
                autoware_touch.publishAngle(handle_view.getAngle());
                autoware_touch.publishSteer((int) handle_view.getAngle());
                return true;
            }
        });

        set_btn.setOnTouchListener(new OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {

                if (event.getAction() == MotionEvent.ACTION_DOWN && !set_btn.isChecked())
                    setOn();
                else if (event.getAction() == MotionEvent.ACTION_DOWN && set_btn.isChecked())
                    setOff();
                return true;
            }
        });

        drive_btn.setOnTouchListener(new OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {

                if (event.getAction() == MotionEvent.ACTION_DOWN && set_btn.isChecked() &&
                    !drive_btn.isChecked()) {
                    driveOn();
                } else if (event.getAction() == MotionEvent.ACTION_DOWN && set_btn.isChecked() &&
                           drive_btn.isChecked()) {
                    driveOff();
                }
                return true;
            }
        });

        navi_btn.setOnTouchListener(new OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {

                if (event.getAction() == MotionEvent.ACTION_DOWN && !navi_btn.isChecked()) {
                    naviOn();
                    Intent intent = new Intent(Intent.ACTION_MAIN);
                    intent.setClassName("com.example.autowareroute",
                                        "com.example.autowareroute.MainActivity");
                    startActivity(intent);
                } else if (event.getAction() == MotionEvent.ACTION_DOWN && navi_btn.isChecked())
                    naviOff();
                return true;
            }
        });

        map_btn.setOnTouchListener(new OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {

                if (event.getAction() == MotionEvent.ACTION_DOWN && !map_btn.isChecked())
                    mapOn();
                else if (event.getAction() == MotionEvent.ACTION_DOWN && map_btn.isChecked())
                    mapOff();
                return true;
            }
        });

        view_btn.setOnTouchListener(new OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {

                if (event.getAction() == MotionEvent.ACTION_DOWN && !view_btn.isChecked())
                    viewOn();
                else if (event.getAction() == MotionEvent.ACTION_DOWN && view_btn.isChecked())
                    viewOff();
                return true;
            }
        });

        info_btn.setOnTouchListener(new OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {

                if (event.getAction() == MotionEvent.ACTION_DOWN && !info_btn.isChecked())
                    infoOn();
                else if (event.getAction() == MotionEvent.ACTION_DOWN && info_btn.isChecked())
                    infoOff();
                return true;
            }
        });
    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    @Override
    protected void onResume() {
        super.onResume();
    }

    @Override
    protected void onRestart() {
        if (navi_btn.isChecked()) {
            naviOff();
            File file = new File(Environment.getExternalStorageDirectory().getPath() + "/MapRoute.txt");
            try {
                FileReader reader = new FileReader(file);
                BufferedReader breader = new BufferedReader(reader);
                String line;
                ArrayList<tablet_socket.Waypoint> waypoints = new ArrayList<tablet_socket.Waypoint>();
                while ((line = breader.readLine()) != null) {
                    StringTokenizer token = new StringTokenizer(line, ",");
                    if (token.countTokens() == 2) {
                        tablet_socket.Waypoint waypoint =
                            autoware_touch.mNode.getTopicMessageFactory().newFromType(tablet_socket.Waypoint._TYPE);
                        waypoint.setLat(Double.parseDouble(token.nextToken()));
                        waypoint.setLon(Double.parseDouble(token.nextToken()));
                        waypoints.add(waypoint);
                    }
                }
                breader.close();
                autoware_touch.publishRoute(waypoints);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        super.onRestart();
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration =
            NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeConfiguration.setNodeName("autoware_touch");

        autoware_touch = new AutowareTouch(this);
        nodeMainExecutor.execute(autoware_touch, nodeConfiguration);
    }

    private void connectedAutoware() {
        handle_view.setImageResource(R.drawable.handle);
    }

    private void notConnectedAutoware() {
        handle_view.setImageResource(R.drawable.handle_dark);
    }

    private void setOn() {
        set_btn.setChecked(true);
        autoware_touch.publishSet(true);
        drive_btn.setEnabled(true);
    }

    private void setOff() {
        set_btn.setChecked(false);
        autoware_touch.publishSet(false);
        driveOff();
        drive_btn.setEnabled(false);
    }

    private void driveOn() {
        drive_btn.setChecked(true);
        autoware_touch.publishDrive(true);
        autoware_touch.publishMode(AutowareTouch.DRIVE_MODE_AUTO);
    }

    private void driveOff() {
        drive_btn.setChecked(false);
        autoware_touch.publishDrive(false);
        autoware_touch.publishMode(AutowareTouch.DRIVE_MODE_MANUAL);
    }

    private void naviOn() {
        navi_btn.setChecked(true);
        autoware_touch.publishNavi(true);
    }

    private void naviOff() {
        navi_btn.setChecked(false);
        autoware_touch.publishNavi(false);
    }

    private void mapOn() {
        map_btn.setChecked(true);
        autoware_touch.publishMap(true);
    }

    private void mapOff() {
        map_btn.setChecked(false);
        autoware_touch.publishMap(false);
    }

    private void viewOn() {
        view_btn.setChecked(true);
        autoware_touch.publishView(true);
    }

    private void viewOff() {
        view_btn.setChecked(false);
        autoware_touch.publishView(false);
    }

    private void infoOn() {
        info_btn.setChecked(true);
        autoware_touch.publishInfo(true);
    }

    private void infoOff() {
        info_btn.setChecked(false);
        autoware_touch.publishInfo(false);
    }

    private int getDigitalResource(int x) {
        if (x < 0 || x > 9)
            return 0;

        int ret = 0;
        switch (x) {
        case 0:
            ret = R.drawable.digital_0;
            break;
        case 1:
            ret = R.drawable.digital_1;
            break;
        case 2:
            ret = R.drawable.digital_2;
            break;
        case 3:
            ret = R.drawable.digital_3;
            break;
        case 4:
            ret = R.drawable.digital_4;
            break;
        case 5:
            ret = R.drawable.digital_5;
            break;
        case 6:
            ret = R.drawable.digital_6;
            break;
        case 7:
            ret = R.drawable.digital_7;
            break;
        case 8:
            ret = R.drawable.digital_8;
            break;
        case 9:
            ret = R.drawable.digital_9;
            break;
        }

        return ret;
    }

    private void setDigital(int vel) {
        if (vel < 0 || vel > 99)
            return;
        int firstDigital = vel % 10;
        int src = getDigitalResource(firstDigital);
        first_digital_view.setImageResource(src);
        firstDigital = (vel / 10) % 10;
        if (firstDigital > 0)
            src = getDigitalResource(firstDigital);
        else
            src = R.drawable.digital_none;
        second_digital_view.setImageResource(src);
    }

    private class setDigitalHandle extends Thread {
        public setDigitalHandle() {
        }
        public void execute(Looper toLooper, String data) {
            final int vel = Integer.parseInt(data);
            new Handler(toLooper).post(new Runnable() {
                public void run() {
                    setDigital(vel);
                    return;
                }
            });
        }
    }

    private class setGasHandle extends Thread {
        public setGasHandle() {
        }
        public void execute(Looper toLooper, String data) {
            final int gas = Integer.parseInt(data);
            new Handler(toLooper).post(new Runnable() {
                public void run() {
                    gas_circle.setProgress(gas);
                    return;
                }
            });
        }
    }

    private class setBrakeHandle extends Thread {
        public setBrakeHandle() {
        }
        public void execute(Looper toLooper, String data) {
            final int brake = Integer.parseInt(data);
            new Handler(toLooper).post(new Runnable() {
                public void run() {
                    brake_circle.setProgress(brake);
                    return;
                }
            });
        }
    }

    private class setConnHandle extends Thread {
        public setConnHandle() {
        }
        public void execute(Looper toLooper, String data) {
            final boolean conn = Boolean.parseBoolean(data);
            new Handler(toLooper).post(new Runnable() {
                public void run() {
                    if (conn)
                        connectedAutoware();
                    else
                        notConnectedAutoware();
                    return;
                }
            });
        }
    }

    private class setTwistHandle extends Thread {
        public setTwistHandle() {
        }
        public void execute(Looper toLooper, String data) {
            final float twist = Float.parseFloat(data);
            new Handler(toLooper).post(new Runnable() {
                public void run() {
                    compass_view.setRotation(twist % 360);
                    return;
                }
            });
        }
    }

    private class setCanSpeedHandle extends Thread {
        public setCanSpeedHandle() {
        }
        public void execute(Looper toLooper, String data) {
            final double canSpeed = Double.parseDouble(data);
            new Handler(toLooper).post(new Runnable() {
                public void run() {
                    setDigital((int) canSpeed);
                    return;
                }
            });
        }
    }

    private class setCanBrakepedalHandle extends Thread {
        public setCanBrakepedalHandle() {
        }
        public void execute(Looper toLooper, String data) {
            final int canBrakepedal = Integer.parseInt(data);
            new Handler(toLooper).post(new Runnable() {
                public void run() {
                    brake_circle.setProgress(canBrakepedal);
                    return;
                }
            });
        }
    }

    private class setCanAngleHandle extends Thread {
        public setCanAngleHandle() {
        }
        public void execute(Looper toLooper, String data) {
            final double canAngle = Double.parseDouble(data);
            new Handler(toLooper).post(new Runnable() {
                public void run() {
                    compass_view.setRotation((float) canAngle % 360);
                    return;
                }
            });
        }
    }

    private class setModeHandle extends Thread {
        public setModeHandle() {
        }
        public void execute(Looper toLooper, String data) {
            final int mode = Integer.parseInt(data);
            new Handler(toLooper).post(new Runnable() {
                public void run() {
                    if (mode == AutowareTouch.DRIVE_MODE_AUTO)
                        connectedAutoware();
                    else
                        notConnectedAutoware();
                    return;
                }
            });
        }
    }

    public void startDigitalHandle(String data) {
        new setDigitalHandle().execute(Looper.getMainLooper(), data);
    }

    public void startGasHandle(String data) {
        new setGasHandle().execute(Looper.getMainLooper(), data);
    }

    public void startBrakeHandle(String data) {
        new setBrakeHandle().execute(Looper.getMainLooper(), data);
    }

    public void startConnHandle(String data) {
        new setConnHandle().execute(Looper.getMainLooper(), data);
    }

    public void startTwistHandle(String data) {
        new setTwistHandle().execute(Looper.getMainLooper(), data);
    }

    public void startCanSpeedHandle(String data) {
        new setCanSpeedHandle().execute(Looper.getMainLooper(), data);
    }

    public void startCanBrakepedalHandle(String data) {
        new setCanBrakepedalHandle().execute(Looper.getMainLooper(), data);
    }

    public void startCanAngleHandle(String data) {
        new setCanAngleHandle().execute(Looper.getMainLooper(), data);
    }

    public void startModeHandle(String data) {
        new setModeHandle().execute(Looper.getMainLooper(), data);
    }
}
