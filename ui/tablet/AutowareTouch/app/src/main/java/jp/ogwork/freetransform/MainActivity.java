package jp.ogwork.freetransform;

import jp.ogwork.freetransform.fragment.MainFragment;
import jp.ogwork.gesturetransformableview.view.GestureTransformableImageView;

import com.lylc.widget.circularprogressbar.CircularProgressBar;
import com.lylc.widget.circularprogressbar.CircularProgressBar.ProgressAnimationListener;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.ToggleButton;

public class MainActivity extends Activity {

    public static final String TAG = MainActivity.class.getName();

    private static final int MP = ViewGroup.LayoutParams.MATCH_PARENT;

    private ToggleButton set_btn;

    private ToggleButton drive_btn;

    private ImageView compass_view;

    private ImageView first_digital_view;

    private ImageView second_digital_view;

    private GestureTransformableImageView handle_view;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // if (savedInstanceState == null) {
        //     getFragmentManager().beginTransaction()
        //             .add(R.id.container, new MainFragment(), MainFragment.class.getName()).commit();
        // }

        set_btn = (ToggleButton) findViewById(R.id.set_btn);
        drive_btn = (ToggleButton) findViewById(R.id.drive_btn);
        compass_view = (ImageView) findViewById(R.id.compass_view);
        first_digital_view = (ImageView) findViewById(R.id.first_digital_view);
        second_digital_view = (ImageView) findViewById(R.id.second_digital_view);

        CircularProgressBar c1 = (CircularProgressBar) findViewById(R.id.circularprogressbar1);
        c1.setTitle("Gas");
        // c1.setSubTitle("2013");
        c1.setProgress(42);
        CircularProgressBar c2 = (CircularProgressBar) findViewById(R.id.circularprogressbar2);
        c2.setTitle("Brake");
        c2.setProgress(60);

        c2.animateProgressTo(0, 77, new ProgressAnimationListener() {

            @Override
            public void onAnimationStart() {
            }

            @Override
            public void onAnimationProgress(int progress) {
                // c2.setTitle(progress + "%");
            }

            @Override
            public void onAnimationFinish() {
                // c2.setSubTitle("done");
            }
        });

        setDigital(10);

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

                Log.d("Rotate", Float.toString(handle_view.getAngle()));
                handle_view.onTouch(v, event);
                compass_view.setRotation(handle_view.getAngle());
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
                    connectedAutoware();
                } else if (event.getAction() == MotionEvent.ACTION_DOWN && set_btn.isChecked() &&
                           drive_btn.isChecked()) {
                    driveOff();
                    notConnectedAutoware();
                }
                return true;
            }
        });
    }

    private void connectedAutoware() {
        handle_view.setImageResource(R.drawable.handle);
    }

    private void notConnectedAutoware() {
        handle_view.setImageResource(R.drawable.handle_dark);
    }

    private void setOn() {
        set_btn.setChecked(true);
        drive_btn.setEnabled(true);
    }

    private void setOff() {
        set_btn.setChecked(false);
        driveOff();
        drive_btn.setEnabled(false);
    }

    private void driveOn() {
        drive_btn.setChecked(true);
    }

    private void driveOff() {
        drive_btn.setChecked(false);
    }

    private void setDigital(int speed) {

        if (speed < 0 || speed > 99) {
            Log.e("setDigital", "speed is out of range");
            return;
        }

        int firstDigital = speed % 10;
        int src = getDigitalResource(firstDigital);
        first_digital_view.setImageResource(src);

        firstDigital = (speed / 10) % 10;
        if (firstDigital > 0)
            src = getDigitalResource(firstDigital);
        else
            src = R.drawable.digital_none;
        second_digital_view.setImageResource(src);
    }

    private int getDigitalResource(int x) {

        if (x < 0 || x > 9) {
            Log.e("getDigitResource", "x is out of range");
            return 0;
        }

        int res = 0;
        switch (x) {
            case 0:
                res = R.drawable.digital_0;
                break;
            case 1:
                res = R.drawable.digital_1;
                break;
            case 2:
                res = R.drawable.digital_2;
                break;
            case 3:
                res = R.drawable.digital_3;
                break;
            case 4:
                res = R.drawable.digital_4;
                break;
            case 5:
                res = R.drawable.digital_5;
                break;
            case 6:
                res = R.drawable.digital_6;
                break;
            case 7:
                res = R.drawable.digital_7;
                break;
            case 8:
                res = R.drawable.digital_8;
                break;
            case 9:
                res = R.drawable.digital_9;
                break;
        }

        return res;
    }
}
