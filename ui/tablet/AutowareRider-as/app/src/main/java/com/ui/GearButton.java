package com.ui;

import android.widget.ImageButton;

import com.ghostagent.R;
import com.ghostagent.SoundManagementActivity;

/**
 * Created by linna on 11/13/2016.
 */
public class GearButton extends RadioButton {
    public static final int DRIVE = 1;
    public static final int REVERSE = 2;
    public static final int BRAKE = 3;
    public static final int NEUTRAL = 4;

    public ImageButton drive;
    public ImageButton reverse;
    public ImageButton brake;
    public ImageButton neutral;

    public GearButton(SoundManagementActivity activity) {
        super(activity);
        drive = (ImageButton) activity.findViewById(R.id.p3);
        drive.setOnClickListener(activity);
        reverse = (ImageButton)activity.findViewById(R.id.p4);
        reverse.setOnClickListener(activity);
        brake = (ImageButton)activity.findViewById(R.id.p1);
        brake.setOnClickListener(activity);
        neutral = (ImageButton)activity.findViewById(R.id.p2);
        neutral.setOnClickListener(activity);

        refresh();
    }

    @Override
    public void refresh() {
        drive.setImageResource(R.drawable.gear_d);
        reverse.setImageResource(R.drawable.gear_r);
        brake.setImageResource(R.drawable.gear_b);
        neutral.setImageResource(R.drawable.gear_n);

        switch (getMode()) {
            case DRIVE:
                drive.setImageResource(R.drawable.pressed_gear_d);
                break;
            case REVERSE:
                reverse.setImageResource(R.drawable.pressed_gear_r);
                break;
            case BRAKE:
                brake.setImageResource(R.drawable.pressed_gear_b);
                break;
            case NEUTRAL:
                neutral.setImageResource(R.drawable.pressed_gear_n);
                break;
        }
    }
}
