package com.ui;

import android.widget.ImageButton;

import com.ghostagent.R;
import com.ghostagent.SoundManagementActivity;

/**
 * Created by linna on 11/13/2016.
 */
public class DriveButton extends RadioButton {
    public static final int AUTO = 1;
    public static final int NORMAL = 0;
    public static final int PURSUIT = 2;

    public ImageButton auto;
    public ImageButton normal;
    public ImageButton pursuit;

    public DriveButton(SoundManagementActivity activity) {
        super(activity);
        auto = (ImageButton)activity.findViewById(R.id.autoCruise);
        auto.setOnClickListener(activity);
        normal = (ImageButton)activity.findViewById(R.id.normalCruise);
        normal.setOnClickListener(activity);
        pursuit = (ImageButton)activity.findViewById(R.id.pursuit);
        pursuit.setOnClickListener(activity);

        refresh();
    }

    @Override
    public void refresh() {
        auto.setImageResource(R.drawable.autocruise);
        normal.setImageResource(R.drawable.normalcruise);
        pursuit.setImageResource(R.drawable.pursuit);

        switch (getMode()) {
            case AUTO:
                auto.setImageResource(R.drawable.pressed_autocruise);
                break;
            case NORMAL:
                normal.setImageResource(R.drawable.pressed_normalcruise);
                break;
            case PURSUIT:
                pursuit.setImageResource(R.drawable.pressed_pursuit);
                break;
        }
    }
}

