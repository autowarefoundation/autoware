package com.ui;

import android.widget.ImageButton;

import com.ghostagent.R;
import com.ghostagent.SoundManagementActivity;

/**
 * Created by linna on 11/13/2016.
 */
public 	class S2Button extends RadioButton {
    public static final int NG = 1;
    public static final int OK = 2;

    public ImageButton s2;

    public S2Button(SoundManagementActivity activity) {
        super(activity);
        s2 = (ImageButton)activity.findViewById(R.id.s2);
        s2.setOnClickListener(activity);

        refresh();
    }

    @Override
    public void refresh() {
        s2.setImageResource(R.drawable.app_s2);

        switch (getMode()) {
            case NG:
                s2.setImageResource(R.drawable.pressed_app_s2_ng);
                break;
            case OK:
                s2.setImageResource(R.drawable.pressed_app_s2_ok);
                break;
        }
    }
}

