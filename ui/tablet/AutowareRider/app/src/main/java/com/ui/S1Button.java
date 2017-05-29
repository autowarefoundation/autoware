package com.ui;

import android.widget.ImageButton;

import com.ghostagent.R;
import com.ghostagent.SoundManagementActivity;

/**
 * Created by linna on 11/13/2016.
 */
public 	class S1Button extends RadioButton {
    public static final int NG = 1;
    public static final int OK = 2;

    public ImageButton s1;

    public S1Button(SoundManagementActivity activity) {
        super(activity);
        s1 = (ImageButton)activity.findViewById(R.id.s1);
        s1.setOnClickListener(activity);

        refresh();
    }

    @Override
    public void refresh() {
        s1.setImageResource(R.drawable.app_s1);

        switch (getMode()) {
            case NG:
                s1.setImageResource(R.drawable.pressed_app_s1_ng);
                break;
            case OK:
                s1.setImageResource(R.drawable.pressed_app_s1_ok);
                break;
        }
    }
}

