package com.ui;

import android.widget.ImageButton;

import com.ghostagent.R;
import com.ghostagent.SoundManagementActivity;

/**
 * Created by linna on 11/13/2016.
 */
public class ApplicationButton extends RadioButton {
    public static final int NAVIGATION = 1;
    public static final int MAP = 2;

    public ImageButton navigation;
    public ImageButton map;

    public ApplicationButton(SoundManagementActivity activity) {
        super(activity);
        navigation = (ImageButton)activity.findViewById(R.id.air);
        navigation.setOnClickListener(activity);
        map = (ImageButton)activity.findViewById(R.id.oil);
        map.setOnClickListener(activity);

        refresh();
    }

    @Override
    public void refresh() {
        navigation.setImageResource(R.drawable.app_navi);
        map.setImageResource(R.drawable.app_map);

        switch (getMode()) {
            case NAVIGATION:
                navigation.setImageResource(R.drawable.pressed_app_navi);
                break;
            case MAP:
                map.setImageResource(R.drawable.pressed_app_map);
                break;
        }
    }
}

