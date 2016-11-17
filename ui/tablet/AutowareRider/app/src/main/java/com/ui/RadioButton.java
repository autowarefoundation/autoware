package com.ui;

import android.app.Activity;

/**
 * Created by linna on 11/13/2016.
 */
public abstract class RadioButton {
    static final int INVALID = -1;

    private int mode = INVALID;
    protected Activity mActivity;

    RadioButton(Activity activity) {
        mActivity = activity;
    }
    public int getMode() {
        return mode;
    }

    public void updateMode(int mode) {
        if (mode == this.mode)
            mode = INVALID;
        this.mode = mode;
        refresh();
    }

    abstract public void refresh();
}
