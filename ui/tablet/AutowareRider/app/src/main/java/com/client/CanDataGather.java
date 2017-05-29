package com.client;

import android.content.Context;
import android.content.Intent;

/**
 * Created by linna on 11/13/2016.
 */
public class CanDataGather {
    public static final int CAN_GATHER = 0;
    public static final int CAR_LINK_BLUETOOTH = 1;
    public static final int CAR_LINK_USB = 2;

    private Intent intent;
    private Context mContext;

    public CanDataGather(Context context, int type) {
        mContext = context;
        intent = new Intent(Intent.ACTION_MAIN);
        switch (type) {
            case CAN_GATHER:
                intent.setClassName("com.ecsgr.android.cangather",
                        "com.ecsgr.android.cangather.MainActivity");
                break;
            case CAR_LINK_BLUETOOTH:
                intent.setClassName("com.metaprotocol.android.carlinkcan232",
                        "com.metaprotocol.android.carlinkcan232.CarLinkMainActivity");
                break;
            case CAR_LINK_USB:
                intent.setClassName("com.metaprotocol.android.carlinkcanusbaccessory",
                        "com.metaprotocol.android.carlinkcanusbaccessory.CarLinkMainActivity");
                break;
        }
    }

    public void start() {
        mContext.startActivity(intent);
    }
}

