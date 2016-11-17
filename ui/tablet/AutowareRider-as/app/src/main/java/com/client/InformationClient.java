package com.client;

/**
 * Created by linna on 11/13/2016.
 */
public class InformationClient extends Client {
    public static final int BEACON = 0;
    public static final int ERROR = 1;
    public static final int CAN = 2;
    public static final int MODE = 3;
    public static final int NDT = 4;
    public static final int LF = 5;

    public static final int MISS_BEACON_LIMIT = 10;

    public static final int CAN_SHIFT_BRAKE = 0x00;
    public static final int CAN_SHIFT_DRIVE = 0x10;
    public static final int CAN_SHIFT_NEUTRAL = 0x20;
    public static final int CAN_SHIFT_REVERSE = 0x40;

    public synchronized int[] recv(int response) {
        int[] data = new int[2];

        if (isClosed()) {
            data[0] = -1;
            data[1] = -1;
            return data;
        }

        data[0] = recvInt();
        if (data[0] < 0) {
            data[1] = -1;
            return data;
        }

        if (data[0] == NDT)
            data[1] = recvNDT();
        else
            data[1] = recvInt();
        if (data[1] < 0)
            return data;

        sendInt(response);

        return data;
    }
}
