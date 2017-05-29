package com.client;

/**
 * Created by linna on 11/13/2016.
 */
public class CommandClient extends Client {
    public static final int EXIT = 0;
    public static final int GEAR = 1;
    public static final int MODE = 2;
    public static final int ROUTE = 3;
    public static final int S1 = 4;
    public static final int S2 = 5;
    public static final int POSE = 6;

    public static final int EXIT_DESTROY_ACTIVITY = 0;
    public static final int EXIT_UPDATE_CONFIGURATION = 1;
    public static final int EXIT_EXCEED_ERROR_LIMIT = -1;
    public static final int EXIT_RECEIVE_BROKEN_PACKET = -2;

    public synchronized int send(int type, int command) {
        if (isClosed())
            return -1;

        sendIntTuple(type, command);

        return recvInt();
    }

    public synchronized int sendRoute(double[] latlong) {
        if (isClosed())
            return -1;

        sendIntTuple(ROUTE, latlong.length * (Double.SIZE / 8));

        sendDoubleArray(latlong);

        return recvInt();
    }

    public synchronized int sendPose(double[] pose) {
        if (isClosed())
            return -1;

        sendIntTuple(POSE, pose.length * (Double.SIZE / 8));

        sendDoubleArray(pose);

        return recvInt();
    }
}

