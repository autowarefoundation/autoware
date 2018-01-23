package jp.tier4.autowaredrive;

/**
 * Created by yuki.iida on 2017/06/20.
 */

public class ControlComand {
    private final static String TAG = "ControlComand";

    private static ControlComand instance = new ControlComand();

    /*** Control Cmd ***/
    float accelCmd = 0;
    float brakeCmd = 0;
    float steeringCmd = 0;
    int gearCmd = 1; // D:1, R:2, B:0, N:4
    int modeCmd = 0;
    int emergencyCmd = 0;

    private ControlComand(){
    }

    public static ControlComand getInstance() {
        return instance;
    }

}
