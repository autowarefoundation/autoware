package jp.tier4.autowaredrive;

/**
 * Created by yuki.iida on 2017/06/28.
 */

public class CanInfo {
    private final static String TAG = "CanInfo";

    private static CanInfo instance = new CanInfo();

    public String tm;
    public float devmode;
    public float drvcontmode;
    public float drvoverridemode;
    public float drvservo;
    public float drivepedal;
    public float targetpedalstr;
    public float inputpedalstr;
    public float targetveloc;
    public float speed;
    public float driveshift;
    public float targetshift;
    public float inputshift;
    public float strmode;
    public float strcontmode;
    public float stroverridemode;
    public float strservo;
    public float targettorque;
    public float torque;
    public float angle;
    public float targetangle;
    public float bbrakepress;
    public float brakepedal;
    public float brtargetpedalstr;
    public float brinputpedalstr;
    public float battery;
    public float voltage;
    public float anp;
    public float battmaxtemparature;
    public float battmintemparature;
    public float maxchgcurrent;
    public float maxdischgcurrent;
    public float sideacc;
    public float accellfromp;
    public float anglefromp;
    public float brakepedalfromp;
    public float speedfr;
    public float speedfl;
    public float speedrr;
    public float speedrl;
    public float velocfromp2;
    public float drvmode;
    public float devpedalstrfromp;
    public float rpm;
    public float velocflfromp;
    public float ev_mode;
    public float temp;
    public float shiftfrmprius;
    public float light;
    public float gaslevel;
    public float door;
    public float cluise;

    private CanInfo(){
    }

    public static CanInfo getInstance() {
        return instance;
    }

    public void parseCanInfo(String message) {

        String[] canInfo = message.split(",", -1);

        tm = canInfo[0];
        devmode = Float.parseFloat(canInfo[1]);
        drvcontmode = Float.parseFloat(canInfo[2]);
        drvoverridemode = Float.parseFloat(canInfo[3]);
        drvservo = Float.parseFloat(canInfo[4]);
        drivepedal = Float.parseFloat(canInfo[5]);
        targetpedalstr = Float.parseFloat(canInfo[6]);
        inputpedalstr = Float.parseFloat(canInfo[7]);
        targetveloc = Float.parseFloat(canInfo[8]);
        speed = Float.parseFloat(canInfo[9]);
        driveshift = Float.parseFloat(canInfo[10]);
        targetshift = Float.parseFloat(canInfo[11]);
        inputshift = Float.parseFloat(canInfo[12]);
        strmode = Float.parseFloat(canInfo[13]);
        strcontmode = Float.parseFloat(canInfo[14]);
        stroverridemode = Float.parseFloat(canInfo[15]);
        strservo = Float.parseFloat(canInfo[16]);
        targettorque = Float.parseFloat(canInfo[17]);
        torque = Float.parseFloat(canInfo[18]);
        angle = Float.parseFloat(canInfo[19]);
        targetangle = Float.parseFloat(canInfo[20]);
        bbrakepress = Float.parseFloat(canInfo[21]);
        brakepedal = Float.parseFloat(canInfo[22]);
        brtargetpedalstr = Float.parseFloat(canInfo[23]);
        brinputpedalstr = Float.parseFloat(canInfo[24]);
        battery = Float.parseFloat(canInfo[25]);
        voltage = Float.parseFloat(canInfo[26]);
        anp = Float.parseFloat(canInfo[27]);
        battmaxtemparature = Float.parseFloat(canInfo[28]);
        battmintemparature = Float.parseFloat(canInfo[29]);
        maxchgcurrent = Float.parseFloat(canInfo[30]);
        maxdischgcurrent = Float.parseFloat(canInfo[31]);
        sideacc = Float.parseFloat(canInfo[32]);
        accellfromp = Float.parseFloat(canInfo[33]);
        anglefromp = Float.parseFloat(canInfo[34]);
        brakepedalfromp = Float.parseFloat(canInfo[35]);
        speedfr = Float.parseFloat(canInfo[36]);
        speedfl = Float.parseFloat(canInfo[37]);
        speedrr = Float.parseFloat(canInfo[38]);
        speedrl = Float.parseFloat(canInfo[39]);
        velocfromp2 = Float.parseFloat(canInfo[40]);
        drvmode = Float.parseFloat(canInfo[41]);
        devpedalstrfromp = Float.parseFloat(canInfo[42]);
        rpm = Float.parseFloat(canInfo[43]);
        velocflfromp = Float.parseFloat(canInfo[44]);
        ev_mode = Float.parseFloat(canInfo[45]);
        temp = Float.parseFloat(canInfo[46]);
        shiftfrmprius = Float.parseFloat(canInfo[47]);
        light = Float.parseFloat(canInfo[48]);
        gaslevel = Float.parseFloat(canInfo[49]);
        door = Float.parseFloat(canInfo[50]);
        cluise = Float.parseFloat(canInfo[51]);
    }
}
