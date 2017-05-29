package com.client;

import android.content.Context;
import android.content.Intent;

/**
 * Created by linna on 11/13/2016.
 */
public class CanDataSender {
    private boolean isRunning = false;

    private String table;
    private String terminal;
    private String str;
    private String sp;
    private String sh;
    private String su;
    private String spss;
    private String lp;
    private String fh;
    private String fp;
    private Intent intent;

    private Context mContext;

    public CanDataSender(Context context, String table, String terminal, String str, String sp, String sh,
                         String su, String spss, String lp, String fh, String fp) {
        mContext = context;
        this.table = table;
        this.terminal = terminal;
        this.str = str;
        this.sp = sp;
        this.sh = sh;
        this.su = su;
        this.spss = spss;
        this.lp = lp;
        this.fh = fh;
        this.fp = fp;

        intent = new Intent(Intent.ACTION_MAIN);
        intent.setClassName("com.example.candatasender",
                "com.example.candatasender.service.CanDataAutoSend");
    }

    public boolean isRunning() {
        return isRunning;
    }

    public void start() {
        intent.putExtra("table", table);
        intent.putExtra("terminal", terminal);
        intent.putExtra("pfd", true);
        intent.putExtra("str", str);
        intent.putExtra("sp", sp);
        intent.putExtra("sh", sh);
        intent.putExtra("su", su);
        intent.putExtra("spss", spss);
        intent.putExtra("lp", lp);
        intent.putExtra("fh", fh);
        intent.putExtra("fp", fp);

       mContext.startService(intent);
        isRunning = true;
    }

    public void stop() {
        isRunning = false;
        mContext.stopService(intent);
    }

    public String getTable() {
        return table;
    }

    public String getTerminal() {
        return terminal;
    }

    public String getStr() {
        return str;
    }

    public String getSp() {
        return sp;
    }

    public String getSh() {
        return sh;
    }

    public String getSu() {
        return su;
    }

    public String getSpss() {
        return spss;
    }

    public String getLp() {
        return lp;
    }

    public String getFh() {
        return fh;
    }

    public String getFp() {
        return fp;
    }
}

