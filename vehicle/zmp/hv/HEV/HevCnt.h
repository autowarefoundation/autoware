#include "HevControl.h"
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;
using namespace zmp::hev;

#define USE_DEMO 1

struct BattInf {
    float current;              // 0x3B 充放電電流[A*10]
    float max_dischg_current;	// 0x3CB 最大放電電流[A*10]
    float max_chg_current;      // 0x3CB 最大充電電流[A*10]
    float soc;                  // 0x3CB 残容量[%]
    int min_temp;               // 0x3CB 最低温度[℃ ]
    int max_temp;               // 0x3CB 最高温度[℃ ]
    int voltage;                // 0x3CD バッテリ電圧[V]
};

struct BrakeInf {
    bool pressed;           // ペダルスイッチ状態(ON=true, OFF=false)
    int actualPedalStr;		// ペダルストローク現在値
    int targetPedalStr;		// ペダルストローク目標値
    int inputPedalStr;		// ペダルストローク入力値
    float prl;              // PRLセンサ値
    float pfl;              // PFLセンサ値
    float prr;              // PRRセンサ値
    float pfr;              // PFRセンサ値
    float sks1;             // SKS1ペダルセンサ値
    float sks2;             // SKS2ペダルセンサ値
    float pmc1;             // PMC1ペダルセンサ値
    float pmc2;             // PMC2ペダルセンサ値
//    int targetStr;		// 目標ストローク
};

struct OtherInf {
    float sideAcc;          // 0x22 Yawレート
    float acc;              // 0x23 前後加速度
    float angleFromP;       // 0x25 ステアリング角度
    float brkPedalStrFromP;// 0x30 ブレーキペダル状態
    float velocFrFromP;		// 0xB1 右前輪速度[km/h*100]
    float velocFlFromP;		// 0xB1 左前輪速度[km/h*100]
    float velocRrFromP;		// 0xB3 右後輪速度[km/h*100]
    float velocRlFromP;		// 0xB3 左後輪速度[km/h*100]
    float velocFromP2;      // 0xB4 速度
    int drv_mode;           // 0x120 ドライブモード
    int drvPedalStrFromP;  // 0x244 アクセルペダル状態
    int rpm;                // 0x3C8 エンジンの回転数
    float velocFromP;       // 0x3CA 速度
    int ev_mode;            // 0x529 EVモード
    int temp;               // 0x52C 温度[℃ ]
    int shiftFromPrius;     // 0x540 シフト状態
    LIGHT_STATE light;		// 0x57F ライト状態
    int level;              // 0x5A4 燃料の残量
    DOOR_STATE door;		// 0x5B6 ドアの状態
    bool cluise;            // 0x5C8 クルーズコントロールON/OFF
    char dtcData1;          // 0x7E8,0x7EA,0x7EB
    char dtcData2;          // 0x7E8,0x7EA,0x7EB
    char dtcData3;          // 0x7E8,0x7EA,0x7EB
    char dtcData4;          // 0x7E8,0x7EA,0x7EB
    char dtcData5;          // 0x7E8,0x7EA,0x7EB
    char dtcData6;          // 0x7E8,0x7EA,0x7EB
    char dtcData7;          // 0x7E8,0x7EA,0x7EB
    char dtcData8;          // 0x7E8,0x7EA,0x7EB
};

struct DrvInf {
    int mode;               // ドライブモード(manual=0x00, program=0x10)
    int contMode;           // ドライブ制御モード(velocity=0x00, stroke=0x10)
    int overrideMode;		// オーバーライドモード(ON=0x00, OFF=0x10)
    int servo;              // 制御のON/OFF(ON=true, OFF=false)
    int actualPedalStr;		// ペダルストローク現在値
    int targetPedalStr;		// ペダルストローク目標値
    int inputPedalStr;		// ペダルストローク入力値
    float vpa1;             // VPA1ペダルセンサ値
    float vpa2;             // VPA2ペダルセンサ値
    float targetVeloc;		// 目標速度[km/h*100]
    float veloc;            // 現在速度[km/h*100]
    int actualShift;        // シフトポジション現在値
    int targetShift;		// シフトポジション目標値
    int inputShift;         // シフトポジション入力値
//    int shiftFromPrius;
    int shiftRawVsx1;       // シフトRaw値VSX1
    int shiftRawVsx2;       // シフトRaw値VSX2
    int shiftRawVsx3;       // シフトRaw値VSX3
    int shiftRawVsx4;       // シフトRaw値VSX4
//    int spls_sp1;		// 車速パルス
//    int spls_fr;		// 右フロント車速パルス
//    int spls_fl;		// 左フロント車速パルス
//    int spls_rr;		// 右リア車速パルス
//    int spls_rl;		// 左リア車速パルス
//    int targetStr;		// 目標ストローク
};

struct StrInf {
    int mode;           // ステアリングモード(manual=0x00, program=0x10)
    int cont_mode;      // ステアリング制御モード(torque=0x00, angle=0x10)
    int overrideMode;   // オーバーライドモード(ON=0x00, OFF=0x10)
    int servo;          // 制御のON/OFF(ON=true, OFF=false)
    int targetTorque;   // 目標トルク
    int torque;         // 操舵トルク
    float trq1;         // TRQ1トルクセンサ
    float trq2;         // TRQ2トルクセンサ
    float angle;        // 操舵角度[deg * 10]
    float targetAngle;  // 目標操舵角[deg*10]
};

struct AccInf{
    unsigned short time;
    short accX;
    short accY;
    short accZ;
};

struct GyroInf{
    unsigned short time;
    short gyroX;
    short gyroY;
    short gyroZ;
};

struct CompInf{
    unsigned short time;
    short compX;
    short compY;
    short compZ;
};

struct SensorInf {
    int ofz[4];
    float seat;
    int cntS[5];
    float distance;
};

struct ConfigInf {
    int data[21];
};

struct selectLogInf {
    bool start;
    bool drvInf;
    bool strInf;
    bool brkInf;
    bool battInf;
    bool otherInf;
};

class ChangeConfig
{
public:
    virtual void UpdateConfig(int num, int index, int data[]) = 0;
};

class HevCnt : public ChangeStateObserver
{
public:
    HevCnt();
    virtual ~HevCnt();

    bool Init();
    bool Start();
    bool Process();
    bool Stop();
    bool Close();
    bool SetConfigCallback(ChangeConfig* callback);

    void GetBattInf(BattInf* batt);
    void GetBrakeInf(BrakeInf* brake);
    void GetOtherInf(OtherInf* other);
    void GetDrvInf(DrvInf* drv);
    void GetStrInf(StrInf* str);
    void GetErrCode(int* leve, int* err);
//    void GetSensInf(SensorInf* sens);

    // Set Steer
    void SetStrMode(int mode);
    void SetStrCMode(int cmode);
    void SetStrOMOde(int omode);
    void SetStrTorque(int torque);
    void SetStrAngle(int angle);
    void SetStrServo(int servo);

    // Set Drive
    void SetDrvMode(int mode);
    void SetDrvCMode(int cmode);
    void SetDrvOMode(int omode);
    void SetDrvStroke(int stroke);
    void SetDrvVeloc(int veloc);
    void SetDrvShiftMode(int shift);
    void SetDrvServo(int servo);

    // Set Brake
    void SetBrakeStroke(int stroke);

    // Set Other
    void SetControlGain(int index, int gain);
    void SetDist(float dist);
    void SetTargetAngle(int target);
    void SndDiagReq(HEV_ECU kind);
    void SndDiagClear(HEV_ECU kind);
    void SndErrReq();

    void GetConfig(HEV_CONFIG kind);
    void SetConfig(HEV_CONFIG kind, int val);
    void SetConfig(HEV_CONFIG kind, float val);
    void SaveConfig();

   // Set log
//    void SetlogEnable(selectLogInf select);

private:
    void UpdateSteerState(REP_STEER_INFO_INDEX index);
    void UpdateDriveState(REP_DRIVE_INFO_INDEX index);
    void UpdateBattState(REP_BATT_INFO_INDEX index);
    void UpdateOtherState(REP_OTHER_INFO_INDEX index);
    void UpdateDemoSensorState(REP_DEMO_SENSOR_INFO_INDEX index);
    void ReceiveConfig(int num, int index, int value[]);
    void ReceiveErrorStatus(int leve, int errCode);
    void ReceiveEcho(int ctlKind, int ctlNo);
    void ReceiveImuMsg(REP_IMU_INFO_INDEX index);



    void rcvTime(char* date);
    void getDate();

    void SetSeat(int seat);
    void SetOfz(int index, int val);
    void SetCnts(int index);

    int GetSeat();
    int GetOfz(int index);
    int GetCnts(int index);
    float GetDist();

    HevControl* _hevCnt;
    CanCommunication* _canCom;
    ChangeConfig* _callback;

    int _errCode;
    int _errLevel;
    BattInf _battInf;
    BrakeInf _brakeInf;
    OtherInf _otherInf;
    DrvInf _drvInf;
    StrInf _strInf;
    int _temp[100];
    SensorInf _sensInf;
    ConfigInf _config;

    int _beforAngle;
    int _targetCnt;
    int _targetAngle;

    selectLogInf _selectLog;
    FILE* _logSave;
    struct tm *_s_time;
    time_t _day_time;
    timeval _getTime;

    int _asistTrq;
};

