#include "PhvControl.h"
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;
using namespace zmp::hev;

#define USE_DEMO 1

struct BattInf {
    float current;
    float soc;
    float voltage;
    float subVoltage;
};

struct BrakeInf {
    float actualPedalStr;		// ペダルストローク現在値
    float targetPedalStr;		// ペダルストローク目標値
    float inputPedalStr;		// ペダルストローク入力値
    short sks;
    short sla;
    short regPress;
    short wheelPress;
    short slr;
    short src;
    short sksT;
    short regT;
};

struct OtherInf {
//    short accX;
//    short accY;
//    short accZ;
//    unsigned char inclino;
    unsigned int odometry;
    float velocFrFromP;		// 右前輪速度[km/h]
    float velocFlFromP;		// 左前輪速度[km/h]
    float velocRrFromP;		// 右後輪速度[km/h]
    float velocRlFromP;		// 左後輪速度[km/h]
    float velocFromP;      // 速度
    float engineRpm;       // エンジンの回転数
    float motorRpm;        // エンジンの回転数
//    unsigned char ice;
    float coolantTemp;     // 温度[℃ ]
    int shiftFromPrius;     // シフト状態
    float drvPedalStrFromP;  // アクセルペダルストローク
    float brkPedalStrFromP;// ブレーキペダルストローク
    bool brkState;          // ブレーキランプON/OFF
    float angleFromP;       // ステアリング角度
    int drv_mode;           // ドライブモード
    bool ecoMode;           // ECOモード
    int ev_mode;            // EVモード
    LIGHT_STATE light;		// ライト状態
    unsigned char diagData[8];  // diagデータ
    DOOR_OPEN_STATE doorOpenDr;     // 運転席ドア開閉状態
    DOOR_OPEN_STATE doorOpenPass;   // 助手席ドア開閉状態
    DOOR_OPEN_STATE doorOpenRr;     // 右後部座席ドア開閉状態
    DOOR_OPEN_STATE doorOpenRl;     // 左後部座席ドア開閉状態
    DOOR_OPEN_STATE doorOpenTrunk;  // トランク開閉状態
    DOOR_LOCK_STATE doorLockDr;     // 運転席ロック状態
    DOOR_LOCK_STATE doorLockPass;   // 助手席ロック状態
    DOOR_LOCK_STATE doorLockRr;     // 右後部座席ロック状態
    DOOR_LOCK_STATE doorLockRl;     // 左後部座席ロック状態
    DOOR_LOCK_STATE doorLockTrunk;  // トランクロック状態
    DOOR_WINDOW_STATE windowDr;     // 運転席ウィンドウ開閉状態
    DOOR_WINDOW_STATE windowPass;   // 助手席ウィンドウ開閉状態
    DOOR_WINDOW_STATE windowRr;     // 右後部座席ウィンドウ開閉状態
    DOOR_WINDOW_STATE windowRl;     // 左後部座席ウィンドウ開閉状態
//    DOOR_WINDOW_STATE windowTrunk;  //
};

struct DrvInf {
    PHV_MODE mode;               // ドライブモード(manual=0x00, program=0x10)
    DRIVE_CONTROL_MODE contMode; // ドライブ制御モード(velocity=0x00, pedal=0x10)
    OVERRIDE_MODE overrideMode;	 // オーバーライドモード(ON=0x00, OFF=0x10)
    SERVO_MODE servo;            // 制御のON/OFF(ON=true, OFF=false)
    float actualPedalStr;		// ペダルストローク現在値
    float targetPedalStr;		// ペダルストローク目標値
    float inputPedalStr;		// ペダルストローク入力値
    short vpa1;                 // VPA1ペダルセンサ値
    short vpa2;                 // VPA2ペダルセンサ値
    float targetVeloc;          // 目標速度[km/h]
    float actualVeloc;          // 現在速度[km/h]
    SHIFT_POSITION actualShift; // シフトポジション現在値
    SHIFT_POSITION targetShift;	// シフトポジション目標値
    SHIFT_POSITION inputShift;  // シフトポジション入力値
    short shiftRawVsx1;       // シフトRaw値VSX1
    short shiftRawVsx2;       // シフトRaw値VSX2
    short shiftRawVsx3;       // シフトRaw値VSX3
    short shiftRawVsx4;       // シフトRaw値VSX4
};

struct StrInf {
    PHV_MODE mode;                  // ステアリングモード(manual=0x00, program=0x10)
    STEER_CONTROL_MODE cont_mode;   // ステアリング制御モード(torque=0x00, angle=0x10)
    OVERRIDE_MODE overrideMode;     // オーバーライドモード(ON=0x00, OFF=0x10)
    SERVO_MODE servo;               // 制御のON/OFF(ON=true, OFF=false)
    float targetTorque;            // 目標トルク
    float actualTorque;            // 現在トルク
    short trq1;                    // TRQ1トルクセンサ
    short trq2;                    // TRQ2トルクセンサ
    float actualAngle;             // 操舵角度[deg]
    float targetAngle;             // 目標操舵角[deg]
};

struct ImuInf{
    double accX;
    double accY;
    double accZ;

    double gyroX;
    double gyroY;
    double gyroZ;

    double compX;
    double compY;
    double compZ;

    IMUZ2_ROLE role;
    int period;
    zmp::hev::RANGE_ACC rangeAcc;
    zmp::hev::RANGE_GYRO rangeGyro;
    zmp::hev::RANGE_COMP rangeComp;
    float battLevel;
    IMUZ2_FORMAT format;

    unsigned char hardVersion;
    unsigned char firmVersion;
};

struct IncInf{
    double accX;
    double accY;
    double angleX;
    double angleY;

    short period;
    float battLevel;
    short mode;
};

struct PosInf{
    GPGGA_DATA gga;

    float humid;
    float hTemp;

    float press;
    float pTemp;
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

class PhvCnt : public ChangeStateObserver
{
public:
    PhvCnt();
    virtual ~PhvCnt();

    void ClearCntDiag(); // Autoware Extension

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
    void GetPosInf(PosInf* pos);
    void GetIncInf(IncInf* inc);
    void GetImuInf(ImuInf* imu);


    // Set Steer
    void SetStrMode(PHV_MODE mode);
    void SetStrCMode(STEER_CONTROL_MODE cmode);
    void SetStrOMOde(OVERRIDE_MODE omode);
    void SetStrTorque(float torque);
    void SetStrAngle(short angle);
    void SetStrServo(SERVO_MODE servo);

    // Set Drive
    void SetDrvMode(PHV_MODE mode);
    void SetDrvCMode(DRIVE_CONTROL_MODE cmode);
    void SetDrvOMode(OVERRIDE_MODE omode);
    void SetDrvStroke(float stroke);
    void SetDrvVeloc(float veloc);
    void SetDrvShiftMode(SHIFT_POSITION shift);
    void SetDrvServo(SERVO_MODE servo);

    // Set Brake
    void SetBrakeStroke(float stroke);
    void SndBrakeLamp(unsigned char lamp);
    void SndLeftBlinker(unsigned char left);
    void SndRightBlinker(unsigned char right);
    void SndBrakeAutoLamp(unsigned char autoMode);

    // Set Other
    void SetControlGain(int index, int gain);
    void SetDist(float dist);
    void SetTargetAngle(int target);
    void SndDiagReq(PHV_ECU kind);
    void SndDiagClear(PHV_ECU kind);
//    void SndErrReq();

    void GetConfig(PHV_CONFIG kind);
    void SetConfig(PHV_CONFIG kind, int val);
    void SaveConfig();

private:
    void UpdateSteerState(REP_STEER_INFO_INDEX index);
    void UpdateDriveState(REP_DRIVE_INFO_INDEX index);
    void UpdateBrakeState(REP_BRAKE_INFO_INDEX index);
    void UpdateBattState(REP_BATT_INFO_INDEX index);
    void UpdateOtherState(REP_OTHER_INFO_INDEX index);
    void ReceiveError(int error_code);
    void ReceiveConfig(int num, int index, int value[]);
    void ReceiveErrorStatus(int leve, int errCode);
    void ReceiveEcho(int ctlKind, int ctlNo);
    void ReceiveImuMsg(REP_IMUZ2_INFO_INDEX index);
    void UpdatePoszState(REP_POSZ_INFO_INDEX index);
    void UpdateInczState(REP_INCZ_INFO_INDEX index);



    void rcvTime(char* date);
    void getDate();

    PhvControl* _phvCnt;
    CanCommunication* _canCom;
    ChangeConfig* _callback;
    CarTomo*     _carTomo;

    int _errCode;
    int _errLevel;
    BattInf _battInf;
    BrakeInf _brakeInf;
    OtherInf _otherInf;
    DrvInf _drvInf;
    StrInf _strInf;
    int _temp[100];
    ConfigInf _config;
    ImuInf  _imuInf;
    IncInf  _incInf;
    PosInf  _posInf;

    int _beforAngle;
    int _targetCnt;
    int _targetAngle;

    selectLogInf _selectLog;
    FILE* _logSave;
    struct tm *_s_time;
    time_t _day_time;
    timeval _getTime;

    float _asistTrq;
};

