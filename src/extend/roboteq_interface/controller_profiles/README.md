## デフォルトからの変更点

### Inputs/Outputs

- Telemetry String を以下に設定
  - `?BS 1:?FF:?V 2:?D:?DO:?AI 1:# 3`
- Commands / Command Safety
  - RS232 Safety / Watchdog (ms) : `200`<sup>(\*1)</sup>
  - Analog Safety / Keep within Guard Bands : `Disabled`<sup>(\*1)</sup>
- CAN<sup>(\*2)</sup>
  - Bit Rate : `500`
  - Node ID : 左モーターは`1`、右は`2`
  - Heartbeat (ms) : `100`
  - CANOpen TPDO Send Rate
    - TPDO 1 Send Rate (ms) : `20`
    - TPDO 2 Send Rate (ms) : `100`
    - TPDO 3 Send Rate (ms) : `100`
    - TPDO 4 Send Rate (ms) : `100`
  - CANOpen Autostart : `Enabled`
- Digital inputs / DIn1 を緊急停止スイッチ用に設定
  - Active Level : `Low`
  - Action : `Dead man Switch`
- Digital Outputs / DOut1 をブレーキ用に設定
  - Active Level : `High`
  - Active When : `Never`
- Digital Outputs / DOut2 をウィンカー用に設定
  - Active Level : `High`
  - Active When : `Never`

### Power Output

- Motor / Motor Configuration
  - Motor Direction : `Inverted`
    - v1α 構成の場合は右だけ `Direct`
  - Number of Pole Pairs : `15`
  - Switching Mode : `Sinusoidal`
  - Reference Seek Power (A) : `3.5`<sup>(\*1)</sup>
  - Sinusoidal settings
    - SinCos/SSI Sensor Poles : `30`
    - FOC Parameters
      - Flux Proportional Gain : `0.003`<sup>(\*1)</sup>
      - Flux Integral Gain : `0.3`<sup>(\*1)</sup>
    - Hall Sensor Angle Table
      - Motor/Sensor Setup による自動調整
- Motor / Motor Output
  - Amps Limits
    - Amps Limit (A) : `25.0`<sup>(\*1)</sup>
    - AmpsTrigger Action : `Emergency Stop`<sup>(\*1)</sup>
  - Speed & Acceleration
    - Operating Mode : `Closed Loop Speed`
  - Closed Loop Parameters
    - Position Turns Min to Max : `444.44`<sup>(\*1)</sup>
    - Proportional Gain : `1.3`
    - Integral Gain : `13.0`
    - Integrator Limit(%) : `100`

\*1 : KHI より送付されたパラメータファイルに設定されていた値

\*2 : CAN 通信用に設定していたパラメータ（シリアル通信時は無関係）

---

## 車輪回転数等の出力周期

出力周期 [Hz] = 1 / (TELS コマンド数 \* interval [ms])

現状 Telemetry String に設定している以下のコマンドの場合、1/(7\*0.003)≒47.6[Hz]

`?CB 1:?BS 1:?FF:?V 2:?D:?DO:?AI 1:# 3`

TELS コマンドについて

- コロン区切りでコマンドと interval を設定
- `#(スペース)数値` で interval[ms]を設定
- interval[ms]毎に 1 コマンドずつ処理されるため、出力周期はコマンド数によっても変化する
- 設定可能なクエリコマンドは公式マニュアルを参照、以下現状のコマンド内容
  - CB : ホールセンサのカウント値
  - BS : モーターの回転速度[RPM]
  - FF : Fault Flag（Emergency 等のエラー情報）
  - V : バッテリー電圧値
  - D : デジタルインプットピンの High/Low 状態
  - DO : デジタルアウトプットピンの High/Low 状態
  - AI : アナログインプットピンの電圧値
