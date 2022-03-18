# Obstacle Avoidance Planner

## Purpose

obstacle_avoidance_planner は入力された path と drivable area、および動物体情報をもとに、車両キネマティクスモデルを考慮して車が走行可能な軌道を生成する。trajectory 内の各経路点の位置姿勢のみ計画しており、速度や加速度の計算は後段の別モジュールで行われる。

<!-- ここで計画されたtrajectoryが、後段の経路追従の制御であるmpc_followerで追従可能であることを想定している。??? -->

## Inputs / Outputs

### input

- reference_path [`autoware_auto_planning_msgs/Path`] : Reference path and the corresponding drivable area.
- objects [`autoware_auto_perception_msgs/PredictedObjects`] : Recognized objects around the vehicle

### output

- optimized_trajectory [`autoware_auto_planning_msgs/Trajectory`] : Optimized trajectory that is feasible to drive and collision-free.

## Flowchart

フローチャートとともに、各機能の概要をおおまかに説明する。

![obstacle_avoidance_planner_flowchart](./media/obstacle_avoidance_planner_flowchart.drawio.svg)

### Manage trajectory generation

以下の条件のいずれかが満たされたときに、経路生成の関数を呼ぶ。それ以外の時は前回生成された経路を出力する。

- 前回経路生成時から一定距離走行 (default: 10.0 [m])
- 一定時間経過 (default: 1.0 [s])
- レーンチェンジなどで入力の path の形状が変わった時
- 自車位置が前回経路から大きく外れた時

入力の path の形状が変わった時と自車が前回経路から大きく外れた時は、保持している前回経路を破棄するリセット機能もある。

### Select objects to avoid

静的で路肩にある障害物のみ回避対象とする。
具体的には、以下の 3 つの条件を満たすものであり、図で示すと赤い id: 3, 4, 5 の物体である。

- 指定された速度以下 (default: 0.1 [m/s])
- 物体重心が center line 上に存在しない（前車追従の車を避けないようにするため）
- 少なくとも 1 つ以上の物体形状ポリゴン点が drivable area に存在する。

![obstacle_to_avoid](./media/obstacle_to_avoid.drawio.svg)

### Generate object costmap

回避対象である障害物からの距離に応じたコストマップを生成する。

### Generate road boundary costmap

道路からの距離に応じたコストマップを生成する。

これらのコストマップは後段の Optimize trajectory の手法である Model predictive trajectory の障害物・道路境界に衝突しない制約条件を定式化する際に使用される。

### Smooth path

後段の最適化処理で曲率のなめらかな参照経路が必要であるため、最適化前に path をなめらかにして trajectory の形式で出力する。
この平滑化は障害物を考慮しない。

### Optimize trajectory

参照経路に基づいたフレネ座標系で車両キネマティクス、及び追従誤差を定義し、二次計画法を用いて車両キネマティクスや障害物回避を考慮しながら追従誤差が小さくなるように経路生成する。
自車近傍の経路の急な変化を防ぐため、直近の経路は前回の経路をそのまま使用する。
計算コストを抑えるため、最終的に出力する経路長よりも短い距離に対してのみ計算を行う。

### Extend trajectory

経路を規定の長さ（default: 200m）に拡張するため、最適化した経路の先を Reference 経路と接続する。

### Check drivable area

生成された経路が走行可能領域を逸脱していた場合、前回生成された走行可能領域内にある経路を出力する。

_Rationale_
現在の設計において、数値誤差による破綻を防ぐために障害物回避は全てソフト制約として考慮されており、生成された経路に置いて車両が走行可能領域内に入っているかの判定が必要。

### Apply path velocity

入力の path に埋め込まれている速度を出力される trajectory に埋め込む。経路間隔が異なるため線形補間を用いる。

## Algorithms

Smooth path で使われている Elastic band と、Optimized trajectory で使われている Model predictive trajectory の詳細な説明をする。

### Elastic band

#### 概要

behavior_path_planner で計算された path は場合によってはなめらかではない可能性があるので、その path の平滑化をすることが目的である。

次の Model predictive trajectory でも平滑化項は入っているが、目標経路になるべく追従しようとする項も入っているため、目標経路がなめらかでなかった場合はこの 2 つの項が反発する。
それを防ぐためにここで平滑化のみを行っている。
また Model predictive trajectory では各点における曲率と法線を元に最適化しており、平滑化された目標経路を渡すことで最適化の結果を安定させる狙いもある。

平滑化の際、障害物や道路壁を考慮していないため障害物や道路壁に衝突した trajectory が計算されることもある。

この Elastic band と次の Model predictive trajectory は、計算負荷を抑えるためにある程度の長さでクリップした trajectory を出力するようになっている。

#### 数式

前後の点からの距離の差の二乗を目的関数とする二次計画。

各点は一定の範囲以内しか動かないという制約を設けることで、入力の軌道をよりなめらかにした軌道を得る。

$\boldsymbol{p}_k$を$k$番目の経路点の座標ととしたとき以下のコスト関数を最小化する二次計画を解く。ここでは始点と終点である$\boldsymbol{p}_0$と$\boldsymbol{p}_n$は固定である。

$$
\min \sum_{k=1}^{n-1} |\boldsymbol{p}_{k+1} - \boldsymbol{p}_{k}| - |\boldsymbol{p}_{k} - \boldsymbol{p}_{k-1}|
$$

### Model predictive trajectory

#### 概要

Elastic band で平滑化された trajectory に対して、以下の条件を満たすように修正を行うことが目的である。

- 線形化された車両のキネマティクスモデルに基づき走行可能である
- 障害物や道路壁面に衝突しない

障害物や道路壁面に衝突しない条件はハードではなくソフト制約として含まれている。車両の後輪位置、前輪位置、その中心位置において障害物・道路境界との距離から制約条件が計算されている。
条件を満たさない解が出力された場合は、後段の後処理で弾かれ、前の周期で計画された trajectory を出力する。

自車付近の経路が振動しないように、自車近傍の経路点を前回の経路点と一致させる制約条件も含まれており、これが唯一の二次計画法のハード制約である。

#### 数式

以下のように、経路に対して車両が追従するときの bicycle kinematics model を考える。
時刻$k$における、経路上の車両の最近傍点の座標($x$座標は経路の接線に平行)から見た追従誤差に関して、横偏差$y_k$、向きの偏差$\theta_k$、ステア角$\delta_k$と定める。

![vehicle_error_kinematics](./media/vehicle_error_kinematics.png)

指令ステア角度を$\delta_{des, k}$とすると、ステア角の遅延を考慮した車両キネマティクスモデルは以下で表される。この時、ステア角$\delta_k$は一次遅れ系として指令ステア角に追従すると仮定する。

$$
\begin{align}
y_{k+1} & = y_{k} + v \sin \theta_k dt \\
\theta_{k+1} & = \theta_k + \frac{v \tan \delta_k}{L}dt - \kappa_k v \cos \theta_k dt \\
\delta_{k+1} & = \delta_k - \frac{\delta_k - \delta_{des,k}}{\tau}dt
\end{align}
$$

次にこれらの式を線形化する。$y_k$, $\theta_k$は追従誤差であるため微小近似でき、$\sin \theta_k \approx \theta_k$となる。

$\delta_k$に関してはステア角であるため微小とみなせない。そこで、以下のように参照経路の曲率$\kappa_k$から計算されるステア角$\delta_{\mathrm{ref}, k}$を用いることにより、$\delta_k$を微小な値$\Delta \delta_k$で表す。

ここで注意すべきこととしては、$\delta_k$は最大ステア角度$\delta_{\max}$以内の値を取る。曲率$\kappa_k$から計算された$\delta_{\mathrm{ref}, k}$が最大ステア角度$\delta_{\max}$より大きいときに$\delta_{\mathrm{ref}, k}$をそのまま使用して線形化すると、$\Delta \delta_k = \delta - \delta_{\mathrm{ref}, k} = \delta_{\max} - \delta_{\mathrm{ref}, k}$となり、$\Delta \delta_k$の絶対値が大きくなる。すなわち、$\delta_{\mathrm{ref}, k}$にも最大ステア角度制限を適用する必要がある。

$$
\begin{align}
\delta_{\mathrm{ref}, k} & = \mathrm{clamp}(\arctan(L \kappa_k), -\delta_{\max}, \delta_{\max}) \\
\delta_k & = \delta_{\mathrm{ref}, k} + \Delta \delta_k, \ \Delta \delta_k \ll 1 \\
\end{align}
$$

$\mathrm{clamp}(v, v_{\min}, v_{\max})$は$v$を$v_{\min}$から$v_{\max}$の範囲内に丸める関数である。

この$\delta_{\mathrm{ref}, k}$を介して$\tan \delta_k$を線形な式で近似する。

$$
\begin{align}
\tan \delta_k & \approx \tan \delta_{\mathrm{ref}, k} + \left.\frac{d \tan \delta}{d \delta}\right|_{\delta = \delta_{\mathrm{ref}, k}} \Delta \delta_k \\
& = \tan \delta_{\mathrm{ref}, k} + \left.\frac{d \tan \delta}{d \delta}\right|_{\delta = \delta_{\mathrm{ref}, k}} (\delta_{\mathrm{ref}, k} - \delta_k) \\
& = \tan \delta_{\mathrm{ref}, k} - \frac{\delta_{\mathrm{ref}, k}}{\cos^2 \delta_{\mathrm{ref}, k}} + \frac{1}{\cos^2 \delta_{\mathrm{ref}, k}} \delta_k
\end{align}
$$

以上の線形化を踏まえ、誤差ダイナミクスは以下のように線形な行列演算で記述できる。

$$
\begin{align}
    \begin{pmatrix}
        y_{k+1} \\
        \theta_{k+1} \\
        \delta_{k+1}
    \end{pmatrix}
    =
    \begin{pmatrix}
        1 & v dt & 0 \\
        0 & 1 & \frac{v dt}{L \cos^{2} \delta_{\mathrm{ref}, k}} \\
        0 & 0 & 1 - \frac{dt}{\tau}
    \end{pmatrix}
    \begin{pmatrix}
        y_k \\
        \theta_k \\
        \delta_k
    \end{pmatrix}
    +
    \begin{pmatrix}
        0 \\
        0 \\
        \frac{dt}{\tau}
    \end{pmatrix}
    \delta_{des}
    +
    \begin{pmatrix}
        0 \\
        \frac{v \tan(\delta_{\mathrm{ref}, k}) dt}{L} - \frac{v \delta_{\mathrm{ref}, k} dt}{L \cos^{2} \delta_{\mathrm{ref}, k}} - \kappa_k v dt\\
        0
    \end{pmatrix}
\end{align}
$$

平滑化と経路追従のための目的関数は以下で表され、

$$
\begin{align}
J_1 & (y_{0...N-1}, \theta_{0...N-1}, \delta_{0...N-1}) \\ & = w_y \sum_{k} y_k^2 + w_{\theta} \sum_{k} \theta_k^2 + w_{\delta} \sum_k \delta_k^2 + w_{\dot{\delta}} \sum_k \dot{\delta}_k^2 + w_{\ddot{\delta}} \sum_k \ddot{\delta}_k^2
\end{align}
$$

前述の通り、車両が障害物・道路境界に侵入しない条件はスラック変数を用いてソフト制約として表されている。
車両の後輪位置、前輪位置、およびその中心位置における障害物・道路境界までの距離をそれぞれ$y_{\mathrm{base}, k}, y_{\mathrm{top}, k}, y_{\mathrm{mid}, k}$とする。
ここでそれぞれに対するスラック変数 $\lambda_{\mathrm{base}}, \lambda_{\mathrm{top}}, \lambda_{\mathrm{mid}}$を定義し、

$$
y_{\mathrm{base}, k, \min} - \lambda_{\mathrm{base}, k} \leq y_{\mathrm{base}, k} (y_k)  \leq y_{\mathrm{base}, k, \max} + \lambda_{\mathrm{base}, k}\\
y_{\mathrm{top}, k, \min} - \lambda_{\mathrm{top}, k} \leq y_{\mathrm{top}, k} (y_k) \leq y_{\mathrm{top}, k, \max} + \lambda_{\mathrm{top}, k}\\
y_{\mathrm{mid}, k, \min} - \lambda_{\mathrm{mid}, k} \leq y_{\mathrm{mid}, k} (y_k) \leq y_{\mathrm{mid}, k, \max} + \lambda_{\mathrm{mid}, k}
$$

$y_{\mathrm{base}, k}, y_{\mathrm{top}, k}, y_{\mathrm{mid}, k}$は$y_k$の 1 次式として表現できるので、このソフト制約の目的関数は、以下のように記述できる。

$$
\begin{align}
J_2 & (\lambda_{\mathrm{base}, 0...N-1}, \lambda_{\mathrm{mid}, 0...N-1}, \lambda_{\mathrm{top}, 0...N-1}) \\ & = w_{\mathrm{base}} \sum_{k} \lambda_{\mathrm{base}, k}^2 + w_{\mathrm{mid}} \sum_k \lambda_{\mathrm{mid}, k}^2 + w_{\mathrm{top}} \sum_k \lambda_{\mathrm{top}, k}^2
\end{align}
$$

スラック変数も二次計画法の設計変数となり、全ての設計変数をまとめたベクトル$\boldsymbol{x}$を定義する。

$$
\begin{align}
\boldsymbol{x} =
\begin{pmatrix}
... & y_k & \lambda_{\mathrm{base}, k} & \lambda_{\mathrm{top}, k} & \lambda_{\mathrm{mid}, k} &  ...
\end{pmatrix}^T
\end{align}
$$

これらの 2 つの目的関数の和を目的関数とする。

$$
\begin{align}
\min_{\boldsymbol{x}} J (\boldsymbol{x}) = \min_{\boldsymbol{x}} J_1 & (y_{0...N-1}, \theta_{0...N-1}, \delta_{0...N-1}) + J_2 (\lambda_{\mathrm{base}, 0...N-1}, \lambda_{\mathrm{mid}, 0...N-1}, \lambda_{\mathrm{top}, 0...N-1})
\end{align}
$$

前述の通り、真にハードな制約条件は車両前方ある程度の距離$N_{fix}$までの経路点の状態は前回値になるような条件であり、以下のように記述できる。

$$
\begin{align}
\delta_k = \delta_{k}^{\mathrm{prev}} (0 \leq i \leq N_{\mathrm{fix}})
\end{align}
$$

であり、これらを以下のような二次計画法の係数行列に変換して二次計画法を解く

$$
\begin{align}
\min_{\boldsymbol{x}} \ & \frac{1}{2} \boldsymbol{x}^T \boldsymbol{P} \boldsymbol{x} + \boldsymbol{q} \boldsymbol{x} \\
\mathrm{s.t.} \ & \boldsymbol{b}_l \leq \boldsymbol{A} \boldsymbol{x} \leq \boldsymbol{b}_u
\end{align}
$$

## Limitation

- カーブ時に外側に膨らんだ経路を返す
- behavior_path_planner と obstacle_avoidance_planner の経路計画の役割分担がはっきりと決まっていない
  - behavior_path_planner 側で回避する場合と、obstacle_avoidance_planner で回避する場合がある
- behavior_path_planner から来る path が道路壁に衝突していると、大きく外側に膨れた trajectory を計画する (柏の葉のカーブで顕著)
- 計算負荷が高い

## How to debug

obstacle_avoidance_planner` から出力される debug 用の topic について説明する。

- **interpolated_points_marker**
  - obstacle avoidance planner への入力経路をリサンプルしたもの。この経路が道路内に収まっているか（道路内にあることが必須ではない）、十分になめらかか（ある程度滑らかでないとロジックが破綻する）、などを確認する。

![interpolated_points_marker](./media/interpolated_points_marker.png)

- **smoothed_points_text**
  - Elastic Band の計算結果。点群ではなく小さい数字が描画される。平滑化されたこの経路が道路内からはみ出ていないか、歪んでいないかなどを確認。

![smoothed_points_text](./media/smoothed_points_text.png)

- **(base/top/mid)\_bounds_line**
  - 壁との衝突判定における横方向の道路境界までの距離（正確には - vehicle_width / 2.0）。
  - 車両の 3 箇所（base, top, mid）で衝突判定を行っており、3 つのマーカーが存在する。
  - 黄色い線の各端点から道路境界までの距離が車幅の半分くらいであれば異常なし（ここがおかしい場合はロジック異常）。

![bounds_line](./media/bounds_line.png)

- **optimized_points_marker**
  - MPT の計算結果。道路からはみ出ていないか、振動していないかなどを確認

![optimized_points_marker](./media/optimized_points_marker.png)

- **Trajectory with footprint**
  - TrajectoryFootprint の rviz_plugin を用いて経路上の footprint を描画することが可能。これを用いて obstacle_avoidance_planner への入出力の footprint、経路に収まっているかどうか等を確認する。

![trajectory_with_footprint](./media/trajectory_with_footprint.png)

- **Drivable Area**
  - obstacle avoidance への入力の走行可能領域を表示する。Drivable Area の生成に不具合があると生成経路が歪む可能性がある。
  - topic 名：`/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/drivable_area`
  - `nav_msgs/msg/OccupancyGrid` 型として出力される

![drivable_area](./media/drivable_area.png)

- **area_with_objects**
  - 入力された走行可能領域から障害物を取り除いた後の、走行可能領域
  - `nav_msgs/msg/OccupancyGrid` 型として出力される

![area_with_objects](./media/area_with_objects.png)

- **object_clearance_map**
  - 回避対象の障害物からの距離を可視化したもの。
  - `nav_msgs/msg/OccupancyGrid` 型として出力される

![object_clearance_map](./media/object_clearance_map.png)

- **clearance_map**
  - 入力された走行可能領域からの距離を可視化したもの。
  - `nav_msgs/msg/OccupancyGrid` 型として出力される

![clearance_map](./media/clearance_map.png)
