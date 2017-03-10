# lane_planner

## Package

This package has following nodes.
- lane_navi
- lane_rule
- lane_select
- lane_stop

## Nodes

### lane_select

1. 概要
    1. 複数経路を受け取る
    1. 全ての経路に対して現在位置からの最近傍点を算出する
    1. 一番近い最近傍点を持つ車線を現在走行している経路と設定
    1. 現在経路に対する左右の経路を検出
    1. 現在経路の最近傍点が持つ車線変更フラグをその経路の車線変更フラグとして保持
    1. 最近傍の右折or左折のフラグを持つ点を探し、その点と車線変更を行う予定の車線の目標点をエルミート補間した経路を生成、その経路と車線変更予定の目標点以降の経路を結合した経路を車線変更用の経路として定義
    1. 車線変更を行わない場合、現在経路、それに対する最近傍点、車線変更フラグをそれぞれpublishする
    1. 車線変更を行う場合、車線変更用の経路、それに対する最近傍点、車線変更フラグをそれぞれpublishする
    
1. 注意
    - 車線変更を行うには、ver3フォーマットの経路ファイルを複数個読み込まれている必要がある。（waypoint_makerパッケージ参照）

1. Subscribed Topics

    - traffic_waypoints_array (waypoint_follower/LaneArray)
    - current_pose (geometry_msgs/PoseStamped)
    - current_velocity (geometry_msgs/TwistStamped)
    - state (std_msgs/String)
    - config/lane_select (runtime_manager/ConfigLaneSelect)
    
1. Published Topics

    - base_waypoints (waypoint_follower/lane)
    - closest_waypoint (std_msgs/Int32)
    - change_flag (std_msgs/Int32)
    - lane_select_marker (visualization_msgs/MarkerArray) : for debug
    
1. Parameter from Runtime Manager

    - Distance threshold to neighbor lanes<br>
    現在経路の周りの有効な車線を検出する際のしきい値を表す。このしきい値より遠い距離にある車線は車線として認識しない。
    
    - Lane Change Interval After Lane Merge
    車線変更を行ったあとに何メートル走ったらまた車線変更を行えるようになるかの値を表す。
    
    - Lane Change Target Ratio
    車線変更を行う予定の車線上の目標点を速度(m/s)に比例した距離で定義する際に使用する値。
    目標点探索の起点は車線変更予定の車線上の点において、右折or左折の車線変更フラグを持つ点の最近傍点。
    - Lane Change Target Minimum
    車線変更を行う予定の車線上の目標点までの最低距離を表す。
    目標点探索の起点は車線変更予定の車線上の点において、右折or左折の車線変更フラグを持つ点の最近傍点。
    - Vector Length of Hermite Curve
    エルミート曲線で補完する際のベクトルの大きさを表す。


