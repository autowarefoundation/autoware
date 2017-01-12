# lane_planner

## 概要

This package has following nodes.
- lane_navi
- lane_rule
- lane_select
- lane_stop

## Nodes

### lane_select

1. 概要
    - 複数経路を受け取り、全ての経路に対して現在位置からの最近傍点を算出する。その後、一番近い最近傍点を持つ経路を初期経路とし、最近傍点と共にpublishする
    - 最近傍点のchange_flagが1（右折）もしくは2（左折）の場合は、右隣もしくは左隣の経路を現在の経路に変更し、最近傍点と共にpublishする
1. 注意
    - 車線変更を行うには、ver3フォーマットの経路ファイルを複数個読み込まれている必要がある。（waypoint_makerパッケージ参照）
1. 使い方
    - appタブ->`lane_change_interval`で、レーンチェンジのインターバル（s）を設定
    - appタブ-`distance_threshold`で、現在走っている経路の両隣の経路を探す際のしきい値（m)を設定

1. Subscribed Topics

    - traffic_waypoints_array (waypoint_follower/LaneArray)
    - /current_pose (geometry_msgs/PoseStamped)
    - /current_velocity (geometry_msgs/TwistStamped)
    
1. Published Topics

    - base_waypoints (waypoint_follower/lane)
    - closest_waypoint (std_msgs/Int32)
    - lane_select_marker (visualization_msgs/MarkerArray) : for debug
    
1. Parameters

    - ~lane_change_interval
    - ~distance_threshold


