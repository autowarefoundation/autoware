# waypoint_maker

## 概要

- This package has following nodes.

    - waypoint_clicker
    - waypoint_saver
    - waypoint_marker_publisher
    - waypoint_loader

- waypoint_makerで扱える経路ファイルのcsv形式には3種類あり

    - ver1：x, y, z, velocityで構成される（1行目はvelocityを持たない）

    ex)

    > 3699.6206,-99426.6719,85.8506 <br>
    > 3700.6453,-99426.6562,85.8224,3.1646 <br>
    > 3701.7373,-99426.6250,85.8017,4.4036 <br>
    > 3702.7729,-99426.6094,85.7969,4.7972 <br>
    > 3703.9048,-99426.6094,85.7766,4.7954 <br>
    > 3704.9192,-99426.5938,85.7504,4.5168 <br>
    > 3705.9497,-99426.6094,85.7181,3.6313 <br>
    > 3706.9897,-99426.5859,85.6877,4.0757 <br>
    > 3708.0266,-99426.4453,85.6608,4.9097 <br>
    > ... <br>

    - ver2：x, y, z, yaw, velocityで構成される（1行目はvelocityを持たない）

    ex)

    > 3804.5657,-99443.0156,85.6206,3.1251 <br>
    > 3803.5195,-99443.0078,85.6004,3.1258,4.8800 <br>
    > 3802.3425,-99442.9766,85.5950,3.1279,7.2200 <br>
    > 3801.2092,-99442.9844,85.5920,3.1293,8.8600 <br>
    > 3800.1633,-99442.9688,85.5619,3.1308,10.6000 <br>
    > 3798.9702,-99442.9609,85.5814,3.1326,10.5200 <br>
    > 3796.5706,-99442.9375,85.6056,3.1359,10.2200 <br>
    > 3795.3232,-99442.9453,85.6082,3.1357,11.0900 <br>
    > 3794.0771,-99442.9375,85.6148,3.1367,11.2300 <br>
    > ... <br>

    - ver3：一番最初の行にデータ項目がある

    ex） x,y,z,yaw,velocity,change_flagで構成される場合

    > x,y,z,yaw,velocity,change_flag <br>
    > 3742.216,-99411.311,85.728,3.141593,0,0 <br>
    > 3741.725,-99411.311,85.728,3.141593,10,0 <br>
    > 3740.725,-99411.311,85.733,3.141593,10,0 <br>
    > 3739.725,-99411.311,85.723,3.141593,10,0 <br>
    > 3738.725,-99411.311,85.719,3.141593,10,0 <br>
    > 3737.725,-99411.311,85.695,3.141593,10,0 <br>
    > 3736.725,-99411.311,85.667,3.141593,10,0 <br>
    > 3735.725,-99411.311,85.654,3.141593,10,0 <br>
    > ... <br>

## Nodes

### waypoint_loader

1. 概要

    - `waypoint_loader`は上記3種類の経路ファイルに対応している。
    - `lane_select`にてレーンチェンジをしたい場合は、ver3フォーマットを用意する必要がある。

1. 使い方

    - 経路のロードをするために、appタブ->`Multi Lane`の項目でRefを押し、複数ファイルを選択する。
    - appタブ->`decelerate`の項目は経路の終点までにどれくらいの加速度で減速していくかが指定できる。

1. Subscribed Topics

    - なし
    
1. Published Topics

    - /lane_waypoints_array (waypoint_follower/LaneArray)
    
1. Parameters

    - ~multi_lane_csv
    - ~decelerate


### waypoint_saver

1. 概要

    - `waypoint_saver`はver3フォーマットの保存に対応している。
    - 起動すると、`/current_pose`、`/current_velocity`(option)をsubscribeし、指定した距離おきにwaypointをファイルに保存していく。
    - `change_flag`は基本的に0（直進)で保存されるので、レーンチェンジを行いたい場合は各自で編集する。（1なら右折、2なら左折）

1. 使い方

    - appタブ->`Save File`の項目でRefを押し、保存ファイル名を指定する。
    - appタブ->`Save /current_velocity`で速度保存可否が選択可能、チェックがない場合は0で保存される。
    - appタブ->`Interval`で、何メートルおきにwaypointを保存するかが設定される。

1. Subscribed Topics

    - /current_pose (geometry_msgs/PoseStamped) : default 
    - /current_velocity (geometry_msgs/TwistStamped) : default 
    
1. Published Topics

    - なし
    
1. Parameters

    - ~save_filename
    - ~interval
    - ~velocity_topic
    - ~pose_topic
    - ~save_velocity