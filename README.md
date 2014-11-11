# Autoware

Open software for autonomous driving

## How to Build

```
$ cd $HOME
$ git clone git@github.com:CPFL/Autoware.git
$ cd ~/Autoware/ros/src
$ catkin_init_workspace
$ cd ../
$ catkin_make clean # for generating setup scripts
$ source devel/setup.bash # you should load devel/setup.zsh if you are zsh user
$ catkin_make
```

cleanする場合は `catkin_make clean`, ビルドプロセスの詳細を見たい場合, `catkin_make VERBOSE=1`.

## How to Run

### カメラキャリブレーション

参考URL
 - http://ros-robot.blogspot.jp/2010/11/cameracalibrationusb.html

USBカメラを接続し起動

```
 $ rosrun uvc_camera uvc_camera_node
```

チェッカーボードを用意 (例えば checkr2.bmp)

```
 $ rosrun camera_calibration cameracalibrator.py --size 7x3 --square 0.0175 image:=/image_raw
```

ボードを動かしてみて
 - calibrate ボタン有効 click
 - save ボタン有効 click

メッセージ `('Wrote calibration data to', '/tmp/calibrationdata.tar.gz')`を確認

```
 $ tar xzf /tmp/calibrationdata.tar.gz ost.txt
```

ost.txt の内容のうち

  - camera matrix の 9 つの値
  - distortion の 5 つの値のうち末尾の 0 以外の4つの値

を使用する

### 空間キャリブレーション

北陽センサを接続し起動

```
 $ rosparam set hokuyo_node/calibrate_time false
 $ rosparam set hokuyo_node/port /dev/ttyACM0
 $ rosrun hokuyo_node hokuyo_node
```

[param.yaml]作成方法は [ReadMe.txt](ros/sensing/calib/offline/camera_lidar_2d/ReadMe.txt)を参照

[param.yaml]中の

```
  intrinsic_matirix: !!oencv-matirx
   data [] に ost.txt の camera matrix の 9 つの値を設定
  distrotion_matrix: !!opencv-matrix
   data [] に ost.txt の distortion の 5 つの値のうち末尾の 0 以外の4つの値を設定
```

```
 $ roscd camera_lidar_2d/
 $ rosrun camera_lidar_2d calibration_of_camera_and_lrf
```

LRF画面 CALIBRATE(click) SAVE(click)
端末の saved 表示確認して ^C で終了
`camera.yaml` が生成される

```
 $ cp camera.yaml $(rosdir)/sensing/fusion/scan_to_image/
```

#### 別の場所にある param.yaml を使用する場合

```
 $ rosparam set scan_to_image/param_yaml ~/other_dir/param.yaml
 $ rosrun camera_lidar_2d calibration_of_camera_and_lrf
```
この場合は roscd camera_lidar_2d/ は不要

#### デフォルトのパスに戻す場合
```
 $ rosparam delete scan_to_image/param_yaml
```


### カメラ起動

```
 $ rosrun uvc_camera uvc_camera_node
```

### 北陽センサ起動

```
 $ rosparam set hokuyo_node/calibrate_time false
 $ rosparam set hokuyo_node/port /dev/ttyACM0
 $ rosrun hokuyo_node hokuyo_node
```

### point_to_image ノード起動

```
 $ roscd scan_to_image/
 $ rosrun scan_to_image point_to_image
```

#### 別の場所にある camera.yaml menual.yaml を使用する場合

```
 $ rosparam set scan_to_image/camera_yaml ~/other_dir/camera.yaml
 $ rosparam set scan_to_image/manual_yaml ~/anther_dir/manual.yaml
 $ rosrun scan_to_image point_to_image
```
この場合は roscd scan_to_image/ は不要

#### デフォルトのパスに戻す場合
```
 $ rosparam delete scan_to_image/camera_yaml
 $ rosparam delete scan_to_image/manual_yaml
```


### car_detectorノードの起動

```
 $ rosrun image car_detector
```

カメラの画像と矩形、その他が画面に表示される

### pedestrian_detectorノードの起動

```
 $ rosrun image pedestrian_detector
```

カメラの画像と矩形、その他が画面に表示される

#### OpenCV版, GPU版の切り替え(デフォルト: `OpenCV`)

```
# OpenCV版
 $ rosparam set car_detector/algorithm ocv
 $ rosparam set pedestrian_detector/algorithm ocv

# GPU版
 $ rosparam set car_detector/algorithm gpu
 $ rosparam set pedestrian_detector/algorithm gpu
```

#### OpenCV版 car_detector, pedestrian_detectorのデフォルトパラメータ

```c++
  overlapThreshold = 0.5
  numThreads = 6
```

#### OpenCV版 car_detector, pedestrian_detectorのパラメータ変更

```
# 車
 $ rosparam set car_detector/threshold 0.48
 $ rosparam set car_detector/threads 7

# 歩行者
 $ rosparam set pedestrian_detector/threshold 0.48
 $ rosparam set pedestrian_detector/threads 7
```


### fusion ノード起動

```
 $ rosrun sensor_fusion fusion
```

カメラの画像と矩形、その他が画面に表示される


### /fused_objects topic出力の確認

```
 $ rostopic echo /fused_objects
```


### デバッグ開発用にダミー画像を送るノードの起動

```
 $ rosrun camera camera_sim [画像ファイルのパス]
```

画像ファイルの内容を /image_raw topic として出力する

#### フレームレートの変更

デフォルト設定は 30 fps

```
 $ rosparam set camera_sim/fps 15
```
