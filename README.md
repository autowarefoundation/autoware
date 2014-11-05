# Autoware

Open software for autonomous driving

## How to Build

```
$ cd $HOME
$ git clone git@github.com:CPFL/Autoware.git
```

### Create symbolic links

`~/Autoware/ros/` 以下生成

```
$ cd ~/catkin_ws/src
$ ln -s ~/Autoware/ros/sensing
$ ln -s ~/Autoware/ros/computing
```

### catkin_make

```
$ cd ~/catkin_ws
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

```
 $ roscd calibration_of_camera_and_lrf/
```

[param.yaml](ros/sensing/calib/calibration_of_camera_and_lrf/param.yaml)の作成方法は [ReadMe.txt](ros/sensing/calib/calibration_of_camera_and_lrf/ReadMe.txt)を参照

[param.yaml](ros/sensing/calib/calibration_of_camera_and_lrf/param.yaml) 中の

```
  intrinsic_matirix: !!oencv-matirx
   data [] に ost.txt の camera matrix の 9 つの値を設定
  distrotion_matrix: !!opencv-matrix
   data [] に ost.txt の distortion の 5 つの値のうち末尾の 0 以外の4つの値を設定
```

```
 $ rosrun calibration_of_camera_and_lrf calibration_of_camera_and_lrf
```

LRF画面 CALIBRATE(click) SAVE(click)
端末の saved 表示確認して ^C で終了
`camera.yaml` が生成される

```
 $ cp camera.yaml ../../fusion/scan_to_image/
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

### image_detector ノード起動 (dpm_ocv)

```
 $ roscd roscd image/detectors/dpm_ocv/Debug/
 $ rosrun image image_detector
```

カメラの画像と矩形、その他が画面に表示される

#### デフォルトの状態

```c++
  overlapThreshold = 0.5
  numThreads = 6
  models_filenames.push_back( std::string("models_VOC2007/car.xml") );
```

#### 変更する場合
```
 $ rosparam set image_detecotr/threshold 0.48
 $ rosparam set image_detecotr/threads 7
 $ rosparam set image_detecotr/models "[ models_VOC2007/car.xml , models_VOC2007/person.xml ]"
```

### image_detector ノード起動 (dpm_ttic)

```
 $ rosparam set image_detector/algorithm ttic
 $ roscd image/detectors/dpm_ttic/
 $ rosrun image image_detector
```

デフォルトのdpm_ocvに戻す場合
```
$ rosparam delete image_detector/algorithm
```


### fusion ノード起動

```
 $ roscd sensor_fusion/src/
 $ rosrun sensor_fusion fusion
```

カメラの画像と矩形、その他が画面に表示される


### /fused_objects topic出力の確認

```
 $ rostopic echo /fused_objects
```
