Author:Yukihiro Saito
E-mail:yukky@ubi.cs.ritsumei.ac.jp

---- 環境 ----
OS:Ubuntu 12.04 32bit
ROS version:hydro
LRF:utm-30lx

---- パッケージ説明 ----
チェッカーボードを使用し、カメラと2D-LRFのキャリブレーションを行うパッケージです。
キャリブレーションを行うときには以下の制約条件があります。制約条件.pngを参照。各座標系の座標軸は各座標系.pngを参照。
1.LRF正面軸とチェッカーボードは法線の関係にあること
2.2D-LRFの観測ラインはチェッカーボードの半分の高さであること
3.カメラの内部パラメータが既知であること(camera_calibrationパッケージ等で求めることができます。)
また、カメラ画像のトピック名は/usb_cam/image_raw、LRF点群データのトピック名は/scanにデフォルトで設定されています。変更する場合は、適切なトピック名に替えリコンパイルしてください。
※OpenCVがインストールされていなければ動作しません。インストールされていない場合は、open_cv.shを実行してください。

---- 使用方法 ----
1.calibration_of_camera_and_lrf/param.yamlファイルにパラメータを入力する
checkerboard:
    pat_row:縦ラインのチェスの個数
    pat_col:横ラインのチェスの個数
    chess_size:チェスのサイズ #mm
    paper_width_margin:横ラインのチェッカーボードの余白
window_lrf:
    width:1240 表示ウィンドウのサイズ
    height:960 表示ウィンドウのサイズ
    line_division_num:10 横ラインの目安線の本数
    judge_margin:30.0 チェッカーボードがLRFの法線上にあるかどうかの誤差許容範囲 #mm
intrinsic_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: f
   data: カメラの内部パラメータ
distrotion_matrix: !!opencv-matrix
   rows: 1
   cols: 4
   dt: f
   data: カメラの歪みパラメータ
2.rosrun calibration_of_camera_and_lrf calibration_of_camera_and_lrfを実行
3.カメラウィンドウとLRFウィンドウが表示されます。LRFウィンドウの右上に"CAMERA OK" "LRF OK"と表示されている状態で"CALIBRATE(click)"をクリック
	3.1.黄色2本の縦線がチェッカーボードのサイズとなっており、黄色点がLRFの真正面の観測点になっています。黄色点の前後30mm(judge_marginデフォルト)内の場合、観測点が赤色になる。黄色2本の縦線内の観測点がすべて赤となった場合、チェッカーボードがLRFの法線上にあると見なし、"LRF OK"と表示する
	3.2.カメラでチェッカーボードが認識された場合、虹色のラインがしかれ"CAMERA OK"と表示する
4.camera.yamlが作成されsensors_fusionパッケージに必要なパラメータが書き込まれる
5.sensors_fusion/にcamera.yamlファイルを移動

