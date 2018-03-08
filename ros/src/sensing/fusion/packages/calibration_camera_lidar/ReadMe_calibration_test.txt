calibration_of_camera_and_lrf


利用時制約条件
ReadMe.png参照
1:チェッカーボード面の垂線がz軸と等しい
	x, y軸回転の固定
2:チェッカーボード中心の位置を観測
	z軸回転の固定
	x, y軸の並進ベクトルを固定

利用方法
1:calibration_of_camera_and_lrf/param.yamlファイルに項目を入力
	1.1:camera_calibration等で予めカメラの内部パラメータを求めておく必要があります。
	参考URL:http://ros-robot.blogspot.jp/2010/11/cameracalibrationusb.html
---param.yaml-start--------------------------------------------------------------------------------------
	checkerboard:
		pat_row:7   #チェッカーボードの縦ラインのマス(白マス4つ+黒マス4つであれば7)
		pat_col:11  #チェッカーボードの縦ラインのマス(白マス6つ+黒マス6つであれば11)
		chess_size:20.0 #マスのサイズ(mm)    
		paper_width_margin:66.0 #チェッカーボード外の紙の余白(mm)。適当な値で構いません
	window_lrf:
		width:1240   #LRFウィンドウサイズ（変更の必要なし）
		height:960   #LRFウィンドウサイズ（変更の必要なし）
		line_division_num:10   #LRFの縦ラインの数（変更の必要なし）
		judge_margin:30.0   #チェッカーボードとLRFのZ軸が法線上にあるかどうかの判断マージン（mm）。（変更の必要なし）    
	intrinsic_matrix: !!opencv-matrix
	    rows: 3
	    cols: 3
	    dt: f
	    data: [583.199804, 0.000000, 343.605359, 0.000000, 583.989335, 241.745468, 0.000000, 0.000000, 1.000000]   #camera_calibrationにて求めた行列
	distrotion_matrix: !!opencv-matrix
		rows: 1
		cols: 4
		dt: f
		data: [0.014429, -0.032025, 0.003003, -0.001721]  #camera_calibrationにて求めた行列
---param.yaml-end--------------------------------------------------------------------------------------
2:urg_node、uvc_camera、calibration_of_camera_and_lrfを実行
	2.1:LRFウィンドウ、カメラ画像ウィンドウが表示される
3:カメラ、LRFでチェッカーボードを認識し、CALIBRATE(CLICK)をクリックする
	3.1:LRFにて認識されるとLRFウィンドウにLRF OKと表示されます。
		3.1.1:LRF OKと表示されるためにはLRFウィンドウの2本の黄色い縦ラインの中にある観測した点群データが全部赤色になる必要があります。
		3.1.2:全て赤色になるにはLRFのZ軸上の点（黄色の点）から前後judge_margin:30.0mmである必要があります
	3.2:カメラにて認識されるとLRFウィンドウにCAMERA OKと表示されます。
4:カメラ画像ウィンドウにてセンサフュージョンが行われます。
5:カメラ画像ウィンドウ上部の６つのトラックバーにてパラメータを調節してください（各座標系.png参照（X軸:横、Y軸:縦、Z奥行））
	5.1:X,Y,Zベクトルを前後1000（1000mm）動かせるようになっています。（トラックバー値0=-1000）
	5.2:X,Y,Zの回転角度を360度動かせるようになっています。
6:適切に調節できましたら、SAVE(CLICK)をクリックしてください
7:現在のディレクトリにcamera.yamlが生成されます。



