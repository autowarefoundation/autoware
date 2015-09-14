＊環境構築
 -Linux
　・Ubuntu 14.04

　-Oculus
　・Oculus DK2
　・SDK ovr_sdk_linux_0.4.4

　-openFrameworks
　・バージョン of_v0.8.4_linux64_release

　・手順
	1.openFrameworks  のサイト(http://openframeworks.jp/)より code::blocks(64 bit) をダウンロード
	2.IDE setup guide の手順に従いセットアップ
	3.openFrameworks の addons より ofxOculusDK2 (https://github.com/sphaero/ofxOculusDK2/tree/linux_0.4.4)をダウンロード
	4.of_v0.8.4_linux64_release の addons ディレクトリ以下に ofxOculusDK2 を置く
	5.of_v0.8.4_linux64_release の apps ディレクトリ以下の projectGenerator よりプロジェクトを作成する。(プロジェクト名称は任意)
	6.作成したプロジェクトの bin/data ディレクトリ, src ディレクトリ, Makefile, addons.make, config.make を このディレクトリ(Autoware/ui/oculus/ofw/) のものに置き換える。

＊使用方法

　・ROS側
	1.oculus_sender の起動(runtime_maneger の Oculus_Rift ボタン)

	↓ runtime_maneger を使用しない場合 (現時点ではこちらのみ？) ↓
	2.端末上で以下のコマンド入力 Oculus PC の ipアドレス を引数にとる
	  $ roslaunch oculus oculus_sender.launch ip:= "xxx.xxx.xxx.xxx"

　・ofw側
	1.ソースコード内、各mapのディレクトリパスの変更
	　-pointcloud_map.hの変更
	　　　#define PCD_DIR を 読み込みたいディレクトリのパスに設定
	　　　#define STATIC_DIR を 読み込みたいディレクトリのパスに設定
	　-testApp.hの変更
	　　　#define VECTOR を 読み込みたいディレクトリのパスに設定
	2.プロジェクトの起動

　・Oculus SDK 0.4.4 
	1.ofw の実行の前に Oculus SDK 内で
	　　　　% ./oculusd 
	　を実行する必要がある。

