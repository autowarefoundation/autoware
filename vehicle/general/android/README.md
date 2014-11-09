# CarLink|CanGather + CanDataSender(仮称) で、CANデータを直接DBに登録する手順

## 制限事項

* CarLink の場合、CANデータの登録処理が間に合いません。
* 画像、動画は捨てています。

## テーブルの作成

下記を実行すると、テーブル「candata」が作成される。  
※ すでにある場合、drop されるので注意!

`$ psql < CanDataSender/CanDataSender-table.sql`

## poststore の起動

1. db1 にログインする。
2. poststore のソースコードのうち、下記を適時変更する。
   * ServerMain.java の CONNECTION_PORT (他とかち合わない番号)
   * postgresql/DbBase.java の db で始まる各値
3. ビルドする。(warning は気にしない :-p)  
   `$ make`
4. poststore を起動する。  
   `$ ./server.sh` あるいは `./server-nohup.sh`  
   (接続した旨のメッセージが出力されればよい)

## CanDataSender の起動

1. CanDataSender を起動する。
2. 「サーバ接続」をタップする。
2. 「接続先の追加」をタップして、情報を入力する。  
   (最後のポート番号が、1-2 で指定した CONNECTION_PORT)
3. 追加した接続名が表示されるので、それをタップする。
4. 「接続」をタップする。  
   (正常に接続されると、接続名の前に "接続中" と表記される。)
5. メイン画面に戻り、「データ登録先テーブル」をタップする。  
   サーバのテーブル一覧が表示されるので、登録したいテーブル名をタップする。
6. 「RAWデータ送信先ディレクトリ」をタップし、ファイルの保存先ディレクトリを入力する。  
   ※ 現状では保存されない。読み捨てている。
7. 「自動データ登録」をタップして、ダイアログ右下の「起動」をタップする。

## CarLink もしくは CanGather を起動

### CarLink の場合

1. 資料を参考に、CAN-BT もしくは CANusbAccessory を接続する。
2. CarLink を起動する。(あるいは機器接続で自動起動される。)
3. 右上メニューの「Local Socket Transfer」にチェックを入れる。ソケット名は変更しなくてよい。
4. 右上メニューの「Video Recording Interval」など設定する。  
   ※ 現状では保存されないため、意味はあまりない。
5. 右上メニューの「Start」を選択すると、データ取得開始。

### CanGather の場合

0. あらかじめ設定ファイルを置いておく。  
   `/sdcard/ECS/CanGather/Properties.txt`  
   `/sdcard/ECS/CanGather/SensorInfo.xml`  
   ※ Properties.txtは、オリジナルから下記を追加している。  
   `# CSVに出力`  
   `STR_STORE_TO_CSV = yes`  
   `# UNIXドメインソケットに出力`  
   `STR_SEND_LOCAL_SOCKET = com.metaprotocol.android.carlink_sample_receiver_service.server`  
   `# 電源チェックしない`  
   `STR_NO_CHECK_POWER_CHARGE = yes`

1. D-CUBE を接続する。
2. CanGather を起動する。
3. 「動作設定」をタップして、ペアリング、プロトコルの選択を行う。
4. メイン画面に戻って、すべてのステータスの色が青になればよい。  
   ※ 画像は常にグルグル回っている。

## データを取得

…じっと待つ...

## CarLink もしくは CanGather を終了

### CarLink の場合

右上メニューから Quit を選択

### CanGather の場合

設定→アプリ→CanGather で強制終了

## CanDataSender を終了

1. CanDataSender を起動し、「自動データ登録」をタップして、ダイアログ左下の「停止」をタップする。
2. 「終了」をタップして終了する。

## リンクなど

* <http://www.metaprotocol.com/CAN/CANusbAccessory.html>
* <http://www.bbcs.jp/pc/m2mcloud/dcube.html>
