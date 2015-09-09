## Voice：音声入力アプリ

下記のアプリの一部を利用した。
https://github.com/thorikawa/AndroidWearSampleJp

mobileモジュールをスマートフォンにインストールすることで音声入力部分は利用可能。
「VoiceMobile」アプリを起動し、「Notification」→「Notification with voice reply」を選択することでスマートウォッチに通知が送信される。
wear側では通知を右にスワイプして返信を選ぶことで返信ができる。
「お話しください」と表示されている状態で音声入力することで、話した音声を文字列に変換してスマートフォンに送信する。
スマートフォンでは受信したメッセージを表示する。


## mapdemo：地図アプリ
mobileモジュールでは下記のアプリをの一部を利用した。
https://github.com/bati11/wear-datalayer-sample
appモジュールでは下記のアプリを一部編集して利用した。
https://github.com/lantian699/MapDemoApp

mobileモジュールをスマートフォンに、appモジュールをスマートウォッチにインストールする。

スマートフォンでは「mapdemo」アプリを起動（USB接続している状態でないと起動出来なかった）、「Message」を選択し、InputMessageと表示されている状態で待機する。

スマートウォッチでは、「GoogleMapsAPIDemos」アプリを起動、「Basic Map」を選択すると地図が表示される。
タップすることでマーカーが設置され、その地点の緯度と経度をスマートフォン側で表示する。
ロングタップすることで現在位置を表示、非表示にする。
表示状態のときに左のボタンを押すことで表示位置を現在地まで移動する。


-----------------
作成者 平岡 祥
show@sqlab.jp
