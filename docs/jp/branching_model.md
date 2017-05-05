# Autoware開発のためのブランチモデル

Autowareのオープンソース開発を効率化するために開発者は以下のブランチモデルを守るようにしてください。
基本ブランチはmaster、develop、release、feature、hotfixの5つになります。
以下、状況による対応とその対応ブランチについて記します。

## 機能追加
開発者は基本的にdevelopブランチ上で機能追加のための作業していただくことになります。
developブランチからfeature/[branch_name]というようにブランチを分岐していただき、作成した変更はdevelopにpull requestを出してください。

## リリース
機能追加がそれなりに終わってリリースできるようになれば、devleopブランチからrelease/ [1.xx.yy]のような名前で分岐し、リリースのための処理をしてからmasterにpull requestを出します。
versionのxxはdevelopから分岐した際、yyはバグフィックスを行った際にインクリメントされます。
この後、このブランチはmasterにマージされます。マージされた段階でmasterにバージョンタグを付与します。
この作業はリリース担当（誰になるかはわかりませんが。。。）が行うので、皆さんが行う必要はありません。

## バグフィックス
もしリリース後masterにバグが発生した場合は、hotfixブランチをmasterから分岐させて、バグの修正を行います。このブランチはmaster、release、developにそれぞれマージされます。マージ後、releaseとmasterのバージョンはインクリメントされます。

実際のコンセプトの詳細については以下のURLを参考にしてください。
- http://qiita.com/KosukeSone/items/514dd24828b485c69a05
- https://havelog.ayumusato.com/develop/git/e513-git_branch_model.html
