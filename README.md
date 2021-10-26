# happymimi_apps
# Overview
ナビゲーションや行動計画実行プログラムなど、各種機能をまとめたROSメタパッケージ

# Description
以下のパッケージを含みます。

- ### [happymimi_navigation](./happymimi_navigation)
  > ロボットの自律走行に関する機能を提供するパッケージ
  このパッケージでできること
  - gmappingを用いた地図生成
  - 安全でスムーズな自律走行
  - 目的地まで移動するためのROSサービスサーバ

- ### [happymimi_teleop](./happymimi_teleoop)
  > ロボットの足回り制御に関するパッケージ
  このパッケージでできること
  - 台車の前後進･左右回転制御
  - コントローラによる台車の制御
  
- ### [actplan_executor](./actplan_executor)
  > 行動計画を遂行するためのパッケージ
  
- ### [happymimi_rviz_launchers](./happymimi_rviz_launchers)
  > 可視化ツールを使うためのパッケージ
  
# Usage
使い方は各パッケージのREADMEをご覧ください。
