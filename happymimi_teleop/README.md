# happymimi_teleop
## Overview
happymimi_teleopは、happymimiの足回り制御機能をまとめたROSパッケージです。


## Description
このパッケージが提供するもの

### base_controlモジュール
このモジュールは前後進、左右回転、。

### 足回りのコントローラ制御ノード
キーボードからの制御と、ジョイスティックコントローラからの制御ができます。

## Usage
### コントローラ制御ノードの起動
ジョイスティックコントローラを使うときは、コントローラをUSBに接続した後に、下記コマンドを実行してください。

`roslaunch happymimi_teleop joycon_teleop.launch`

キーボードを使う場合

`roslaunch happymimi_teleop keyboard_teleop.launch`
