# enter_room
## Overview
enter_roomは、ドアが開いたときに入室するパッケージ

## Description
このパッケージが提供する機能は主に以下の１つです。
- ドアが開いたら、指定速度で指定距離進む

## Usage
|Communication|Name|Type|Request|Result|
| :---: | :---: | :---: | :---: | :---: |
| Service | /enter_room_server | [EnterRoom](https://github.com/KIT-Happy-Robot/happymimi_apps/blob/develop/enter_room/srv/EnterRoom.srv) | float32型： `distance`,`velocity` | bool型： `result` |

---
### enter_room_server サービスサーバー
ここではenter_roomをコマンドから実行する方法を紹介します。

サービスサーバ /enter_room_server起動
```
$ rosrun enter_room enter_server.py
```
距離`distance`と速度`velocity`を指定
```
$ rosservice call /enter_room_server "distance: 0.0 velocity: 0.0"
```
