# enter_room
## Overview
enter_roomは、ドアが開いたときに入室するパッケージ

## Description
このパッケージが提供する機能は主に以下の１つです。
- ドアが開いたら、指定速度で指定距離進む </br>
⚠️ **指定距離は入室後に進む距離を指定してください。開始位置からではありません。**

## Usage
|Communication|Name|Type|Request|Result|
| :---: | :---: | :---: | :---: | :---: |
| Service | /enter_room_server | [EnterRoom](https://github.com/KIT-Happy-Robot/happymimi_apps/blob/develop/enter_room/srv/EnterRoom.srv) | float32型： `distance`,`velocity` | bool型： `result` |
</br>

### コマンドラインから使う
サービスサーバ /enter_room_server起動
```
$ rosrun enter_room enter_server.py
```
距離`distance`と速度`velocity`を指定
```
$ rosservice call /enter_room_server "distance: 0.0 velocity: 0.0"
```

### プログラムから使う
0.5m/sで1.0m進ませる
```
from enter_room.srv import EnterRoom

enter_room = rospy.ServiceProxy('/enter_room_server', EnterRoom)
enter_room(1.0, 0.5)
```
