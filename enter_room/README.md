# enter_room
## Overview
enter_roomは、ドアが開いたときに入室するパッケージ

## Description
このパッケージが提供する機能は主に以下の１つです。
- ドアが開いたら、指定速度で指定距離進む

## Usage
|Communication|Name|Type|Request|Result|
| :---: | :---: | :---: | :---: | :---: |
| Service | /enter_room_server | [EnterRoom](https://github.com/KIT-Happy-Robot/happymimi_apps/blob/develop/enter_room/srv/EnterRoom.srv) | float32型： `distance`,`velocity` | bool `result` |
