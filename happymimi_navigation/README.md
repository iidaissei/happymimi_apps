# happymimi_navigation
## Overview
happymimi_navigationは、happymimiの自律走行機能を実現するためのパッケージです。
このパッケージが提供するものは以下の3つです。
- SLAMを用いた地図生成
- ナビゲーション機能
- ロケーションの登録サービスサーバ
- ロケーションへのナビゲーションサービスサーバ

## Description

### set_locationサービスサーバ
set_locationは、ロケーションの登録を行うROSサービスサーバです。<br>
登録した情報は[happymimi_params/params/location]()に、辞書型のyamlファイルとして保存されます。<br>

**set_locationの仕様**
| Name | Type | Request | Result |
|---|---|---|---|
| /set_location | [SetLocation]()| string型: `state`<br>string型: `name` | bool型: `result` |

**stateの種類**
| state | Contents |
|---|---|
| add | 現在地の座標を`name`の名前で登録する |
| save | ロケーションデータとマップを`name`の名前で保存する |


### navi_locationサービスサーバ
navi_locationは、`/location_dict`をもとにナビゲーションを実行するROSサービスサーバです。<br>
※/location_dictは、[happymimi_params/params/location]()の辞書型データをROSパラメータとして登録したものです。

**navi_locationの仕様**
| Name | Type | Request | Result |
|---|---|---|---|
| /navi_location | [NaviLocation]()| string型: `location_name` | bool型: `result` |


## Usage

### set_locationサービスサーバ

### コマンドラインからの登録作業

ロボットとセンサの起動
```
$ roslaunch happymimi_bringup minimal.launch
```
```
$ roslaunch happymimi_bringup sensor.launch
```

gmapping（地図生成アプリ）の起動
```
$ roslaunch happymimi_navigaiton gmapping.laucnh
```

set_locationの起動
```
$ rosrun happymimi_navigaiton set_location.py
```

ロケーションの追加
```
$ rosservice call /set_location "state: 'add' name: '<location_name>'
```

ロケーションの保存
```
$ rosservice call /set_location "state: 'save' name: '<file_name>'
```

---
