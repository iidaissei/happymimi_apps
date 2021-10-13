# happymimi_navigation
## Overview
happymimi_navigationは、happymimiの自律走行機能を実現するためのパッケージです。<br>
このパッケージが提供するものは以下の3つです。
- SLAMを用いた地図生成
- ナビゲーション機能
- ロケーションの登録を行うROSサービスサーバ
- 登録したロケーションへ移動するためのROSサービスサーバ

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
navi_locationは、`/location_dict`をもとにナビゲーションを実行するROSサービスサーバです。
> `/location_dict`は、[happymimi_params/params/location]()のyamlファイルをROSパラメータとして登録したものです。<br>
> パラメータの読み込みは、[happymimi_bringupのminimal.laucnh]()の起動時に行われます。

ナビゲーションは、[/move_base](http://wiki.ros.org/move_base)アクションサーバを用いて実現しています。<br>
アクションサーバを呼び出す直前に、[/move_base/clear_costmaps](http://wiki.ros.org/move_base#Services)サービスサーバを呼ぶことで、ナビゲーション・スタックの時短を図っています。

**navi_locationの仕様**
| Name | Type | Request | Result |
|---|---|---|---|
| /navi_location | [NaviLocation]()| string型: `location_name` | bool型: `result` |


## Usage
ここではhappymimi_navigationの各機能を、コマンドラインから実行する方法を紹介します。<br>
以下のコマンドは全ての機能で必要となります。

ロボットとセンサの起動
```
$ roslaunch happymimi_bringup minimal.launch
```
```
$ roslaunch happymimi_bringup sensor.launch

```

---
### set_locationサービスサーバ
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
### navi_locationサービスサーバ
amcl.launchの起動
```
$ roslaunch happymimi_navigaiton amcl.laucnh
```
rviz(可視化ツール)の起動
```
$ roslaunch happymimi_rviz_launchers view_navigation.launch`
```
rvizが起動したら、`2dPointEstimate`でロボットの初期位置を調整してください。<br>

navi_locationサービスサーバの起動
```
$ rosrun happymimi_navigation navi_location.py
```

`/location_dict`ROSパラメータに登録されているロケーション名を入力すると移動を開始します。
```
$ rosservice call /navi_location "location_name: '<location_name>'"
```

---
