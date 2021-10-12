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
| save | ロケーションデータとマップを保存する |



### navi_locationサービスサーバ
[/location_dict]()をもとに、ロケーションまでナビゲーションするROSサービスサーバ

| Name | Type | Request | Result |
|---|---|---|---|
| /navi_location | [NaviLocation]()| string型の`location_name` | bool型の`result` |


## Usage

### set_locationサービスサーバ
**コマンドラインから登録する時**
ロボットとセンサの起動
`roslaunch happymimi_bringup minimal.launch`
`roslaunch happymimi_bringup sensor.launch`
gmapping（地図生成アプリ）の起動
`roslaunch happymimi_navigaiton gmapping.laucnh`
set_locationの起動
`rosser`
- save
> ロケーションの保存処理を実行します。
> Requestの`name`[happymimi_params/params/location]()に保存します。
> また、同名のマップファイルを保存します。
