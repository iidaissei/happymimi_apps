# happymimi_navigation
## Overview
happymimi_navigationは、happymimiの自律走行機能を実現するためのパッケージです。
このパッケージが提供するものは以下の3つです。
- SLAMを用いた地図生成
- ナビゲーション機能
- ナビゲーション・スタックに関する独自のサブ機能

## Description

### ナビゲーション・スタックに関するサブ機能
### navi_locationサービスサーバ
[/location_dict]()をもとに、ロケーションまでナビゲーションするROSサービスサーバ

| Name | Type | Request | Result |
|---|---|---|---|
| /navi_location | [NaviLocation]()| string型の`location_name` | bool型の`result` |


### set_locationサービスサーバ
ロケーションの登録をROSサービスサーバ
Requestの`state`に以下の文字列を入力すると、ロケーションの追加、登録処理を実行します。

- add
> 辞書型のロケーションデータを生成します。Requestの`name`に入った文字列をロケーション名と現在地の座標を取得します。

- save
> ロケーション


| Name | Type | Request | Result |
|---|---|---|---|
| /set_location | [SetLocation]()| string型の`state`と`name` | bool型の`result` |



## Usage
