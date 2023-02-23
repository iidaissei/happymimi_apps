# approach_person
approach_personは人接近機能を提供するROSパッケージです。

## Description
本パッケージは以下の3つのノードから構成されています。

- ### approach_person
    > approach_perosnは人接近を行うためのサービスサーバです。ここでは、human_coord_generatorノードにより生成された人のidと座標を用いて、人に接近する処理を行います。
    > 人のidと座標は[happymimi_params/location](https://github.com/KIT-Happy-Robot/happymimi_robot/tree/develop/happymimi_params/location)に保存された[tmp_human_location.yaml](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_params/location/tmp_human_location.yaml)を読み込むことで取得します。
    > 人の接近には[navi_locationサービスサーバ](https://github.com/KIT-Happy-Robot/happymimi_apps/blob/develop/happymimi_navigation/src/navi_location.py)を利用しており、目標が障害物として埋まってしまうのを回避するために目標位置の許容誤差を指定するパラメータ（Table1）を動的に変更しています。
    > 
    > Table1. 目標位置の許容誤差を指定するパラメータ
    > | パラメータ名 | 変更前 | 変更後|
    > |---|---|---|
    > | xy_goal_tolerance | 0.15 | 0.70 |
    > | yaw_goal_tolerance | 0.08 | 6.2 |


- ### human_coord_generator
    > human_coord_generatorは人のidと座標の辞書データを作成し、`tmp_human_location`として[happymimi_params/location]()に保存するためのサービスサーバです。
    > ここでは、[happymimi_recognition/recognition_processing]()の[recognition/multiple_localizeサービスサーバ]()から人のxy座標を取得し、pub_human_tfノードを用いてクォータ二オン座標に変換します。
    > さらにそれをマップ座標系に変換して人のidと紐付けることで辞書型データを作成し、[happymimi_params/location]()に`tmp_human_location`として保存します。

- ### pub_human_tf
    > pub_human_tfは人のクォータ二オン座標を生成するためにアクションサーバです。受け取った人のidとxy座標をもとに、クォータ二オン座標を生成し[TFMessage型](http://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html)のトピックとして配布します。


<p align="center">
 <img src="https://user-images.githubusercontent.com/45844173/220968416-d7ffb16e-dc6c-4b94-9a7a-d9f49ef3a3f1.png" width="100%">
</p>
<p align="center">
  approach_person概略図
</p>

## Usage

