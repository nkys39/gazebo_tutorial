# Gazeboの応用例

## 他のロボット


## 自律走行関連
[qiita_@BEIKE_2次元地図からGazeboのmodelを作ってナビゲーションしてみた](https://qiita.com/BEIKE/items/825e62bbd8d92b6d347e)



# Gazeboのチュートリアル作成
## 静的Webページ構築
下記を用いて静的Webページを構築した。[ページはこちら](https://nkys39.github.io/gazebo_tutorial/)
- GitHub Pages
- material for docs

## 参考サイト
- [qiita_M1 MacでもROSでgazeboとUSBデバイスを全部使いたい！](https://qiita.com/B-SKY-Lab/items/c519e5bc9a37b3805fd0#docker%E3%82%B3%E3%83%B3%E3%83%86%E3%83%8A%E5%86%85%E3%81%8B%E3%82%89%E3%81%AEopengl%E3%81%AE%E5%91%BC%E3%81%B3%E5%87%BA%E3%81%97%E7%A2%BA%E8%AA%8D)
- [Gazebo-Classic本家チュートリアル](https://classic.gazebosim.org/tutorials?cat=guided_a)
- [Homebrew](https://brew.sh/ja/)
- https://gazebosim.org/docs/harmonic/releases
- https://dev.classmethod.jp/articles/reinvent_robotics_gazebo/
- https://discourse.ros.org/t/ros-gui-on-macbook-air-m1-chip/27205
- https://github.com/HarvestX/h6x-Internship/tree/main
- https://note.com/npaka/n/ncfc333ddf20e
- https://qiita.com/srs/items/49b8512e29bdef0ebcd6
- https://qiita.com/srs/items/5f44440afea0eb616b4a#%E3%82%B7%E3%83%9F%E3%83%A5%E3%83%AC%E3%83%BC%E3%82%B7%E3%83%A7%E3%83%B3%E5%9F%BA%E7%A4%8E%E7%B7%A8

https://github.com/RAFALAMAO/hector-quadrotor-noetic
[ROS/ROS2のGUIをWebブラウザ経由でお手軽に試せるDockerfileを公開しました](https://memoteki.net/archives/2955)
## 内容
- Mac上でのインストール方法
    - Homebrewでのインストール方法
        - /usr/local/share/gazebo/setup.bash
    - Dockerを用いたインストール方法
    https://www.drbuho.com/jp/how-to/uninstall-docker-mac
    - UTM仮想環境(Ubuntu)内にDockerをインストールする方法
        - https://www.server-world.info/query?os=Ubuntu_22.04&p=japanese
        https://docs.getutm.app/guides/ubuntu/#creating-a-new-virtual-machine

        https://www.server-world.info/query?os=Ubuntu_22.04&p=japanese
- Gazeboの概要
    - Gazeboについて
    - Gazeboのバージョン
    - ROS/ROS2連携する際の互換性
- Gazeboの使い方
    - Gazeboの操作方法
        - GUIの使い方
        - CUIの使い方(gz --verbose)
    - ロボットの作り方
        - ロボット本体
        - センサーの配置
        - 
    - worldの作り方
    - プラグインの設定方法と作り方

- ROS/ROS2連携した色々なロボットのシミュレーション


https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp
- 4輪ステアリング
    - https://www.youtube.com/watch?v=4_bftUbQTaM&t=90s
    https://github.com/ros-controls/ros_controllers/tree/noetic-devel/four_wheel_steering_controller
- 差動二輪
https://github.com/ROBOTIS-GIT/turtlebot3_simulations
- 差動四輪
- メカナム
- オムニ
https://qiita.com/srs/items/6191b81d564fd934aa6a
- アッカーマンステアリング
- クローラー
- クアドロコプタードローン
    https://github.com/RAFALAMAO/hector-quadrotor-noetic
- モバイルマニピュレータ
    - https://github.com/hsr-project/hsrb_wrs_gazebo_launch

- マニピュレータ
    - https://github.com/utecrobotics/ur5
    - https://demura.net/robot/athome/14710.html
- 4脚
https://qiita.com/nisshan_/items/fa4a4ab807c5e4bb1ff4

world作り
https://classic.gazebosim.org/tutorials?tut=static_map_plugin&cat=build_world

https://mizuncoonote.blogspot.com/2021/11/blender-free3d-data-download-revue.html
https://harvestx.jp/technology/
https://github.com/HarvestX/h6x-Internship/tree/main
robot作り
https://qiita.com/RyodoTanaka/items/b1ebd48da81669db7409
https://classic.gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros#PlanarMovePlugin