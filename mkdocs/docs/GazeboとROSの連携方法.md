

## ROS Noeticのインストール
インストール手順での変更点は、aptでインストールしていたpythonがpython3になっている点のみ。
```bash
# aptのリストにROSを登録
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# ROSをaptからインストールするためのapt-keyを設定
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update

# ROS Noeticのインストール
$ sudo apt install ros-noetic-desktop-full

# ROSパッケージの依存関係を整理してくれるrosdepのインストールと初期設定
$ sudo apt install python3-rosdep
$ sudo rosdep init
$ rosdep update

# ROSの環境設定
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc 

# ROSパッケージのinstallが便利になるツールのインストール
$ sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
$ sudo apt-get install python3-catkin-tools

#ROSワークスペースを作成し、環境設定
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws/
$ catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc 
```

## 動作確認
下記コマンドを入力し、カメのウインドウが表示されればインストール完了
```
ROSマスタを起動
$ roscore
カメシミュレーターを起動
$ rosrun turtlesim turtlesim_node
カメを動かしたければ、以下を実行し、矢印キーで操作可能
$ rosrun turtlesim turtlesim_node
```

## 講習会環境構築
```
sudo apt install ros-noetic-teleop-twist-joy ros-noetic-urg-node ros-noetic-ira-laser-tools ros-noetic-depthimage-to-laserscan ros-noetic-pointcloud-to-laserscan ros-noetic-gmapping ros-noetic-slam-gmapping ros-noetic-navigation ros-noetic-mrpt-navigation ros-noetic-teb-local-planner ros-noetic-xacro ros-noetic-rosbridge-suite ros-noetic-tf2-web-republisher ros-noetic-velodyne-description ros-noetic-velodyne-gazebo-plugins ros-noetic-iris-lama-ros ros-noetic-slam-toolbox ros-noetic-velodyne
```
```bash
# コントローラ
sudo apt install ros-noetic-teleop-twist-joy
# LRF
sudo apt install ros-noetic-urg-node 
# LRFマージ
sudo apt install ros-noetic-ira-laser-tools 
# realsense-LRF変換
sudo apt install ros-noetic-depthimage-to-laserscan 
sudo apt install ros-noetic-pointcloud-to-laserscan 
# Gmapping:
sudo apt install ros-noetic-gmapping 
sudo apt install ros-noetic-slam-gmapping 
# navigation(map_server+amcl+move_base)
sudo apt install ros-noetic-navigation
# mrpt自己位置推定
sudo apt install ros-noetic-mrpt-navigation 
# move_baseの走行制御プラグイン(teb_local_planner)
sudo apt install ros-noetic-teb-local-planner 
# ロボットシミュレータ読込用
sudo apt install ros-noetic-xacro 
# roslibjsとの接続(rosbridge_server)
sudo apt install ros-noetic-rosbridge-suite 
# roslibjs上でtfを扱う(tf2_web_republisher)
sudo apt install ros-noetic-tf2-web-republisher 
# gazebo上でのvelodyneシミュレーション
sudo apt install ros-noetic-velodyne-description 
sudo apt install ros-noetic-velodyne-gazebo-plugins 
# iris_lama
sudo apt install ros-noetic-iris-lama-ros 
# slam_toolbox
sudo apt install ros-noetic-slam-toolbox
# velodyneドライバー
sudo apt install ros-noetic-velodyne
```