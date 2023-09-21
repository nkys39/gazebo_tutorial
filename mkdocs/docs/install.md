# macOSでGazeboを動かす方法
macOSでGazeboを動かす方法は下記の4通りがあります。

1. macOSに直接インストールして動かす(homebrew)
2. 仮想マシン(UTM)上のUbuntuで動かす
3. macOSのDocker上のUbuntuで動かす
4. 仮想マシン(UTM)のDocker上のUbuntuで動かす

それぞれメリットとデメリットがあります。
||メリット|デメリット|
|---|---|---|
|①直接インストール|環境構築が簡単<br>パフォーマンスが良い|環境が汚れる<br>ROS/ROS2連携に接続手順が必要|
|②仮想マシン|参考情報が多い<br>USBデバイスが使える|Ubuntuの知識が必要<br>非力なPCだと動作が重い|
|③macOS上のDocker|移植性が高い<br>パフォーマンスが良い|Dockerの知識が必要<br>Ubuntuの知識が必要<br>USBデバイスが使えない|
|④仮想マシン上のdocker|移植性が高い<br>USBデバイスが使える|Dockerの知識が必要<br>Ubuntuの知識が必要<br>非力なPCだと動作が重い|

導入ハードルの高さは、①<②<③<④順で、特にDockerのGUIやGPU周りの設定が使用しているPCによって異なるため難易度が高いです。

③に関しては[こちら](https://github.com/Tiryoh/docker-ros-desktop-vnc)を使用することで環境依存を気にせずに実行できます。