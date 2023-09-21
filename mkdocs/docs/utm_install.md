# 仮想マシン(UTM)上のUbuntuで動かす

## 仮想マシンについて
仮想マシンは、OSから丸ごとシミュレーションし、別のOS環境を作ることができます。

仮想マシンのソフトウェアには有名なものとして下記があります。
- VMware
- Vurtualbox
- UTM

どれを使ってもよいのですが、Mac環境においてはUTMが最も性能が出るのでお勧めです。

## UTMのダウンロード先
下記からダウンロードしましょう

## CPUの違いによる注意点
Macには下記の2種類があり、CPUのアーキテクチャが異なるため注意が必要です。手持ちのPCがどちらか確認してからインストールする必要があります。
大抵のオープンソースの場合、AMD64用とARM64が用意されていますので間違えずに選びましょう。

- AMDベースのIntelチップ搭載のIntel_mac(古めのMac)
- ARMベースのM1/M2チップ搭載のM1/M2_mac(新しいMac)

## Ubuntuイメージのダウンロード先
- [ubuntu20.04_AMD(Intel_mac)イメージ](https://releases.ubuntu.com/20.04.6/?_ga=2.11851060.1856522629.1695269195-1682692205.1693276115)
- [ubuntu22.04_AMD(Intel_mac)イメージ](https://ubuntu.com/download/desktop)
- [ubuntu20.04_ARM(M1/M2_mac)イメージ](https://cdimage.ubuntu.com/releases/focal/release/)
- [ubuntu22.04_ARM(M1/M2_mac)イメージ](https://ubuntu.com/download/server/arm)
