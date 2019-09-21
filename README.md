# 东南大学RoboCup2019校赛资料包说明

## 运行环境

* ubuntu 16.04
* ros-kinetic-desktop-full
* webots

**环境配置教程参见[EnvSetupGuide.md](https://github.com/Sciroccogti/SEURoboCup2019Kidsize/blob/master/EnvSetupGuide.md)**

> ubuntu安装可参照网上教程，[官方镜像](http://releases.ubuntu.com/16.04/ubuntu-16.04.6-desktop-amd64.iso)
>
> ros安装可参照[官方网站](http://wiki.ros.org/kinetic/Installation/Ubuntu)，
> ros入门可参照[bilibili古月居](https://www.bilibili.com/video/av59458869?from=search&seid=5767370996297806957)
>
> webots[官方下载地址](https://github.com/omichel/webots/releases/download/R2019b/webots_2019b_amd64.deb)（由于近期外网环境处于特殊时期，普通用户下载速度可能受限，如果遇到*技术问题*，请转至[KidsizeQQ群](https://jq.qq.com/?_wv=1027&k=55BwToG)下载）

## 使用步骤

1. 下载代码

    ```Bash
    cd ~
    git clone https://github.com/Sciroccogti/SEURoboCup2019Kidsize.git
    ```

2. 初始化

    ```Bash
    cd ~/SEURoboCup2019
    ./init.sh
    catkin_make
    ```

    如果中间报错，忽略即可

3. 创建自己的包

    ```Bash
    cd ~/SEURoboCup2019/src
    ./createPkg teamname
    # teamname为自己的队名
    ```

4. 修改自己的代码

    只需要修改自己创建的包内的代码

5. 启动

    * 打开一个新终端，执行下面命令

    ```Bash
    roslaunch simulation webots.launch
    ```

    * 打开一个新终端，执行下面命令

    ```Bash
    roslaunch simulation controller.launch
    ```

    * 打开一个新终端，执行下面命令

    ```Bash
    roslaunch game_ctrl game_ctrl.launch
    ```

    * 打开一个新终端，执行下面命令

    ```Bash
    roslaunch teamname robot_cpp.launch # run cxx
    roslaunch teamname robot_py.alunch  # run py
    # teamname为上面创建包时输入的名称
    # 第一条为运行c++控制程序，第二条为运行python控制程序，选择其中一个
    ```

## 推荐IDE

* Visual Studio Code
* VS Code插件
  * C/C++
  * Python
  * ROS
  * CMake
* 使用说明
    上述插件安装完毕后，使用VS Code打开工程文件夹，即可对工程进行修改，同时具备自动补全功能。

