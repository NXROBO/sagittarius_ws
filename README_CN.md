# NXROBO Spark sagittarius_arm_ros


## 说明 Description
- 本产品为初学者体验版，一个基于六个自由度和末端夹具的机械臂。可以用来学习moveit的设备。
- <img src="https://raw.githubusercontent.com/NXROBO/sagittarius_ws/master/src/sagittarius_arm_ros/sdk_sagittarius_arm/picture/nxrobo_sagittarius.png" width="300">

## 列表 Table of Contents

* [功能包说明packages-overview](#功能包说明packages-overview)
* [使用usage](#使用usage)
* [视频展示Video](#视频展示Video)

## 功能包说明packages-overview

* ***sagittarius_demo*** : 机械臂的DEMO。
* ***sagittarius_descriptions*** : 机械臂的描述功能包。
* ***sagittarius_moveit*** : 机械臂的moveit功能包。
* ***sagittarius_toolboxes*** : 机械臂的基础工具箱。
* ***sak_sagittarius_arm*** : 机械臂的SDK源码。
* ***install.sh*** : 安装脚本。
## 使用usage

### 系统要求 Prequirement

* System:	Ubuntu 16.04 ,Ubuntu 18.04 or Ubuntu 20.04
* ROS Version:	kinetic, melodic or noetic

### 下载安装 Download and install
* 下载工作空间 Download the workspace:
```yaml
    cd ~
    git clone https://github.com/NXROBO/sagittarius_ws.git
```
* 安装依赖库 Install libraries and dependencies:
```yaml
    cd sagittarius_ws
    ./onekey.sh
    根据提示选择 103 ，然后回车键进行安装，或者执行
    ./src/sagittarius_arm_ros/install.sh
```
### 编译运行 compile and run
```yaml
    cd ~/sagittarius_ws
    catkin_make
```
* 如果编译一切正常，可根据提示运行相关例程。If everything goes fine, test the examples as follow:
```yaml
    ./onekey.sh
    根据提示选择相应功能的序号。或者相关功能的启动命令，如：
    source devel/setup.bash
    roslaunch sagittarius_moveit demo_true.launch
```

## 视频展示Video
