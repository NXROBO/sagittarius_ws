# NXROBO Spark sagittarius_arm_ros


## Description
- This product is a trial version for beginners, a robotic arm based on six degrees of freedom and end grippers. 
Equipment that can be used to learn rviz's MoveIt.

<img src="https://raw.githubusercontent.com/NXROBO/sagittarius_ws/master/src/sagittarius_arm_ros/sdk_sagittarius_arm/picture/nxrobo_sagittarius.png" width="300" >

## Table of Contents

* [Packages](#Packages)
* [Usage](#Usage)
* [Video](#Video)

## Packages

* ***sagittarius_demo*** : Robotic Arm Demo
* ***sagittarius_descriptions*** : A package of description functions (Models) for the robotic arm.
* ***sagittarius_moveit*** : The MoveIt package for robotic arms.
* ***sagittarius_toolboxes*** : Basic toolbox for robotic arms.
* ***sak_sagittarius_arm*** : The SDK source code for the robotic arm.
* ***install.sh*** : Installation Script
## Usage

### Prerequisites

* System:	Ubuntu 16.04 ,Ubuntu 18.04 or Ubuntu 20.04
* ROS Version:	kinetic, melodic or noetic

### Download and Install
* Clone the workspace:
```yaml
    cd ~
    git clone https://github.com/NXROBO/sagittarius_ws.git
```
* Install libraries and dependencies:
```yaml
    cd sagittarius_ws
    ./onekey.sh
    # Select 103, then press Enter to install dependancies
    ./src/sagittarius_arm_ros/install.sh
```
### Compile and Run
```yaml
    cd ~/sagittarius_ws
    catkin_make
```
* If everything goes fine, test the examples as follows:
```yaml
    ./onekey.sh
    # Select any example from the list
    source devel/setup.bash
    roslaunch sagittarius_moveit demo_true.launch
```

## Video
TBA