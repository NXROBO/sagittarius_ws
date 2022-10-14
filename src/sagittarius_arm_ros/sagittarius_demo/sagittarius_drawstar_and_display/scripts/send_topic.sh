#!/bin/bash

echo -e "${Info} 请等待机械臂蜂鸣器停止鸣响后，按回车键开始程序，否则退出请输入：Ctrl + c    " 
echo -e "${Info} 如何显示运动轨迹？" 
echo -e "${Info} 1. 在 Rviz 的 Displays 中，找到 Motion Planning" 
echo -e "${Info} 2. 然后找到 Scene Robot >> Links >> sgr532/link_grasping_frame" 
echo -e "${Info} 3. 把 Show Trail 复选框勾上" 
echo && stty erase '^H' && read -p "按回车键确定：" 

rostopic pub /start_topic std_msgs/String "data: 'start'" -1
