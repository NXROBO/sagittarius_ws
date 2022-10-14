#!/bin/bash

Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Yellow_background_prefix="\033[43;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
echo -e "${Info} 流程如下："
echo -e "${Info} 1. 机械臂移动到一个点，放置方块到机械臂的正下方"
echo -e "${Info} 2. 机械臂移动到抓取位置，获取方块在图像上的位置"
echo -e "${Info} 3. 移动到新的标定点，重复执行1~2步骤，直到所有标定点标定完成"
echo -e "${Info} 4. 计算线性回归后的值，并保存到文件"

echo -e ""
echo -e "${Info} 一切准备好？\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c，完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'start'" -1 >/dev/null
rostopic pub /cali_cmd_topic std_msgs/String "data: 'start'" -1 >/dev/null

echo -e ""
echo -e "${Info} (1/5) No.01"
echo -e "${Info} 等待机械臂运动到新位置后，请把${Blue_font_prefix}蓝色${Font_color_suffix}方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "放置好后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

echo -e ""
echo -e "${Info} (2/5) No.02"
echo -e "${Info} 等待机械臂运动到新位置后，请把${Blue_font_prefix}蓝色${Font_color_suffix}方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "放置好后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

echo -e ""
echo -e "${Info} (3/5) No.03"
echo -e "${Info} 等待机械臂运动到新位置后，请把${Blue_font_prefix}蓝色${Font_color_suffix}方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "放置好后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

echo -e ""
echo -e "${Info} (4/5) No.04"
echo -e "${Info} 等待机械臂运动到新位置后，请把${Blue_font_prefix}蓝色${Font_color_suffix}方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "放置好后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

echo -e ""
echo -e "${Info} (5/5) No.05"
echo -e "${Info} 等待机械臂运动到新位置后，请把${Blue_font_prefix}蓝色${Font_color_suffix}方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "放置好后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

echo && stty erase '^H' && read -p "标定结束，按任意键退出..." 
exit