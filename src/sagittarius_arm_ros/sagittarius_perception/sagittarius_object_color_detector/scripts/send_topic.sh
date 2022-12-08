#!/bin/bash

Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Yellow_background_prefix="\033[43;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[info]${Font_color_suffix}"
echo -e "The process of ${Info} is as follows:"
echo -e "${Info} 1. The robot arm moves to a point, place the block directly under the robot arm"
echo -e "${Info} 2. The robotic arm moves to the grabbing position and obtains the position of the block on the image"
echo -e "${Info} 3. Move to a new calibration point, repeat steps 1~2 until all calibration points are calibrated"
echo -e "${Info} 4. Calculate the value after linear regression and save it to a file"

echo -e ""
echo -e "${Info} ready?\c"
echo && stty erase '^H' && read -p "Please enter Ctrl + c to exit, and press Enter to continue..."
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'start'" -1 >/dev/null
rostopic pub /cali_cmd_topic std_msgs/String "data: 'start'" -1 >/dev/null

echo -e ""
echo -e "${Info} (1/5) No.01"
echo -e "${Info} Waiting for the robot arm to move to the new position, please put the ${Blue_font_prefix} blue ${Font_color_suffix} square directly below the robot arm gripper\c"
echo && stty erase '^H' && read -p "Press Enter to continue..."
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

echo -e ""
echo -e "${Info} (2/5) No.02"
echo -e "${Info} Waiting for the robot arm to move to the new position, please put the ${Blue_font_prefix} blue ${Font_color_suffix} square directly below the robot arm gripper\c"
echo && stty erase '^H' && read -p "Press Enter to continue..."
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

echo -e ""
echo -e "${Info} (3/5) No.03"
echo -e "${Info} Waiting for the robot arm to move to the new position, please put the ${Blue_font_prefix} blue ${Font_color_suffix} square directly below the robot arm gripper\c"
echo && stty erase '^H' && read -p "Press Enter to continue..."
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

echo -e ""
echo -e "${Info} (4/5) No.04"
echo -e "${Info} Waiting for the robot arm to move to the new position, please put the ${Blue_font_prefix} blue ${Font_color_suffix} square directly below the robot arm gripper\c"
echo && stty erase '^H' && read -p "Press Enter to continue..."
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

echo -e ""
echo -e "${Info} (5/5) No.05"
echo -e "${Info} Waiting for the robot arm to move to the new position, please put the ${Blue_font_prefix} blue ${Font_color_suffix} square directly below the robot arm gripper\c"
echo && stty erase '^H' && read -p "Press Enter to continue..."
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

echo && stty erase '^H' && read -p "The calibration is over, press any key to exit..."
exit