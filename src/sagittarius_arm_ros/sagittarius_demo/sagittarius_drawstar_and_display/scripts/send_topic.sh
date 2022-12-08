#!/bin/bash

echo -e "${Info} Please wait for the buzzer of the robotic arm to stop beeping, and press the Enter key to start the program."
echo -e "${Info} Otherwise, to exit, please enter: Ctrl + c" 
echo -e "${Info} How to display motion track?" 
echo -e "${Info} 1. In Rviz GUI, find Motion Planning" 
echo -e "${Info} 2. Then find Scene Robot >> Links >> sgr532/link_grasping_frame" 
echo -e "${Info} 3. Check the Show Trail checkbox" 
echo && stty erase '^H' && read -p "Press Enter to confirm:" 

rostopic pub /start_topic std_msgs/String "data: 'start'" -1
