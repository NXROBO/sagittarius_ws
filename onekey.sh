#!/usr/bin/env bash
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH

#=================================================
#	System Required: Ubuntu 20.04
#        litian.zhuang@nxrobo.com
#	Site: http://www.nxrobo.com/
#    技术讨论与反馈QQ群：6646169  8346256
#=================================================

sh_ver="2.0"
filepath=$(cd "$(dirname "$0")"; pwd)
Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Yellow_background_prefix="\033[43;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[INFO]${Font_color_suffix}"
Error="${Red_font_prefix}[ERROR]${Font_color_suffix}"
Warn="${Yellow_font_prefix}[WARN]${Font_color_suffix}"
Tip="${Green_font_prefix}[TIP]${Font_color_suffix}"
Separator_1="——————————————————————————————"
Version=$(lsb_release -r --short)
Codename=$(lsb_release -c --short)
OSDescription=$(lsb_release -d --short)
OSArch=$(uname -m)


# Check System Requirements
check_sys(){
		if [[ "${Version}" == "16.04" ]]; then
                ROS_Ver="kinetic"
        elif [[ "${Version}" == "18.04" ]]; then
                ROS_Ver="melodic"
        elif [[ "${Version}" == "20.04" ]]; then
                ROS_Ver="noetic"
        else
                echo -e "${Error} sagittarius does not support the current system ${OSDescription} !" && exit 1
        fi
}

# Check device connection
check_dev(){
	# Check the arm is connected via USB
	if [ ! -n "$(lsusb -d 2e88:4603)" ]; then
		echo -e "${Error} The usb of the robotic arm is not connected correctly!"
	fi
}

# Detect if a full version needs to be installed
check_install_ros_full(){
	if [ -f "/usr/bin/rosversion" ]; then
		ROSVER=`/usr/bin/rosversion -d`
		if [ $ROSVER ]; then
			echo -e "${Tip} Full ROS installation detected: ${ROSVER}" 
		fi
	else
		echo -e "${Tip} No ROS installation's detected! Please install Full ROS version first"
	fi 
}

#Install the sagittarius dependency library
install_sagittarius_require(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	echo -e "${Info} Ready to install sagittarius related drivers..."
	${PROJECTPATH}/src/sagittarius_arm_ros/install.sh
	echo -e "${Info} Completing the dependency library installation..."
}

# Compile Sagittarius
compile_sagittarius(){
	source /opt/ros/${ROS_Ver}/setup.bash
	catkin_make
	#catkin_make install
}

# Remote Settings
master_uri_setup(){
	eth_ip=`/sbin/ifconfig eth0|grep 'inet '|awk '{print $2}'`
	wlp1s_ip=`/sbin/ifconfig wlp1s0|grep 'inet '|awk '{print $2}'`
	wlp2s_ip=`/sbin/ifconfig wlp2s0|grep 'inet '|awk '{print $2}'`
	wlan_ip=`/sbin/ifconfig wlan0|grep 'inet '|awk '{print $2}'`
        enp3s_ip=`/sbin/ifconfig enp3s0|grep 'inet '|awk '{print $2}'`
	if [ $eth_ip ]; then
		echo -e "${Info} Use wired network eth0" 
		local_ip=$eth_ip
	elif [ $wlp1s_ip ]; then
		echo -e "${Info} Use wireless network wlp1s0" 
	  	local_ip=$wlp1s_ip
	elif [ $wlp2s_ip ]; then
		echo -e "${Info} Use wireless network wlp2s0" 
	  	local_ip=$wlp2s_ip
	elif [ $wlan_ip ]; then
		echo -e "${Info} Use wireless network wlan0" 
	  	local_ip=$wlan_ip
        elif [ $enp3s_ip ]; then
                echo -e "${Info} Use wireless network enp3s0" 
                local_ip=$enp3s_ip	
	fi
	export ROS_HOSTNAME=$local_ip
	export ROS_MASTER_URI="http://${local_ip}:11311"
	echo -e "${Info}Using ROS MASTER at ${Red_font_prefix}$ROS_MASTER_URI${Font_color_suffix} from ${Red_font_prefix}$ROS_HOSTNAME${Font_color_suffix}"
}

print_command()
{
	echo -e "${Yellow_background_prefix}${Red_font_prefix}${1}${Font_color_suffix}"
}

# start_sagittarius_moveit
start_sagittarius_moveit(){

	echo -e "${Info}" 
	echo -e "${Info} Start the ROS MOVEIT interface" 
	echo -e "${Info} To exit, enter: Ctrl + C        "
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo && stty erase ^? && read -p "Press Enter to continue:" 
	print_command "roslaunch sagittarius_moveit demo_true.launch"
	roslaunch sagittarius_moveit demo_true.launch
}

# Forward_kinematics
Forward_kinematics(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info} Forward Kinematics example" 
	echo -e "${Info}" 
	echo -e "${Info} Options:
	  ${Green_font_prefix}1.${Font_color_suffix} View Source Code
	  ${Green_font_prefix}2.${Font_color_suffix} Run the Example
	  ${Green_font_prefix}3.${Font_color_suffix} To exit, enter: Ctrl + c" 
	echo && stty erase ^? && read -p "Please enter [1-3]:" asrnum
	case "$asrnum" in
		1)
		cat  ${PROJECTPATH}/src/sagittarius_arm_ros/sdk_sagittarius_arm/scripts/Forward_kinematics.py
		;;
		2)
		print_command "roslaunch sdk_sagittarius_arm Forward_kinematics.launch"
		roslaunch sdk_sagittarius_arm Forward_kinematics.launch
		;;
		*)
		echo -e "${Error} Error, Exiting"
		;;
	esac
}

#Inverse_kinematics
Inverse_kinematics(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info} Inverse Kinematics example" 
	echo -e "${Info}" 
	echo -e "${Info} Options:
	  ${Green_font_prefix}1.${Font_color_suffix} View Source Code
	  ${Green_font_prefix}2.${Font_color_suffix} Run the Example
	  ${Green_font_prefix}3.${Font_color_suffix} To exit, enter: Ctrl + c" 
	echo && stty erase ^? && read -p "Please enter [1-3]:" asrnum
	case "$asrnum" in
		1)
		cat  ${PROJECTPATH}/src/sagittarius_arm_ros/sdk_sagittarius_arm/scripts/Inverse_kinematics.py
		;;
		2)
		print_command "roslaunch sdk_sagittarius_arm Inverse_kinematics.launch"
		roslaunch sdk_sagittarius_arm Inverse_kinematics.launch
		;;
		*)
		echo -e "${Error} Error, Exiting"
		;;
	esac
}

#control_single_servo
control_single_servo(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info} Single Servo control" 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "Press Enter to continue:" 
	print_command "roslaunch sdk_sagittarius_arm rviz_control_sagittarius.launch"
	roslaunch sdk_sagittarius_arm rviz_control_sagittarius.launch
}

#rosbag record and play
rosbag_record_play(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info} Rosbag recording and playback" 
	echo -e "${Info}" 
	echo -e "${Info} Options:
	  ${Green_font_prefix}1.${Font_color_suffix} record robotic arm movements
	  ${Green_font_prefix}2.${Font_color_suffix} play recorded robotic arm movements
	  ${Green_font_prefix}3.${Font_color_suffix} To exit, enter: Ctrl + c" 
	echo && stty erase ^? && read -p "Please enter [1-3]:" asrnum
	case "$asrnum" in
		1)
		rosbag_record
		;;
		2)
		rosbag_play
		;;
		*)
		echo -e "${Error} Error, Exiting"
		;;
	esac
}

#rosbag record 
rosbag_record(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info} record robotic arm movements with rosbag" 
	echo -e "${Info} Please hold the robot arm to the initial position with your hand," 
	echo -e "${Info} After starting, move the joints of the robotic arm," 
	echo -e "${Info} When finished recording, press Ctrl+C to exit & autosave"
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "Press Enter to continue:" 
	print_command "roslaunch sagittarius_puppet_control puppet_control_single.launch bag_name:=record1"
	roslaunch sagittarius_puppet_control puppet_control_single.launch bag_name:="record1"
}

#rosbag_play
rosbag_play(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info} play recorded robotic arm movements with rosbag" 
	echo -e "${Info} Please use caution!" 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "Press Enter to continue:" 
	print_command "roslaunch sagittarius_puppet_control rosbag_play.launch "
	roslaunch sagittarius_puppet_control rosbag_play.launch
}

#puppet_control
puppet_control(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info} Real-time reproduction of multi-arm movements" 
	echo -e "${Info} Please pay attention to keep the space distance between each robot arm" 
	echo -e "${Info} Ensure that the serial port number of the main robot arm is /dev/ttyACM0" 	
	echo -e "${Info} That is, the USB cable of the main robotic arm is first connected to the host" 	
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "Please enter the number of arms [2-6], press Enter to continue:" asrnum
	case "$asrnum" in
		2)
		print_command "roslaunch sagittarius_puppet_control puppet_control_s1.launch"
		roslaunch sagittarius_puppet_control puppet_control_s1.launch 
		;;
		3)
		print_command "roslaunch sagittarius_puppet_control puppet_control_s2.launch"
		roslaunch sagittarius_puppet_control puppet_control_s2.launch 
		;;
		4)
		print_command "roslaunch sagittarius_puppet_control puppet_control_s3.launch"
		roslaunch sagittarius_puppet_control puppet_control_s3.launch 
		;;
		5)
		print_command "roslaunch sagittarius_puppet_control puppet_control_s4.launch"
		roslaunch sagittarius_puppet_control puppet_control_s4.launch 
		;;
		6)
		print_command "roslaunch sagittarius_puppet_control puppet_control_s5.launch"
		roslaunch sagittarius_puppet_control puppet_control_s5.launch 
		;;
		*)
		echo -e "${Error} Error, only 2-6 robotic arms are supported, and 2 robotic arms are run by default"
		print_command "roslaunch sagittarius_puppet_control puppet_control_s1.launch"
		roslaunch sagittarius_puppet_control puppet_control_s1.launch 
		;;
	esac
}

drawstar_and_display(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info} Synchronous simulation display of manipulator drawing five-pointed star" 
	echo -e "${Info} After the program starts, a new terminal will be opened, and when it needs to start running" 
	echo -e "${Info} press the Enter key in the new terminal"
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "Press Enter to continue:" 
	print_command "roslaunch sagittarius_drawstar_and_display moveit_draw_star.launch"
	roslaunch sagittarius_drawstar_and_display moveit_draw_star.launch
}

#2d-object_color_detection_and_grab
2d-object_color_detection_and_grab(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	if [ ! -n "$(lsusb -d 0c45:64ab)" ]; then
		echo -e "${Error} Camera not found. Check whether the camera USB data cable is connected correctly to the PC"
	else
		echo -e "${Info}" 
		echo -e "${Info} Color recognition capture" 
		echo -e "${Info}" 
		echo -e "${Info} Repeated capture is possible without changing the capture scene" 	
		echo -e "${Info} After changing the capture scene, you need to set the HSV value and re-calibrate" 
		echo -e "${Info}" 
		echo -e "${Info} Camera has been detected and in use"
		echo -e "${Info} Options:
	${Green_font_prefix}1.${Font_color_suffix} Set item HSV value
	${Green_font_prefix}2.${Font_color_suffix} Hand-Eye Calibration
	${Green_font_prefix}3.${Font_color_suffix} Take a sample crawl
	${Green_font_prefix}4.${Font_color_suffix} Color classification (color placement is fixed)
	${Green_font_prefix}5.${Font_color_suffix} Color classification (cooperate with the map to find placements)
	${Green_font_prefix}6.${Font_color_suffix} To exit, enter: Ctrl + c" 
		echo && stty erase ^? && read -p "Please enter [1-5]:" asrnum
		case "$asrnum" in
			1)
			echo -e "${Info} Set item HSV value"
			echo -e "${Info} Use rqt_reconfigure to set ${Red_font_prefix}red ${Font_color_suffix}, ${Green_font_prefix}green ${Font_color_suffix}, ${Blue_font_prefix} blue${Font_color_suffix} HSV values of the block"
			echo -e "${Info} The dynamic parameter server is named HSVParams_node"
			echo -e "${Info} After the setting is completed, press Ctrl+C on the terminal to exit the program, and the program will automatically save the HSV value"
			echo && stty erase ^? && read -p "Press Enter to continue:" foonum
			print_command "roslaunch sagittarius_object_color_detector hsv_params.launch"
			roslaunch sagittarius_object_color_detector hsv_params.launch
			;;
			2)
			echo -e "${Info} Hand-Eye calibration"
			echo -e "${Info} Follow the next prompts to calibrate"
			echo -e "${Info} default: ${Blue_font_prefix}blue ${Font_color_suffix} box"
			echo && stty erase ^? && read -p "Press Enter to continue:" foonum
			print_command "roslaunch sagittarius_object_color_detector camera_calibration_hsv.launch "
			roslaunch sagittarius_object_color_detector camera_calibration_hsv.launch 
			;;
			3)
			echo -e "${Info} Example of a grab by a robotic arm"
			echo -e "${Info} Put a block in the center of the recognition area, and the robotic arm will automatically grab and place it in the R area"
			echo -e "${Info} Do not put debris in the R area, the default recognition ${Blue_font_prefix} blue${Font_color_suffix} box"
			echo && stty erase ^? && read -p "Press Enter to continue:" foonum
			print_command "roslaunch sagittarius_object_color_detector object_pick.launch"
			roslaunch sagittarius_object_color_detector object_pick.launch
			;;
			4)
			echo -e "${Info} Color sort blocks (placement fixed)"
			echo -e "${Info} The robotic arm recognizes, grabs and sorts the blocks, and puts them in a fixed place"
			echo -e "${Info} sort the container by color ${Red_font_prefix} red${Font_color_suffix}, ${Green_font_prefix}, green${Font_color_suffix}, ${Blue_font_prefix} blue${Font_color_suffix} Fasten them in order, and when facing the front of the robot arm, put them in a row on the right side of the identification area"
			echo -e "${Info} The placement position of the robot arm is fixed, please adjust the position of the container according to the placement position"
			echo && stty erase ^? && read -p "Press Enter to continue:" foonum
			print_command "roslaunch sagittarius_object_color_detector color_classification_fixed.launch"
			roslaunch sagittarius_object_color_detector color_classification_fixed.launch
			;;
			5)
			echo -e "${Info} Sort blocks by color (cooperate with the map to find placements)"
			echo -e "${Info} Place the drop container arbitrarily on the ABCD area of the map, the robotic arm grabs the block and drops the container of the corresponding color"
			echo -e "${Info} The robotic arm determines the delivery area by identifying the color of the container, do not cover the color of the container"
			echo && stty erase ^? && read -p "Press Enter to continue:" foonum
			print_command "roslaunch sagittarius_object_color_detector color_classification.launch"
			roslaunch sagittarius_object_color_detector color_classification.launch
			;;
			*)
			echo -e "${Error} Error, Exiting"
			;;
		esac
	fi
}

#3d_perception_detection_and_grab
3d_perception_detection_and_grab(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	if [ -n "$(roscd sagittarius_perception)" ]; then
		echo -e "${Info} "
	elif [ ! -n "$(lsusb -d 8086:0b07)" ]; then
		echo -e "${Error} The high-end version of the vision kit function package was not found. If you need a high-end version, please contact our marketing department"
	else
		echo -e "${Info}" 
		echo -e "${Info} 3D visual capture (eye-to-hand)" 
		echo -e "${Info}" 
		echo -e "${Info} The camera is placed where the robot arm can see the QR code and the recognition area at the same time"
		echo -e "${Info} Do not move the camera after calibration" 
		echo -e "${Info}" 
		echo -e "${Info} Please select the relevant option:
	${Green_font_prefix}1.${Font_color_suffix} hand-eye calibration (eye-to-hand)
	${Green_font_prefix}2.${Font_color_suffix} Color Grabbing Classification
	${Green_font_prefix}3.${Font_color_suffix} To exit, enter: Ctrl + c" 
		echo && stty erase ^? && read -p "Please Select [1-2]:" asrnum
		case "$asrnum" in
			1)
			echo -e "${Info}" 
			echo -e "${Info} Interface ArmTag Tuner GUI for calibration" 
			echo -e "${Info} Interface PointCloud Tuner GUI for tuning parameters of object recognition" 
			echo -e "${Info} To exit, enter: Ctrl + c" 
			echo -e "${Info}" 
			echo && stty erase ^? && read -p "Press Enter to continue:" 
			print_command "roslaunch sagittarius_perception calibrate.launch"
			roslaunch sagittarius_perception calibrate.launch
			;;
			2)
			echo -e "${Info}" 
			echo -e "${Info} Please place the object.." 
			echo -e "${Info} To exit, enter: Ctrl + c" 
			echo -e "${Info}" 
			echo && stty erase ^? && read -p "Press Enter to continue:" 
			print_command "roslaunch sagittarius_perception demo_color_classifcation.launch"
			roslaunch sagittarius_perception demo_color_classifcation.launch
			;;
			*)
			echo -e "${Error} Error, Exiting.."
			;;
		esac
	fi
}

qrcode_transfer_files(){
	eno1_ip=`/sbin/ifconfig eno1|grep 'inet '|awk '{print $2}'`
	eth_ip=`/sbin/ifconfig eth0|grep 'inet '|awk '{print $2}'`
	wlp1s_ip=`/sbin/ifconfig wlp1s0|grep 'inet '|awk '{print $2}'`
	wlp2s_ip=`/sbin/ifconfig wlp2s0|grep 'inet '|awk '{print $2}'`
	wlan_ip=`/sbin/ifconfig wlan0|grep 'inet '|awk '{print $2}'`
	enp3s_ip=`/sbin/ifconfig enp3s0|grep 'inet '|awk '{print $2}'`
	wlp0s_ip=`/sbin/ifconfig wlp0s20f3|grep 'inet '|awk '{print $2}'`
	wlo1_ip=`/sbin/ifconfig wlo1|grep 'inet '|awk '{print $2}'`
	if [ $wlp1s_ip ]; then
		echo -e "${Info} Using wireless network wlp1s0" 
	  	net_interface="wlp1s0"
	elif [ $wlo1_ip ]; then
		echo -e "${Info} Using wireless network wlo1" 
	  	net_interface="wlo1"	  	
	elif [ $wlp2s_ip ]; then
		echo -e "${Info} Using wireless network wlp2s0" 
	  	net_interface="wlp2s0"
	elif [ $wlan_ip ]; then
		echo -e "${Info} Using wireless network wlan0" 
	  	net_interface="wlan0"
	elif [ $enp3s_ip ]; then
		echo -e "${Info} Using wireless network enp3s0" 
		net_interface="enp3s0"
	elif [ $wlp0s_ip ]; then
		echo -e "${Info} Using wireless network wlp0s20f3" 
		net_interface="wlp0s20f3"		
	elif [ $eth_ip ]; then
		echo -e "${Info} Using wired network eth0" 
		net_interface="eth0"
	elif [ $eno1_ip ]; then
		echo -e "${Info} Using wireless network eno1" 
		net_interface="eno1"
	fi	
	
	echo -e "${Info}" 
	echo -e "${Info} Send and receive files via LAN" 
	echo -e "${Info}" 
	echo -e "${Info} Options:
	  ${Green_font_prefix}1.${Font_color_suffix} Send file (file name, bring the absolute path of the file)
	  ${Green_font_prefix}2.${Font_color_suffix} Receive files (stored in ~/Downloads path by default)
	  ${Green_font_prefix}3.${Font_color_suffix} Press Enter to continuTo exit, enter: Ctrl + c"
	echo && stty erase ^? && read -p "Please enter [1-2]:" cnum
	case "$cnum" in
		1)
		echo -e "${Info} Please enter the file name with the absolute path of the file, such as /home/${USER}/a.jpg:
		  Press Enter to continuTo exit, enter: Ctrl + c"
		echo && stty erase ^? && read -p "Please enter the file to send:" s_file
		if [ -f "$s_file" ]; then
			echo -e "${Info} The machine is about to send the file:${Green_font_prefix}"$s_file"${Font_color_suffix}, please scan the code at the receiving end or directly enter the URL below to receive the file"
		else 
			echo -e "${Info} Please enter a file name with an absolute path"
			exit
		fi
		
		qrcp send  -i $net_interface $s_file
		;;
		2)
		echo -e "${Info} Please enter the path where the received file is stored, the default is /home/${USER}/Downloads:
		  Press Enter to continuTo exit, enter: Ctrl + c"
		echo && stty erase ^? && read -p "Please enter the folder path where the file is stored:" s_file
		if [ -d "$s_file" ]; then
			echo ""
		else 
			echo -e "${Info}${Red_font_prefix} The folder does not exist, it will be stored in the default folder /home/${USER}/Downloads${Font_color_suffix}"
			s_file="/home/${USER}/Downloads"
		fi
		echo -e "${Info} Received documents will be deposited at:${Green_font_prefix}"$s_file"${Font_color_suffix}, directory, please scan the QR code at the sending end or directly enter the URL below to select the file to send"
		qrcp  -i $net_interface receive --output=$s_file
		;;
		*)
		echo -e "${Error} Error, Exiting"
		;;
	esac
}

coming_soon(){
	echo -e "${Tip} coming_soon!" 
}


#printf
menu_status(){
	echo -e "${Tip} current system version ${OSDescription} !" 
	if [ -f "/usr/bin/rosversion" ]; then
		ROSVER=`/usr/bin/rosversion -d`
		if [ $ROSVER ]; then
			echo -e "${Tip} Current ROS version ${ROSVER} !"
			return
		fi 
	fi
	echo -e "${Error} No ROS installation detected! Please install Full ROS version first" 
}

tell_us(){
	echo -e ""
	echo -e "${Tip} ----------------------------------" 
	echo -e "${Tip} 网址：www.nxrobo.com" 
	echo -e "${Tip} ROS技术讨论与反馈QQ群：一群 8346256(已满)；二群 6646169" 
	echo -e "${Tip} ---------QQ扫描加入我们--------------"
	echo 'https://jq.qq.com/?_wv=1027&k=1JV8oyB8'|qrencode -o - -t UTF8
	echo -e ""
}

qrcode_picture()
{
	echo 'www.NXROBO.com'|qrencode -o - -t UTF8
}
check_sys
echo -e "————————————
  Sagittarius -- One-click management script"
qrcode_picture

echo -e "  
  Note: Steps 101~103 are the installation and settings of the relevant environment. 
  If it has been executed, do not repeat it.
  
  Please select the desired function

  ${Green_font_prefix}  0.${Font_color_suffix} Compile seperately
————————————
  ${Green_font_prefix}  1.${Font_color_suffix} Start the MoveIt interface
  ${Green_font_prefix}  2.${Font_color_suffix} Forward Kinematics Example
  ${Green_font_prefix}  3.${Font_color_suffix} Inverse Kinematics Example
  ${Green_font_prefix}  4.${Font_color_suffix} Single Servo Control
  ${Green_font_prefix}  5.${Font_color_suffix} Recording and playback of robotic arm movements using Rosbag
  ${Green_font_prefix}  6.${Font_color_suffix} Real-time reproduction of multi-arm movements
  ${Green_font_prefix}  7.${Font_color_suffix} Synchronous simulation display of manipulator drawing five-pointed star (requires sudo)
  ${Green_font_prefix}  8.${Font_color_suffix} Visualisation, Planning and object manipulation via Arm Camera\c"


echo -e "
————————————

  ${Green_font_prefix}100.${Font_color_suffix} Provide Feedback
  ${Green_font_prefix}103.${Font_color_suffix} Install dependent libraries
  ${Green_font_prefix}104.${Font_color_suffix} File transfer
 "
menu_status
check_dev
echo && stty erase ^? && read -p "Enter the number of your selection: " num
case "$num" in
	0)
	compile_sagittarius	
	;;
	1)
	start_sagittarius_moveit
	;;
	2)
	Forward_kinematics
	;;
	3)
	Inverse_kinematics
	;;
	4)
	control_single_servo
	;;
	5)
	rosbag_record_play
	;;
	6)
	puppet_control
	;;
	7)
	drawstar_and_display
	;;
	8)
	2d-object_color_detection_and_grab
	;;
	9)
	3d_perception_detection_and_grab
	;;
	100)
	tell_us
	;;
	103)
	install_sagittarius_require
	;;
	104)
	qrcode_transfer_files
	;;	
	*)
	echo -e "${Error} Please enter a correct number "
	;;
esac

