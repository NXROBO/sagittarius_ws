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
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
Error="${Red_font_prefix}[错误]${Font_color_suffix}"
Warn="${Yellow_font_prefix}[警告]${Font_color_suffix}"
Tip="${Green_font_prefix}[注意]${Font_color_suffix}"
Separator_1="——————————————————————————————"
Version=$(lsb_release -r --short)
Codename=$(lsb_release -c --short)
OSDescription=$(lsb_release -d --short)
OSArch=$(uname -m)


#检查系统要求
check_sys(){
		if [[ "${Version}" == "16.04" ]]; then
                ROS_Ver="kinetic"
        elif [[ "${Version}" == "18.04" ]]; then
                ROS_Ver="melodic"
        elif [[ "${Version}" == "20.04" ]]; then
                ROS_Ver="noetic"
        else
                echo -e "${Error} sagittarius不支持当前系统 ${OSDescription} !" && exit 1
        fi
}

#检查设备连接
check_dev(){
	#检查机械臂
	if [ ! -n "$(lsusb -d 2e88:4603)" ]; then
		echo -e "${Error} 机械臂usb没有正确连接，请确认正确连接！！"
	fi
}

#检测是否需要安装完整版
check_install_ros_full(){
	if [ -f "/usr/bin/rosversion" ]; then
		ROSVER=`/usr/bin/rosversion -d`
		if [ $ROSVER ]; then
			echo -e "${Tip} 检测到当前系统已安装了ROS的${ROSVER}版本!" 
		fi
	else
		echo -e "${Tip} 检测到未安装任何ROS版本!请先安装ROS"
	fi 
}

#安装sagittarius依赖库
install_sagittarius_require(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	echo -e "${Info} 准备安装sagittarius相关驱动……"
	${PROJECTPATH}/src/sagittarius_arm_ros/install.sh
	echo -e "${Info} 完成依赖库安装……"
}


#编译sagittarius
compile_sagittarius(){
	source /opt/ros/${ROS_Ver}/setup.bash
	catkin_make
	#catkin_make install
}




#远程设置
master_uri_setup(){
	eth_ip=`/sbin/ifconfig eth0|grep 'inet '|awk '{print $2}'`
	wlp1s_ip=`/sbin/ifconfig wlp1s0|grep 'inet '|awk '{print $2}'`
	wlp2s_ip=`/sbin/ifconfig wlp2s0|grep 'inet '|awk '{print $2}'`
	wlan_ip=`/sbin/ifconfig wlan0|grep 'inet '|awk '{print $2}'`
        enp3s_ip=`/sbin/ifconfig enp3s0|grep 'inet '|awk '{print $2}'`
	if [ $eth_ip ]; then
		echo -e "${Info}使用有线网络eth0" 
		local_ip=$eth_ip
	elif [ $wlp1s_ip ]; then
		echo -e "${Info}使用无线网络wlp1s0" 
	  	local_ip=$wlp1s_ip
	elif [ $wlp2s_ip ]; then
		echo -e "${Info}使用无线网络wlp2s0" 
	  	local_ip=$wlp2s_ip
	elif [ $wlan_ip ]; then
		echo -e "${Info}使用无线网络wlan0" 
	  	local_ip=$wlan_ip
        elif [ $enp3s_ip ]; then
                echo -e "${Info}使用无线网络enp3s0" 
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

#start_sagittarius_moveit
start_sagittarius_moveit(){

	echo -e "${Info}" 
	echo -e "${Info} 启动ROS MOVEIT界面！" 
	echo -e "${Info} 退出请输入：Ctrl + c        "
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo && stty erase ^? && read -p "按回车键（Enter）继续：" 
	print_command "roslaunch sagittarius_moveit demo_true.launch"
	roslaunch sagittarius_moveit demo_true.launch
}

#Forward_kinematics
Forward_kinematics(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}正运动学示例" 
	echo -e "${Info}" 
	echo -e "${Info}请选择相关选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 只查看源代码
	  ${Green_font_prefix}2.${Font_color_suffix} 只运行示例
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-3]：" asrnum
	case "$asrnum" in
		1)
		cat  ${PROJECTPATH}/src/sagittarius_arm_ros/sdk_sagittarius_arm/scripts/Forward_kinematics.py
		;;
		2)
		print_command "roslaunch sdk_sagittarius_arm Forward_kinematics.launch"
		roslaunch sdk_sagittarius_arm Forward_kinematics.launch
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}

#Inverse_kinematics
Inverse_kinematics(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}逆运动学示例" 
	echo -e "${Info}" 
	echo -e "${Info}请选择相关选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 只查看源代码
	  ${Green_font_prefix}2.${Font_color_suffix} 只运行示例
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-3]：" asrnum
	case "$asrnum" in
		1)
		cat  ${PROJECTPATH}/src/sagittarius_arm_ros/sdk_sagittarius_arm/scripts/Inverse_kinematics.py
		;;
		2)
		print_command "roslaunch sdk_sagittarius_arm Inverse_kinematics.launch"
		roslaunch sdk_sagittarius_arm Inverse_kinematics.launch
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}
#control_single_servo
control_single_servo(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}通过界面控制单个舵机运动" 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）继续：" 
	print_command "roslaunch sdk_sagittarius_arm rviz_control_sagittarius.launch"
	roslaunch sdk_sagittarius_arm rviz_control_sagittarius.launch
}
#rosbag record and play
rosbag_record_play(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}rosbag录制与播放" 
	echo -e "${Info}" 
	echo -e "${Info}请选择相关选择：
	  ${Green_font_prefix}1.${Font_color_suffix} rosbag录制机械臂动作
	  ${Green_font_prefix}2.${Font_color_suffix} rosbag播放录制的机械臂动作
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-3]：" asrnum
	case "$asrnum" in
		1)
		rosbag_record
		;;
		2)
		rosbag_play
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}
#rosbag record 
rosbag_record(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}rosbag录制机械臂动作" 
	echo -e "${Info}请用手扶位机械臂至初始位置，" 
	echo -e "${Info}启动后移动机械臂各个关节，" 
	echo -e "${Info}完成录制后，按Ctrl + c退出，自动保存，" 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）继续：" 
	print_command "roslaunch sagittarius_puppet_control puppet_control_single.launch bag_name:=record1"
	roslaunch sagittarius_puppet_control puppet_control_single.launch bag_name:="record1"

}
#rosbag_play
rosbag_play(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}rosbag播放录制的机械臂动作" 
	echo -e "${Info}请注意安全" 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）继续：" 
	print_command "roslaunch sagittarius_puppet_control rosbag_play.launch "
	roslaunch sagittarius_puppet_control rosbag_play.launch

}

#puppet_control
puppet_control(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}多机械臂动作实时复现" 
	echo -e "${Info}请注意保持各个机械臂的空间距离" 
	echo -e "${Info}请确保主机械臂的串口号为/dev/ttyACM0" 	
	echo -e "${Info}即主机械臂的USB线最先与主机相连" 	
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "请输入机械臂的数量 [2-6]，按回车键（Enter）继续：" asrnum
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
		echo -e "${Error} 错误，只支持2~6个机械臂，默认运行2个机械臂"
		print_command "roslaunch sagittarius_puppet_control puppet_control_s1.launch"
		roslaunch sagittarius_puppet_control puppet_control_s1.launch 
		;;
	esac
}

drawstar_and_display(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}" 
	echo -e "${Info} 机械臂画五角星同步仿真显示" 
	echo -e "${Info} 程序启动后会新开一个终端，需要开始运行时在新终端按回车键" 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）继续：" 
	print_command "roslaunch sagittarius_drawstar_and_display moveit_draw_star.launch"
	roslaunch sagittarius_drawstar_and_display moveit_draw_star.launch
}

#2d-object_color_detection_and_grab
2d-object_color_detection_and_grab(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	if [ ! -n "$(lsusb -d 0c45:64ab)" ]; then
		echo -e "${Error}没有找到臂上摄像头，检查摄像头数据线是否正确连接"
	else
		echo -e "${Info}" 
		echo -e "${Info}颜色识别抓取 (Eye In Hand)" 
		echo -e "${Info}" 
		echo -e "${Info}抓取场景不变情况下可重复抓取" 	
		echo -e "${Info}更换抓取场景后需要设置 HSV 值与重新标定" 
		echo -e "${Info}" 
		echo -e "${Info}正在使用臂上摄像头"
		echo -e "${Info}请选择相关选择：
	${Green_font_prefix}1.${Font_color_suffix} 设置物品 HSV 值
	${Green_font_prefix}2.${Font_color_suffix} 手眼标定 (Eye In Hand)
	${Green_font_prefix}3.${Font_color_suffix} 进行一次抓取示例
	${Green_font_prefix}4.${Font_color_suffix} 颜色分类 (颜色投放位置固定)
	${Green_font_prefix}5.${Font_color_suffix} 颜色分类 (配合地图寻找投放位置)
	${Green_font_prefix}6.${Font_color_suffix} 退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入数字 [1-5]：" asrnum
		case "$asrnum" in
			1)
			echo -e "${Info} 设置物品 HSV 值"
			echo -e "${Info} 使用 rqt_reconfigure 设置 ${Red_font_prefix}红色${Font_color_suffix}、${Green_font_prefix}绿色${Font_color_suffix}、${Blue_font_prefix}蓝色${Font_color_suffix}方块的 HSV 值"
			echo -e "${Info} 动态参数服务器名为 HSVParams_node"
			echo -e "${Info} 设置完成后在本终端 Ctrl+C 退出程序，程序自动保存 HSV 值"
			echo && stty erase ^? && read -p "按回车键（Enter）继续：" foonum
			print_command "roslaunch sagittarius_object_color_detector hsv_params.launch"
			roslaunch sagittarius_object_color_detector hsv_params.launch
			;;
			2)
			echo -e "${Info} 手眼标定 (Eye In Hand)"
			echo -e "${Info} 按照接下来的提示进行标定"
			echo -e "${Info} 默认使用${Blue_font_prefix}蓝色${Font_color_suffix}方块"
			echo && stty erase ^? && read -p "按回车键（Enter）继续：" foonum
			print_command "roslaunch sagittarius_object_color_detector camera_calibration_hsv.launch "
			roslaunch sagittarius_object_color_detector camera_calibration_hsv.launch 
			;;
			3)
			echo -e "${Info} 机械臂进行一次抓取的示例"
			echo -e "${Info} 把一个方块放在识别区中心，机械臂自动抓取并放置到 R 区"
			echo -e "${Info} 请勿放杂物在 R 区，默认识别${Blue_font_prefix}蓝色${Font_color_suffix}方块"
			echo && stty erase ^? && read -p "按回车键（Enter）继续：" foonum
			print_command "roslaunch sagittarius_object_color_detector object_pick.launch"
			roslaunch sagittarius_object_color_detector object_pick.launch
			;;
			4)
			echo -e "${Info} 对方块进行颜色分类 (投放位置固定)"
			echo -e "${Info} 机械臂识别抓取并分类方块，投放在固定的地方"
			echo -e "${Info} 把容器按颜色${Red_font_prefix}红${Font_color_suffix}、${Green_font_prefix}绿${Font_color_suffix}、${Blue_font_prefix}蓝${Font_color_suffix}顺序扣好，在正对机械臂正面时，识别区的右侧按列方向放好"
			echo -e "${Info} 机械臂投放位置固定，请配合投放位置调整容器的位置"
			echo && stty erase ^? && read -p "按回车键（Enter）继续：" foonum
			print_command "roslaunch sagittarius_object_color_detector color_classification_fixed.launch"
			roslaunch sagittarius_object_color_detector color_classification_fixed.launch
			;;
			5)
			echo -e "${Info} 对方块进行颜色分类 (配合地图寻找投放位置)"
			echo -e "${Info} 把投放容器任意放置在地图的 ABCD 区上，机械臂抓取方块并投放对应颜色的容器"
			echo -e "${Info} 机械臂通过识别容器颜色确定投放区，请勿遮挡容器颜色"
			echo && stty erase ^? && read -p "按回车键（Enter）继续：" foonum
			print_command "roslaunch sagittarius_object_color_detector color_classification.launch"
			roslaunch sagittarius_object_color_detector color_classification.launch
			;;
			*)
			echo -e "${Error} 错误，退出"
			;;
		esac
	fi
}

#3d_perception_detection_and_grab
3d_perception_detection_and_grab(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	if [ -n "$(roscd sagittarius_perception)" ]; then
		echo -e "${Info}没有找到高配版的视觉套件功能包，如需要高配版请联系我们市场部"
	elif [ ! -n "$(lsusb -d 8086:0b07)" ]; then
		echo -e "${Error}没有找到3D视觉摄像头, 检查摄像头数据线是否正确连接"
	else
		echo -e "${Info}" 
		echo -e "${Info}3D视觉抓取 (eye-to-hand)" 
		echo -e "${Info}" 
		echo -e "${Info}摄像头放置在能同时看到机械臂标定二维码和识别区域的地方"
		echo -e "${Info}标定完成后请勿移动摄像头" 
		echo -e "${Info}" 
		echo -e "${Info}请选择相关选择：
	${Green_font_prefix}1.${Font_color_suffix} 手眼标定 (eye-to-hand)
	${Green_font_prefix}2.${Font_color_suffix} 颜色抓取分类
	${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入数字 [1-2]：" asrnum
		case "$asrnum" in
			1)
			echo -e "${Info}" 
			echo -e "${Info}界面 ArmTag Tuner GUI 用于标定" 
			echo -e "${Info}界面 PointCloud Tuner GUI 用于调整物体识别的参数" 
			echo -e "${Info}退出请输入：Ctrl + c" 
			echo -e "${Info}" 
			echo && stty erase ^? && read -p "按回车键（Enter）继续：" 
			print_command "roslaunch sagittarius_perception calibrate.launch"
			roslaunch sagittarius_perception calibrate.launch
			;;
			2)
			echo -e "${Info}" 
			echo -e "${Info}请放置好物体" 
			echo -e "${Info}退出请输入：Ctrl + c" 
			echo -e "${Info}" 
			echo && stty erase ^? && read -p "按回车键（Enter）继续：" 
			print_command "roslaunch sagittarius_perception demo_color_classifcation.launch"
			roslaunch sagittarius_perception demo_color_classifcation.launch
			;;
			*)
			echo -e "${Error} 错误，退出"
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
		echo -e "${Info}使用无线网络wlp1s0" 
	  	net_interface="wlp1s0"
	elif [ $wlo1_ip ]; then
		echo -e "${Info}使用无线网络wlo1" 
	  	net_interface="wlo1"	  	
	elif [ $wlp2s_ip ]; then
		echo -e "${Info}使用无线网络wlp2s0" 
	  	net_interface="wlp2s0"
	elif [ $wlan_ip ]; then
		echo -e "${Info}使用无线网络wlan0" 
	  	net_interface="wlan0"
	elif [ $enp3s_ip ]; then
		echo -e "${Info}使用无线网络enp3s0" 
		net_interface="enp3s0"
	elif [ $wlp0s_ip ]; then
		echo -e "${Info}使用无线网络wlp0s20f3" 
		net_interface="wlp0s20f3"		
	elif [ $eth_ip ]; then
		echo -e "${Info}使用无线网络eth0" 
		net_interface="eth0"
	elif [ $eno1_ip ]; then
		echo -e "${Info}使用无线网络eno1" 
		net_interface="eno1"
	fi	
	
	echo -e "${Info}" 
	echo -e "${Info}通过局域网收发文件" 
	echo -e "${Info}" 
	echo -e "${Info}请选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 发送文件（文件名，带上文件绝对路径）
	  ${Green_font_prefix}2.${Font_color_suffix} 接收文件（默认存放在~/Downloads路径中）
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" cnum
	case "$cnum" in
		1)
		echo -e "${Info}请输入文件名，带上文件绝对路径，如 /home/${USER}/a.jpg：
		 退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入要发送的文件：" s_file
		if [ -f "$s_file" ]; then
			echo -e "${Info}本机即将发送文件：${Green_font_prefix}"$s_file"${Font_color_suffix}，请接收端扫码或者直接输入下面的网址接收文件"
		else 
			echo -e "${Info}请输入带绝对路径的文件名"
			exit
		fi
		
		qrcp send  -i $net_interface $s_file
		;;
		2)
		echo -e "${Info}请输入接收到的文件存放的路径，默认为 /home/${USER}/Downloads：
		退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入文件存放的文件夹路径：" s_file
		if [ -d "$s_file" ]; then
			echo ""
		else 
			echo -e "${Info}${Red_font_prefix}文件夹不存在，将存放在默认文件夹/home/${USER}/Downloads中${Font_color_suffix}"
			s_file="/home/${USER}/Downloads"
		fi
		echo -e "${Info}接收的文件将存放在：${Green_font_prefix}"$s_file"${Font_color_suffix}，目录下，请发送端扫码或者直接输入下面的网址选择文件发送"
		qrcp  -i $net_interface receive --output=$s_file
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}

coming_soon(){
	echo -e "${Tip} coming_soon!" 
}


#printf
menu_status(){
	echo -e "${Tip} 当前系统版本 ${OSDescription} !" 
	if [ -f "/usr/bin/rosversion" ]; then
		ROSVER=`/usr/bin/rosversion -d`
		if [ $ROSVER ]; then
			echo -e "${Tip} 当前ROS版本 ${ROSVER} !"
			return
		fi 
	fi
	echo -e "${Error} 未检测到ROS版本，请先安装ROS！可以选择102直接安装。" 
}

tell_us(){
	echo -e ""
	echo -e "${Tip} --------------分隔线----------------" 
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
  Sagittarius 一键管理脚本"
qrcode_picture

echo -e "  
  请根据右侧的功能说明选择相应的序号。
  注意：101～103为相关环境的安装与设置，如果已执行过，不要再重复执行。
  
  ${Green_font_prefix}  0.${Font_color_suffix} 单独编译
————————————
  ${Green_font_prefix}  1.${Font_color_suffix} 启动MOVEIT界面
  ${Green_font_prefix}  2.${Font_color_suffix} 正运动学示例
  ${Green_font_prefix}  3.${Font_color_suffix} 逆运动学示例
  ${Green_font_prefix}  4.${Font_color_suffix} 单个舵机控制
  ${Green_font_prefix}  5.${Font_color_suffix} rosbag录制与播放机械臂动作
  ${Green_font_prefix}  6.${Font_color_suffix} 多机械臂动作实时复现
  ${Green_font_prefix}  7.${Font_color_suffix} 机械臂画五角星同步仿真显示
  ${Green_font_prefix}  8.${Font_color_suffix} 通过臂上摄像头进行视觉抓取\c"


echo -e "
————————————

  ${Green_font_prefix}100.${Font_color_suffix} 问题反馈
  ${Green_font_prefix}103.${Font_color_suffix} 安装依赖库
  ${Green_font_prefix}104.${Font_color_suffix} 文件传输
 "
menu_status
check_dev
echo && stty erase ^? && read -p "请输入数字：" num
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
	echo -e "${Error} 请输入正确的数字 "
	;;
esac

