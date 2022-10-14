#!/usr/bin/env python3
import rospy
import _thread
from std_msgs.msg import String

import actionlib
from sagittarius_object_color_detector.msg import SGRCtrlAction, SGRCtrlGoal
start_cali = 0
go_cali_pos = 0
next_cali_pos = 0


def msg_callback(data):
    global start_cali
    global go_cali_pos
    global next_cali_pos

    if(data.data == "start"):
        start_cali = 1
    elif(data.data == "go"):
        go_cali_pos = 1
    elif(data.data == "next"):
        next_cali_pos = 1


def talker(threadName, delay):
    global next_cali_pos
    global go_cali_pos
    client = actionlib.SimpleActionClient(
        rospy.get_param("~arm_name", "sgr532") + '/' + 'sgr_ctrl', SGRCtrlAction)
    client.wait_for_server()
    pub2 = rospy.Publisher('cali_pix_topic', String, queue_size=5)
    r1 = rospy.Rate(5)  # 0.2s

    goal = SGRCtrlGoal()
    goal.action_type = goal.ACTION_TYPE_XYZ_RPY
    goal.pos_z = 0.06
    goal.pos_pitch = 1.57
    goal_cali = SGRCtrlGoal()
    goal_cali.action_type = goal.ACTION_TYPE_XYZ_RPY
    goal_cali.pos_x = 0.2
    goal_cali.pos_z = 0.15
    goal_cali.pos_pitch = 1.57

    point_list = [
        [0.25, 0],
        [0.225, 0.025],
        [0.275, 0.025],
        [0.275, -0.025],
        [0.225, -0.025]
    ]
    print("start to move the sagittarius----")
    for i in range(len(point_list)):
        if rospy.is_shutdown():
            break

        # 移动到标定位置，引导用户放置方块的地方
        goal.pos_x = point_list[i][0]
        goal.pos_y = point_list[i][1]
        client.send_goal_and_wait(goal, rospy.Duration.from_sec(30))
        print("Point %d: %.4f, %.4f" % (i + 1, goal.pos_x, goal.pos_y))

        # 等待用户放好方块后，机械臂移动到其他地方，不遮挡摄像头
        while(go_cali_pos == 0):
            r1.sleep()
        if(go_cali_pos == 0):
            return
        go_cali_pos = 0
        client.send_goal_and_wait(goal_cali, rospy.Duration.from_sec(30))
        pub2.publish("")

        cnt = 0
        # 等待标定数据采集后进行下一个动作
        while(next_cali_pos == 0):
            r1.sleep()
            cnt += 1
            if cnt > 20:
                rospy.logerr("timeout waiting \"next\" command, please check the hsv value range")
                break
        if(next_cali_pos == 0):
            return
        next_cali_pos = 0

    print("end move----")


if __name__ == '__main__':
    rospy.init_node('cali_pos', anonymous=True)
    r1 = rospy.Rate(5)  # 0.2s

    sub3 = rospy.Subscriber("cali_arm_cmd_topic", String, msg_callback)
    while not start_cali:
        r1.sleep()
    
    _thread.start_new_thread(talker, ("Thread-1", 2, ))
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
