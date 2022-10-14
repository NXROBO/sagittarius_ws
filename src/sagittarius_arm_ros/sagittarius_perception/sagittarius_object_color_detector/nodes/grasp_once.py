#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import yaml
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import actionlib
from actionlib import GoalStatus
from sagittarius_object_color_detector.msg import SGRCtrlAction, SGRCtrlGoal
from sensor_msgs.msg import Image


lower_HSV = None
upper_HSV = None
hue_cut = False
xc = 0
yc = 0


def object_detector(img, lower_HSV, upper_HSV):
    cv_image_cp = img.copy()
    cv_image_hsv = cv2.cvtColor(cv_image_cp, cv2.COLOR_BGR2HSV)
    if lower_HSV[0] > upper_HSV[0]: # hue will jump between 360 and 0 in red area
        lower = np.array([0 / 2, lower_HSV[1], lower_HSV[2]])
        upper = np.array([upper_HSV[0], upper_HSV[1], upper_HSV[2]])

        lower2 = np.array([lower_HSV[0], lower_HSV[1], lower_HSV[2]])
        upper2 = np.array([180, upper_HSV[1], upper_HSV[2]])
        cv_image_gray = cv2.add(
            cv2.inRange(cv_image_hsv, lower, upper),
            cv2.inRange(cv_image_hsv, lower2, upper2)
        )
    else:
        cv_image_gray = cv2.inRange(cv_image_hsv, lower_HSV, upper_HSV)

    # smooth and clean noise
    cv_image_gray = cv2.erode(cv_image_gray, None, iterations=2)
    cv_image_gray = cv2.dilate(cv_image_gray, None, iterations=2)
    cv_image_gray = cv2.GaussianBlur(cv_image_gray, (5, 5), 0)

    # detect contour
    cv2.imshow("source", img)
    cv2.imshow("gray", cv_image_gray)
    cv2.waitKey(1)

    # find contours
    contours, hier = cv2.findContours(
        cv_image_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    size = []
    size_max = 0
    xc = 0
    yc = 0
    for i, c in enumerate(contours):
        # get the contours border rectangle
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # get the center point of the border rectangle
        x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
        y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4

        # get the area of the border rectangle
        w = math.sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2)
        h = math.sqrt((box[0][0] - box[3][0])**2 + (box[0][1] - box[3][1])**2)
        size.append(w * h)

        # mark the largest area
        if size[i] > size_max:
            size_max = size[i]
            xc = x_mid
            yc = y_mid
    return size_max, xc, yc


def image_callback(data):
    global xc, yc
    global lower_HSV, upper_HSV

    # change to opencv
    try:
        cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    # hsv range
    size_max, foo_xc, foo_yc = object_detector(cv_image1, lower_HSV, upper_HSV)
    if size_max > 50 * 50:
        xc = foo_xc
        yc = foo_yc

def main():
    global lower_HSV
    global upper_HSV
    global hue_cut
    rospy.init_node('grasp_once_node', anonymous=False)

    # sagittarius control action
    client = actionlib.SimpleActionClient(
        rospy.get_param("~arm_name", "sgr532") + '/' + 'sgr_ctrl', SGRCtrlAction)
    client.wait_for_server()

    goal_search = SGRCtrlGoal()
    goal_search.action_type = goal_search.ACTION_TYPE_XYZ_RPY
    goal_search.pos_x = 0.2
    goal_search.pos_z = 0.15
    goal_search.pos_pitch = 1.57
    goal_pick = SGRCtrlGoal()
    goal_pick.grasp_type = goal_pick.GRASP_OPEN
    goal_pick.action_type = goal_pick.ACTION_TYPE_PICK_XYZ
    goal_pick.pos_z = 0.02
    goal_pick.pos_pitch = 1.57
    goal_put = SGRCtrlGoal()
    goal_put.action_type = goal_pick.ACTION_TYPE_PUT_XYZ_RPY
    goal_put.pos_x = 0
    goal_put.pos_y = 0.2
    goal_put.pos_z = 0.02
    goal_put.pos_pitch = 1.57
    goal_put.pos_yaw = 1.57

    # get hsv range
    filename = rospy.get_param("~vision_config")
    try:
        with open(filename, "r") as f:
            content = yaml.load(f.read())
    except:
        rospy.logerr("can't not open hsv file: ", filename)
        exit(1)
    hsv_value = content[rospy.get_param('~color_type', 'blue')]
    print(hsv_value)
    lower_HSV = np.array([hsv_value['hmin'] / 2, hsv_value['smin'], hsv_value['vmin']])
    upper_HSV = np.array([hsv_value['hmax'] / 2, hsv_value['smax'], hsv_value['vmax']])
    rospy.loginfo(lower_HSV)
    rospy.loginfo(upper_HSV)
    
    # get LinearRegression k and b value
    k1 = content['LinearRegression']['k1']
    b1 = content['LinearRegression']['b1']
    k2 = content['LinearRegression']['k2']
    b2 = content['LinearRegression']['b2']
    rospy.loginfo("x axis k and b: %f, %f" % (k1, b2))
    rospy.loginfo("y axis k and b: %f, %f" % (k2, b2))

    # sub the image topic
    sub1 = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    # move arm to search pose
    client.send_goal_and_wait(goal_search, rospy.Duration.from_sec(30))
    try:
        r1 = rospy.Rate(5)  # 0.2s
        cnt = 0
        xc_last = xc
        yc_last = yc
        while not rospy.is_shutdown():
            # object is stationary for 1 second
            if xc_last - 10 < xc < xc_last + 10 and yc_last - 10 < yc < yc_last + 10:
                cnt = cnt + 1
            else:
                xc_last = xc
                yc_last = yc
                cnt = 0
            if cnt < 5:
                r1.sleep()
                continue

            rospy.loginfo("object found!!!")

            goal_pick.pos_x = k1 * yc + b1
            goal_pick.pos_y = k2 * xc + b2
            rospy.loginfo("ee-pose: %.4f %.4f %.4f" %
                          (goal_pick.pos_x, goal_pick.pos_y, goal_pick.pos_z))

            client.send_goal_and_wait(goal_pick, rospy.Duration.from_sec(30))
            client.send_goal_and_wait(goal_put, rospy.Duration.from_sec(30))
            client.send_goal_and_wait(goal_search, rospy.Duration.from_sec(30))
            cnt = 0

            break
        goal_sleep = SGRCtrlGoal()
        goal_sleep.action_type = goal_sleep.ACTION_TYPE_DEFINE_SAVE
        client.send_goal_and_wait(goal_sleep, rospy.Duration.from_sec(30))
    
        print()
        print("**********  抓取完成  **********")
        print()
    except rospy.ROSInterruptException:
        rospy.loginfo("Finised")


if __name__ == '__main__':
    main()
