#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import yaml
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import actionlib
from sagittarius_object_color_detector.msg import SGRCtrlAction, SGRCtrlGoal, SGRCtrlResult
from sensor_msgs.msg import Image
import tf.transformations as tf_transformations


object_dst = {
    'blue': {
        'lower_HSV': None,
        'upper_HSV': None,
        'cnt': 0,
        'found': False,
        'x': 0,
        'y': 0
    },
    'red': {
        'lower_HSV': None,
        'upper_HSV': None,
        'cnt': 0,
        'found': False,
        'x': 0,
        'y': 0
    },
    'green': {
        'lower_HSV': None,
        'upper_HSV': None,
        'cnt': 0,
        'found': False,
        'x': 0,
        'y': 0
    }
}

def object_detector(img, lower_hsv, upper_hsv, win_name="gray"):
    cv_image_cp = img.copy()
    cv_image_hsv = cv2.cvtColor(cv_image_cp, cv2.COLOR_BGR2HSV)
    if lower_hsv[0] > upper_hsv[0]: # hue will jump between 360(180) and 0 in red area
        lower = np.array([0 / 2, lower_hsv[1], lower_hsv[2]])
        upper = np.array([upper_hsv[0], upper_hsv[1], upper_hsv[2]])

        lower2 = np.array([lower_hsv[0], lower_hsv[1], lower_hsv[2]])
        upper2 = np.array([180, upper_hsv[1], upper_hsv[2]])
        cv_image_gray = cv2.add(
            cv2.inRange(cv_image_hsv, lower, upper),
            cv2.inRange(cv_image_hsv, lower2, upper2)
        )
    else:
        cv_image_gray = cv2.inRange(cv_image_hsv, lower_hsv, upper_hsv)

    # smooth and clean noise
    cv_image_gray = cv2.erode(cv_image_gray, None, iterations=2)
    cv_image_gray = cv2.dilate(cv_image_gray, None, iterations=2)
    cv_image_gray = cv2.GaussianBlur(cv_image_gray, (5, 5), 0)

    # detect contour
    cv2.imshow("source", img)
    cv2.imshow(win_name, cv_image_gray)
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
    global object_dst
    # change to opencv
    try:
        cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    # hsv range
    for color in object_dst:
        if object_dst[color]['found'] == True: continue
        obj = object_dst[color]
        size_max, foo_xc, foo_yc = object_detector(cv_image1, obj['lower_HSV'], obj['upper_HSV'], win_name=color)
        if size_max > 80 * 80:
            if obj['x'] - 10 < foo_xc < obj['x'] + 10 and \
                obj['y'] - 10 < foo_yc < obj['y'] + 10:
                obj['cnt'] = obj['cnt'] + 1
                if obj['cnt'] > 30:
                    obj['x'] = foo_xc
                    obj['y'] = foo_yc
                    obj['found'] = True
                    obj['cnt'] = 0
                    rospy.logwarn("%s %.4f %.4f %d" % (color, obj['x'], obj['y'], obj['cnt']))
            else:
                obj['x'] = foo_xc
                obj['y'] = foo_yc
                obj['cnt'] = 0

def main():
    global object_dst
    rospy.init_node('color_classification_node', anonymous=False)

    # sagittarius control action
    client = actionlib.SimpleActionClient(
        rospy.get_param("~arm_name", "sgr532") + '/' + 'sgr_ctrl', SGRCtrlAction)
    client.wait_for_server()

    goal_search = SGRCtrlGoal()
    goal_search.action_type = goal_search.ACTION_TYPE_XYZ_RPY
    goal_search.grasp_type = goal_search.GRASP_OPEN
    goal_search.pos_x = 0.2
    goal_search.pos_z = 0.15
    goal_search.pos_pitch = 1.57
    
    filename = rospy.get_param("~vision_config")
    try:
        with open(filename, "r") as f:
            content = yaml.load(f.read())
    except:
        rospy.logerr("can't not open hsv file: ", filename)
        exit(1)
    
    # get LinearRegression k and b value
    k1 = content['LinearRegression']['k1']
    b1 = content['LinearRegression']['b1']
    k2 = content['LinearRegression']['k2']
    b2 = content['LinearRegression']['b2']
    rospy.loginfo("x axis k and b: %f, %f" % (k1, b2))
    rospy.loginfo("y axis k and b: %f, %f" % (k2, b2))

    box_dst = {
        'red':{
            'found': False,
            'lower_HSV': np.array([0 / 2, 70, 50]),
            'upper_HSV': np.array([40 / 2, 240, 240]),
            'x': 0.1,
            'y': 0.18
        },
        'green':{
            'found': False,
            'lower_HSV': np.array([130 / 2, 70, 50]),
            'upper_HSV': np.array([170 / 2, 240, 240]),
            'x': 0.18,
            'y': 0.18
        },
        'blue':{
            'found': False,
            'lower_HSV': np.array([200 / 2, 70, 50]),
            'upper_HSV': np.array([250 / 2, 240, 240]),
            'x': 0.26,
            'y': 0.18
        }
    }
    
    # get hsv range
    for color in object_dst:
        object_dst[color]['lower_HSV'] = np.array([content[color]['hmin'] / 2, content[color]['smin'], content[color]['vmin']])
        object_dst[color]['upper_HSV'] = np.array([content[color]['hmax'] / 2, content[color]['smax'], content[color]['vmax']])

    goal_pick = SGRCtrlGoal()
    goal_pick.grasp_type = goal_pick.GRASP_OPEN
    goal_pick.action_type = goal_pick.ACTION_TYPE_PICK_XYZ
    goal_pick.pos_z = 0.02
    goal_pick.pos_pitch = 1.57
    goal_put = SGRCtrlGoal()
    goal_put.action_type = goal_pick.ACTION_TYPE_PUT_XYZ
    goal_put.pos_z = 0.2
    
    # move arm to search pose
    client.send_goal_and_wait(goal_search, rospy.Duration.from_sec(30))
    
    sub1 = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    r1 = rospy.Rate(5)  # 0.2s
    try:
        while not rospy.is_shutdown():
            # object is stationary for 1 second
            for color in object_dst:
                if object_dst[color]['found'] == False:
                    r1.sleep()
                    continue
                rospy.loginfo("%s object found!!!" % (color))
                
                goal_pick.pos_x = k1 * object_dst[color]['y'] + b1
                goal_pick.pos_y = k2 * object_dst[color]['x'] + b2
                rospy.loginfo("ee-pose: %.4f %.4f %.4f" %
                            (goal_pick.pos_x, goal_pick.pos_y, goal_pick.pos_z))

                client.send_goal_and_wait(goal_pick, rospy.Duration.from_sec(30))
                ret = client.get_result()
                if ret.result == SGRCtrlResult.PLAN_NOT_FOUND:
                    rospy.logwarn("no plan return. pass")
                elif ret.result == SGRCtrlResult.GRASP_FAILD:
                    rospy.logwarn("grasp faild. pass")
                else:
                    rospy.loginfo("grasp success")
                    goal_put.pos_x = box_dst[color]['x']
                    goal_put.pos_y = box_dst[color]['y']
                    client.send_goal_and_wait(goal_put, rospy.Duration.from_sec(30))

                for i in object_dst:
                    object_dst[i]['cnt'] = 0
                    object_dst[i]['found'] = False

                client.send_goal_and_wait(goal_search, rospy.Duration.from_sec(30))

    except rospy.ROSInterruptException:
        rospy.loginfo("Finised")


if __name__ == '__main__':
    main()
