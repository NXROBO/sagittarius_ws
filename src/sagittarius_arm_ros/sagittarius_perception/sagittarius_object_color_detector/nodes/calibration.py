#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import os
import yaml
#import pandas as pd
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sklearn.linear_model import LinearRegression
#from PIL import Image, ImageDraw, ImageFont
xc = 0
yc = 0

count = 5
index = 0
arm_cmd_sub = None

xarray_list = [0.25, 0.225, 0.275, 0.275, 0.225]
yarray_list = [0, 0.025, 0.025, -0.025, -0.025]
xarray = np.zeros(count)
yarray = np.zeros(count)
xc_array = np.zeros(count)
yc_array = np.zeros(count)

start_flag = 0
c_cnt = 0
lower_HSV = None
upper_HSV = None

content = None


def image_callback(data):
    global xc, yc
    global c_cnt
    global lower_HSV, upper_HSV

    # change to opencv
    try:
        cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv_image_cp = cv_image1.copy()
    cv_image_hsv = cv2.cvtColor(cv_image_cp, cv2.COLOR_BGR2HSV)
    cv_image_gray = cv2.inRange(cv_image_hsv, lower_HSV, upper_HSV)
    # smooth and clean noise
    cv_image_gray = cv2.erode(cv_image_gray, None, iterations=2)
    cv_image_gray = cv2.dilate(cv_image_gray, None, iterations=2)
    cv_image_gray = cv2.GaussianBlur(cv_image_gray, (5, 5), 0)
    # detect contour
    cv2.imshow("win1", cv_image1)
    cv2.imshow("win2", cv_image_gray)
    cv2.waitKey(1)
    contours, hier = cv2.findContours(
        cv_image_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    size = []
    size_max = 0
    for i, c in enumerate(contours):
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
        y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
        w = math.sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2)
        h = math.sqrt((box[0][0] - box[3][0])**2 + (box[0][1] - box[3][1])**2)
        size.append(w * h)
        if size[i] > size_max:
            size_max = size[i]
            index = i
            xc = x_mid
            yc = y_mid


def command_callback(data):
    global xc_array
    global yc_array
    global xarray
    global yarray
    global index
    if index < count:

        xc_array[index] = xc
        yc_array[index] = yc
        xarray[index] = xarray_list[index]
        yarray[index] = yarray_list[index]
        print("%d/%d,pose x,y: %.4f,%.4f. cam x,y: %d,%d" %
              (index+1, count, xarray[index], yarray[index], xc, yc))
        # reshape to 2D array for linear regression
        xc_array = xc_array.reshape(-1, 1)
        yc_array = yc_array.reshape(-1, 1)
        xarray = xarray.reshape(-1, 1)
        yarray = yarray.reshape(-1, 1)
        index = index + 1
        arm_cmd_sub.publish(String("next"))
    if index == count:
        Reg_x_yc = LinearRegression().fit(yc_array, xarray)
        Reg_y_xc = LinearRegression().fit(xc_array, yarray)
        k1 = Reg_x_yc.coef_[0][0]
        b1 = Reg_x_yc.intercept_[0]
        k2 = Reg_y_xc.coef_[0][0]
        b2 = Reg_y_xc.intercept_[0]
        content['LinearRegression']['k1'] = float(k1)
        content['LinearRegression']['b1'] = float(b1)
        content['LinearRegression']['k2'] = float(k2)
        content['LinearRegression']['b2'] = float(b2)

        filename = rospy.get_param("~vision_config")
        try:
            with open(filename, "w") as f:
                yaml.dump(content, f)
        except:
            rospy.logerr("can't not open hsv file: ", filename)

        index = 0
        print("Linear Regression for x and yc is :  x = %.5fyc + (%.5f)" % (k1, b1))
        print("Linear Regression for y and xc is :  y = %.5fxc + (%.5f)" % (k2, b2))
        print("******************************************************")
        print("     finish the calibration. Press ctrl-c to exit     ")
        print("             标定完成. 然后 Ctrl-C 退出程序             ")
        print("******************************************************")


def msg_callback(data):
    global start_flag
    if(data.data == "start"):
        start_flag = 1


def main():
    global arm_cmd_sub
    global content
    global lower_HSV
    global upper_HSV
    rospy.init_node('eye_in_hand_calibration', anonymous=True)
    r1 = rospy.Rate(5)  # 0.2s

    filename = rospy.get_param("~vision_config")
    try:
        with open(filename, "r") as f:
            content = yaml.load(f.read())
    except:
        rospy.logerr("can't not open hsv file: ", filename)
        exit(1)

    lower_HSV = np.array([content['blue']['hmin'] / 2, content['blue']['smin'], content['blue']['vmin']])
    upper_HSV = np.array([content['blue']['hmax'] / 2, content['blue']['smax'], content['blue']['vmax']])

    print("Calibration node wait to start----")
    sub3 = rospy.Subscriber("cali_cmd_topic", String, msg_callback)
    while not start_flag:
        r1.sleep()

    sub1 = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    sub2 = rospy.Subscriber("cali_pix_topic", String, command_callback)
    arm_cmd_sub = rospy.Publisher('cali_arm_cmd_topic', String, queue_size=5)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
