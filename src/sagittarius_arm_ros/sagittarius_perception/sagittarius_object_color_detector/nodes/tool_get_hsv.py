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

cnt = 0
hsv_sum = [0, 0, 0]
hsv_last = [0, 0, 0]
def image_callback(data):
    global cnt
    global hsv_sum
    global hsv_last

    # change to opencv
    try:
        cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    # hsv range
    cv_image_cp = cv_image1.copy()
    
    hsv = cv2.cvtColor(cv_image_cp[230:250, 310:330], cv2.COLOR_BGR2HSV)
    hsv_sum[0] += np.mean(hsv[:, :, 0]) * 2
    hsv_sum[1] += np.mean(hsv[:, :, 1])
    hsv_sum[2] += np.mean(hsv[:, :, 2])
    cnt += 1
    if cnt >= 5:
        hsv_last[0] = hsv_sum[0] / cnt
        hsv_last[1] = hsv_sum[1] / cnt
        hsv_last[2] = hsv_sum[2] / cnt
        hsv_sum = [0, 0, 0]
        cnt = 0
    cv2.putText(cv_image_cp, 'Center HSV ', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(cv_image_cp, 'H:%3d S:%3d V:%3d' % (hsv_last[0], hsv_last[1], hsv_last[2]), (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.rectangle(cv_image_cp, (310, 230), (330, 250), (0, 255, 0), 2)
    cv2.imshow("win_draw", cv_image_cp)
    cv2.waitKey(1)


def main():
    rospy.init_node('grasp_once_node', anonymous=False)

    # sub the image topic
    sub1 = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Finised")


if __name__ == '__main__':
    main()
