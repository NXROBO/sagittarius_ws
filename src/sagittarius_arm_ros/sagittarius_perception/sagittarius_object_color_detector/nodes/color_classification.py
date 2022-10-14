#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import yaml
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import actionlib
from sagittarius_object_color_detector.msg import SGRCtrlAction, SGRCtrlGoal, SGRCtrlResult
from sensor_msgs.msg import Image
# import tf.transformations as tf_transformations
import smach


class FoundObject(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=['success', 'preempted', 'error'],
            input_keys=['object_dst'],
            output_keys=['color_type', 'screen_position']
        )

        self.object_dst = None

    def object_detector(self, img, lower_hsv, upper_hsv, win_name="gray"):
        cv_image_cp = img.copy()
        cv_image_hsv = cv2.cvtColor(cv_image_cp, cv2.COLOR_BGR2HSV)
        # hue will jump between 360(180) and 0 in red area
        if lower_hsv[0] > upper_hsv[0]:
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
            w = math.sqrt((box[0][0] - box[1][0])**2 +
                          (box[0][1] - box[1][1])**2)
            h = math.sqrt((box[0][0] - box[3][0])**2 +
                          (box[0][1] - box[3][1])**2)
            size = w * h

            # mark the largest area
            if size > size_max:
                size_max = size
                xc = x_mid
                yc = y_mid
        return size_max, xc, yc

    def image_callback(self, data):
        # change to opencv
        try:
            cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # hsv range
        for color in self.object_dst:
            if self.object_dst[color]['found'] == True:
                continue
            obj = self.object_dst[color]
            size_max, foo_xc, foo_yc = self.object_detector(
                cv_image1, obj['lower_HSV'], obj['upper_HSV'], win_name=color)
            if size_max > 80 * 80:
                if obj['x'] - 10 < foo_xc < obj['x'] + 10 and \
                        obj['y'] - 10 < foo_yc < obj['y'] + 10:
                    obj['cnt'] = obj['cnt'] + 1
                    if obj['cnt'] > 30:
                        obj['x'] = foo_xc
                        obj['y'] = foo_yc
                        obj['found'] = True
                        obj['cnt'] = 0
                        rospy.logwarn("color type:%s x=%.4f y=%.4f" %
                                      (color, obj['x'], obj['y']))
                else:
                    obj['x'] = foo_xc
                    obj['y'] = foo_yc
                    obj['cnt'] = 0

    def execute(self, ud):
        client = actionlib.SimpleActionClient(
            rospy.get_param("~arm_name", "sgr532") + '/' + 'sgr_ctrl', SGRCtrlAction)
        client.wait_for_server()
        goal_search = SGRCtrlGoal()
        goal_search.action_type = goal_search.ACTION_TYPE_XYZ_RPY
        goal_search.pos_x = 0.2
        goal_search.pos_z = 0.15
        goal_search.pos_pitch = 1.57
        client.send_goal_and_wait(goal_search, rospy.Duration.from_sec(30))

        self.object_dst = {}
        for color in ud.object_dst:
            self.object_dst[color] = {}
            self.object_dst[color]['lower_HSV'] = ud.object_dst[color]['lower_HSV']
            self.object_dst[color]['upper_HSV'] = ud.object_dst[color]['upper_HSV']
            self.object_dst[color]['found'] = False
            self.object_dst[color]['cnt'] = 0
            self.object_dst[color]['x'] = 0
            self.object_dst[color]['y'] = 0

        r1 = rospy.Rate(15)
        sub1 = rospy.Subscriber("/usb_cam/image_raw",
                                Image, self.image_callback)
        while True:
            for color in self.object_dst:
                if self.object_dst[color]['found'] == True:
                    ud.color_type = color
                    ud.screen_position = {
                        'x': self.object_dst[color]['x'],
                        'y': self.object_dst[color]['y']
                    }
                    sub1.unregister()
                    return 'success'
                if self.preempt_requested():
                    return 'preempted'
                r1.sleep()


class GraspObject(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=['success', 'grasp_faild', 'preempted', 'error'],
            input_keys=['color_type', 'screen_position',
                        'linearression_kb_dst'],
            output_keys=['color_type']
        )

    def execute(self, ud):
        client = actionlib.SimpleActionClient(
            rospy.get_param("~arm_name", "sgr532") + '/' + 'sgr_ctrl', SGRCtrlAction)
        client.wait_for_server()
        goal_pick = SGRCtrlGoal()
        goal_pick.grasp_type = goal_pick.GRASP_OPEN
        goal_pick.action_type = goal_pick.ACTION_TYPE_PICK_XYZ_RPY
        goal_pick.pos_z = 0.01
        goal_pick.pos_pitch = 1.57
        goal_pick.pos_x = ud.linearression_kb_dst['k1'] * \
            ud.screen_position['y'] + ud.linearression_kb_dst['b1']
        goal_pick.pos_y = ud.linearression_kb_dst['k2'] * \
            ud.screen_position['x'] + ud.linearression_kb_dst['b2']

        grasp_step = 1
        if grasp_step == 1:
            if self.preempt_requested():
                return 'preempted'
            client.send_goal_and_wait(goal_pick, rospy.Duration.from_sec(30))
            ret = client.get_result().result
            if ret == SGRCtrlResult.SUCCESS:  # grasp success, next
                grasp_step = 3
            elif ret == SGRCtrlResult.PLAN_NOT_FOUND:  # plan fail, try other posture
                grasp_step = 2
            elif ret == SGRCtrlResult.GRASP_FAILD:  # grasp fail, search again
                grasp_step = 4

        if grasp_step == 2:  # plan fail, try other posture
            goal_pick.action_type = goal_pick.ACTION_TYPE_PICK_XYZ
            if self.preempt_requested():
                return 'preempted'
            client.send_goal_and_wait(goal_pick, rospy.Duration.from_sec(30))
            ret = client.get_result().result
            if ret == SGRCtrlResult.SUCCESS:  # grasp success, next
                grasp_step = 3
            elif ret == SGRCtrlResult.PLAN_NOT_FOUND:  # plan fail, search again
                grasp_step = 4
            elif ret == SGRCtrlResult.GRASP_FAILD:  # grasp fail, search again
                grasp_step = 4

        if grasp_step == 3:
            return 'success'
        elif grasp_step == 4:
            return 'grasp_faild'
        else:
            return 'error'


class DropObject(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=['success', 'preempted', 'error'],
            input_keys=['color_type', 'drop_position_dst']
        )

    def execute(self, ud):
        client = actionlib.SimpleActionClient(
            rospy.get_param("~arm_name", "sgr532") + '/' + 'sgr_ctrl', SGRCtrlAction)
        client.wait_for_server()
        color_type = ud.color_type
        if color_type not in ud.drop_position_dst.keys():
            color_type = 'None'
        goal_put = SGRCtrlGoal()
        goal_put.action_type = goal_put.ACTION_TYPE_PUT_XYZ
        goal_put.pos_x = ud.drop_position_dst[color_type][0]
        goal_put.pos_y = ud.drop_position_dst[color_type][1]
        goal_put.pos_z = ud.drop_position_dst[color_type][2]
        if self.preempt_requested():
            return 'preempted'
        client.send_goal_and_wait(goal_put, rospy.Duration.from_sec(30))
        return 'success'


class ColorClassificationDemo:
    def __init__(self) -> None:

        filename = rospy.get_param("~vision_config")
        try:
            with open(filename, "r") as f:
                content = yaml.load(f.read())
        except:
            rospy.logerr("can't not open hsv file: ", filename)
            exit(1)

        # get LinearRegression k and b value
        self.linearression_kb_dst = {
            'k1': float(content['LinearRegression']['k1']),
            'b1': float(content['LinearRegression']['b1']),
            'k2': float(content['LinearRegression']['k2']),
            'b2': float(content['LinearRegression']['b2'])
        }
        rospy.loginfo("x axis k and b: %f, %f" % (
            self.linearression_kb_dst['k1'], self.linearression_kb_dst['b1']))
        rospy.loginfo("y axis k and b: %f, %f" % (
            self.linearression_kb_dst['k2'], self.linearression_kb_dst['b2']))

        # get hsv range
        self.object_dist = {
            'blue': {
                'lower_HSV': None,
                'upper_HSV': None
            },
            'red': {
                'lower_HSV': None,
                'upper_HSV': None
            },
            'green': {
                'lower_HSV': None,
                'upper_HSV': None
            }
        }
        for color in self.object_dist:
            self.object_dist[color]['lower_HSV'] = np.array(
                [content[color]['hmin'] / 2, content[color]['smin'], content[color]['vmin']])
            self.object_dist[color]['upper_HSV'] = np.array(
                [content[color]['hmax'] / 2, content[color]['smax'], content[color]['vmax']])

        # get drop position
        area_position = {  # 'drop area name': [x, y, z]
            # The y-axis of the A area(map v1.0) is offset by +2
            'A': [0.15, -0.26, 0.2],
            'B': [0.15, 0.24, 0.2],
            # The y-axis of the C area(map v1.0) is offset by +2
            'C': [0.26, -0.26, 0.2],
            'D': [0.26, 0.24, 0.2],
        }
        color_type = self.check_drop_position()
        rospy.logwarn('drop position found:' + str(color_type))
        self.drop_position_dst = {}
        for name in color_type:
            self.drop_position_dst[name] = area_position[color_type[name]]
        if len(self.drop_position_dst) < 4:
            rospy.logerr("one or more drop position not found")

    def check_drop_position(self):
        client = actionlib.SimpleActionClient(
            rospy.get_param("~arm_name", "sgr532") + '/' + 'sgr_ctrl', SGRCtrlAction)
        client.wait_for_server()

        color_type = {}
        drop_detector_area = {
            'width_size': 50,
            'height_size': 70,
            'A_x': 500, 'A_y': 220,
            'B_x': 180, 'B_y': 220,
            'C_x': 200, 'C_y': 220,
            'D_x': 480, 'D_y': 220
        }

        # to left
        goal_search = SGRCtrlGoal()
        goal_search.action_type = goal_search.ACTION_TYPE_XYZ_RPY
        goal_search.pos_x = 0.2
        goal_search.pos_y = -0.1
        goal_search.pos_z = 0.11
        goal_search.pos_pitch = 0.785
        goal_search.pos_yaw = -1.57
        client.send_goal_and_wait(goal_search, rospy.Duration.from_sec(30))

        cnt = 5
        # search A
        hsv_sum = [0, 0, 0]
        for i in range(cnt):
            img = rospy.wait_for_message(
                "/usb_cam/image_raw", Image, rospy.Duration.from_sec(3))
            cv_image1 = CvBridge().imgmsg_to_cv2(img, "bgr8")
            hsv = cv2.cvtColor(
                cv_image1[
                    drop_detector_area['A_y'] - drop_detector_area['height_size']:drop_detector_area['A_y'] + drop_detector_area['height_size'],
                    drop_detector_area['A_x'] - drop_detector_area['width_size']:drop_detector_area['A_x'] + drop_detector_area['width_size']
                ],
                cv2.COLOR_BGR2HSV
            )
            hsv_sum[0] += np.mean(hsv[:, :, 0])
            hsv_sum[1] += np.mean(hsv[:, :, 1])
            hsv_sum[2] += np.mean(hsv[:, :, 2])
        color_type[self.get_color(
            hsv_sum[0] / cnt, hsv_sum[1] / cnt, hsv_sum[2] / cnt)] = 'A'

        # search C
        hsv_sum = [0, 0, 0]
        for i in range(cnt):
            hsv = cv2.cvtColor(
                cv_image1[
                    drop_detector_area['C_y'] - drop_detector_area['height_size']:drop_detector_area['C_y'] + drop_detector_area['height_size'],
                    drop_detector_area['C_x'] - drop_detector_area['width_size']:drop_detector_area['C_x'] + drop_detector_area['width_size']
                ],
                cv2.COLOR_BGR2HSV
            )
            hsv_sum[0] += np.mean(hsv[:, :, 0])
            hsv_sum[1] += np.mean(hsv[:, :, 1])
            hsv_sum[2] += np.mean(hsv[:, :, 2])
        color_type[self.get_color(
            hsv_sum[0] / cnt, hsv_sum[1] / cnt, hsv_sum[2] / cnt)] = 'C'

        # to right
        goal_search.pos_x = 0.2
        goal_search.pos_y = 0.1
        goal_search.pos_z = 0.11
        goal_search.pos_pitch = 0.785
        goal_search.pos_yaw = 1.57
        client.send_goal_and_wait(goal_search, rospy.Duration.from_sec(30))

        cnt = 5

        # search B
        hsv_sum = [0, 0, 0]
        for i in range(cnt):
            img = rospy.wait_for_message(
                "/usb_cam/image_raw", Image, rospy.Duration.from_sec(3))
            cv_image1 = CvBridge().imgmsg_to_cv2(img, "bgr8")
            hsv = cv2.cvtColor(
                cv_image1[
                    drop_detector_area['B_y'] - drop_detector_area['height_size']:drop_detector_area['B_y'] + drop_detector_area['height_size'],
                    drop_detector_area['B_x'] - drop_detector_area['width_size']:drop_detector_area['B_x'] + drop_detector_area['width_size']
                ],
                cv2.COLOR_BGR2HSV
            )
            hsv_sum[0] += np.mean(hsv[:, :, 0])
            hsv_sum[1] += np.mean(hsv[:, :, 1])
            hsv_sum[2] += np.mean(hsv[:, :, 2])
        color_type[self.get_color(
            hsv_sum[0] / cnt, hsv_sum[1] / cnt, hsv_sum[2] / cnt)] = 'B'

        # search D
        hsv_sum = [0, 0, 0]
        for i in range(cnt):
            hsv = cv2.cvtColor(
                cv_image1[
                    drop_detector_area['D_y'] - drop_detector_area['height_size']:drop_detector_area['D_y'] + drop_detector_area['height_size'],
                    drop_detector_area['D_x'] - drop_detector_area['width_size']:drop_detector_area['D_x'] + drop_detector_area['width_size']
                ],
                cv2.COLOR_BGR2HSV
            )
            hsv_sum[0] += np.mean(hsv[:, :, 0])
            hsv_sum[1] += np.mean(hsv[:, :, 1])
            hsv_sum[2] += np.mean(hsv[:, :, 2])
        color_type[self.get_color(
            hsv_sum[0] / cnt, hsv_sum[1] / cnt, hsv_sum[2] / cnt)] = 'D'

        # return color type
        return color_type

    def get_color(self, H, S, V):
        rospy.logwarn("H=%d S=%d V=%d" % (H, S, V))
        if (90 < H < 120 and 100 < S):
            color_type = 'blue'
        elif (0 < H < 20 and 100 < S):
            color_type = 'red'
        elif (50 < H < 80 and 100 < S):
            color_type = 'green'
        else:
            color_type = 'None'

        rospy.logwarn("color_type is %s" % (color_type))
        return color_type

    def run(self):
        sm = smach.StateMachine(outcomes=['finish', 'error'])
        sm.userdata.object_dst = self.object_dist
        sm.userdata.drop_position_dst = self.drop_position_dst
        sm.userdata.linearression_kb_dst = self.linearression_kb_dst
        with sm:
            smach.StateMachine.add('FoundObject', FoundObject(),
                                   transitions={
                                       'success': 'GraspObject', 'preempted': 'finish', 'error': 'error'}
                                   )
            smach.StateMachine.add('GraspObject', GraspObject(),
                                   transitions={
                                       'success': 'DropObject', 'grasp_faild': 'FoundObject', 'preempted': 'finish', 'error': 'error'}
                                   )
            smach.StateMachine.add('DropObject', DropObject(),
                                   transitions={
                                       'success': 'FoundObject', 'preempted': 'finish', 'error': 'error'}
                                   )

        # https://github.com/ros/executive_smach/issues/7#issuecomment-14762723
        # Create a thread to execute the smach container
        smach_thread = threading.Thread(target=sm.execute)
        smach_thread.start()

        # Wait for ctrl-c
        rospy.spin()

        # Request the container to preempt
        sm.request_preempt()

        # Block until everything is preempted
        # (you could do something more complicated to get the execution outcome if you want it)
        smach_thread.join()

        rospy.loginfo("Finish...")


if __name__ == '__main__':
    rospy.init_node('color_classification_node', anonymous=False)
    ColorClassificationDemo().run()
