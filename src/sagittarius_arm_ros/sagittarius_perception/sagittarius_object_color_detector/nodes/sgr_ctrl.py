#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
Description: Saggitarius moveit application action
version: 1.00
Author: shudong.hong@nxrobo.com
Company: NXROBO (Shenzhen Creative Future Robot Co., Ltd.)
LastEditors: shudong.hong@nxrobo.com
'''

import math
import numpy as np
import rospy
import sys
from moveit_msgs.msg import MoveGroupActionFeedback
import moveit_commander
import tf. transformations as transformations

import actionlib
from sagittarius_object_color_detector.msg import SGRCtrlAction, SGRCtrlGoal, SGRCtrlResult, SGRCtrlFeedback
from sdk_sagittarius_arm.srv import ServoRtInfo, ServoRtInfoRequest

ispython3 = True if sys.version_info.major == 3 else False


class MoveItSGRTool:
    def __init__(self, init_pose=True, end_effector=None):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot_name = rospy.get_param("~robot_name", "sgr532")

        # Do you need to use Cartesian space motion planning, get the parameters, if not set, the default is True, that is, go straight
        self. cartesian = rospy. get_param('~cartesian', False)

        # Initialize the arm group in the robotic arm that needs to be controlled by the move group
        self.arm_group = moveit_commander.MoveGroupCommander('sagittarius_arm')

        # Initialize the gripper group in the robotic arm that needs to be controlled by the move group
        self.gripper = moveit_commander.MoveGroupCommander(
            'sagittarius_gripper')

        # When motion planning fails, allow replanning
        self.arm_group.allow_replanning(False)

        self.reference_frame = rospy.get_namespace()[1:] + 'base_link'
        # Set the reference coordinate system used by the target position
        self.arm_group.set_pose_reference_frame(self.reference_frame)

        # Set the reference coordinate system used by the target position
        self.gripper.set_pose_reference_frame(self.reference_frame)

        # Set the allowable error of position (unit: meter) and attitude (unit: radian)
        self.arm_group.set_goal_position_tolerance(0.001)
        self.arm_group.set_goal_orientation_tolerance(0.001)

        self.gripper.set_goal_joint_tolerance(0.001)
        # Set the maximum velocity and acceleration allowed
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_group.set_max_velocity_scaling_factor(0.5)

        # Set the name of the end effect point link
        if (end_effector is not None):
            self.arm_group.set_end_effector_link(end_effector)

        # Get the name of the terminal link
        self.end_effector_link = self.arm_group.get_end_effector_link()
        rospy.loginfo("end effector link: %s" % self.end_effector_link)

        #
        rospy.Subscriber("move_group/feedback", MoveGroupActionFeedback,
                        self._move_group_feedback_callback)
        self.moveit_group_status = "IDLE"

        # Control the robotic arm to return to the initial position first
        if init_pose:
            self.arm_group.set_named_target('home')
            self. arm_group. go()
            rospy. sleep(1)

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def _move_group_feedback_callback(self, fb):
        self.moveit_group_status = fb.feedback.state

    def isPlanSuccess(self, px, py, pz, roll=0, pitch=0, yaw=0):
        self.arm_group.set_pose_target([px, py, pz, roll, pitch, yaw])
        plan = self. arm_group. plan()

        if ispython3:
            plan = plan[1]

        return len(plan. joint_trajectory. points) != 0

    def gripper_catch(self):
        self.gripper.set_joint_value_target([-0.021, -0.021])
        ret = self.gripper.go()
        if ret:
            rospy. sleep(2)
        return ret

    def gripper_open(self):
        self.gripper.set_joint_value_target([0.0, 0.0])
        ret = self.gripper.go()
        if ret:
            rospy. sleep(2)
        return ret

    def to_pose_eular(self, times, px, py, pz, roll=0, pitch=0, yaw=0):
        self.arm_group.set_pose_target([px, py, pz, roll, pitch, yaw])
        plan = self. arm_group. plan()

        if ispython3: # python3 returns a list, subscript 1 is RobotTrajectory, and can also be judged by error_code:MoveItErrorCodes in the list
            plan = plan[1]

        if len(plan. joint_trajectory. points) != 0:
            self.arm_group.execute(plan)
            rospy. sleep(times)
            return True
        else:
            rospy.logwarn("plan can not be found!")
            return False

    def ee_target_offset(self, px, py, pz, roll=0, pitch=0, yaw=0, ee_type='grasp'):
        M = transformations.compose_matrix(
            angles=[roll, pitch, yaw], translate=[px, py, pz])
        if ee_type == 'grasp':
            M1 = np.dot(M, transformations.translation_matrix([-0.07, 0, 0]))
        else:
            M1 = M
        scale, shear, angles, translate, perspective = transformations.decompose_matrix(
            M1)
        return translate[0], translate[1], translate[2], angles[0], angles[1], angles[2]

    def ee_xyz_get_rpy(self, x, y, z):
        # Get yaw by Arctangent of x and y
        yaw = math. atan2(y, x)
        roll = 0
        pitch = 0

        # Use trigonometry to approximate pitch
        # Model: The robotic arm is abstracted into an isosceles triangle, the distance from the end of the robotic arm to the origin of coordinates is the base, and the length of the robotic arm is the sum of the two waists
        # Find the bottom corner
        a = 0.532 / 2
        b = 0.532 / 2
        c = math. sqrt(x*x + y*y + z*z)
        if a + b <= c:
            # triangle does not hold
            pitch = 0
        else:
            # Know the side length of the triangle to find the specified angle
            pitch = math.acos((a*a - b*b - c*c)/(-2*b*c)) - math.asin(z / c)

            # limited range
            if pitch > 1.57:
                pitch = 1.57
            elif pitch < 0:
                pitch = 0
        return roll, pitch, yaw

    def pose_stay(self):
        self.arm_group.set_joint_value_target(
            [0, 1.4, -1.48, 0, 1.6, 0])
        self. arm_group. go()

    def stop(self):
        self.arm_group.set_named_target('home')
        self. arm_group. go()
        rospy. sleep(1)
        self.arm_group.set_named_target('sleep')
        self. arm_group. go()
        rospy. sleep(1)
        self.gripper.set_joint_value_target([0, 0])
        self.gripper.go()
        rospy. sleep(0.5)


class CecelException(Exception):
    pass


class SGRCtrlActionServer:
    def __init__(self) -> None:
        self.sgr_tool = MoveItSGRTool()
        rospy.wait_for_service('get_servo_info', 3)
        self.servo_info_srv = rospy.ServiceProxy('get_servo_info', ServoRtInfo)
        self._server = actionlib.SimpleActionServer(
            'sgr_ctrl', SGRCtrlAction, self. execute, False)
        self._server.start()

    def execute(self, goal:SGRCtrlGoal):
        r = rospy.Rate(1)

        resp = SGRCtrlResult()
        fb = SGRCtrlFeedback()

        # Initialize the gripper
        fb.step = fb.EXEC_GRASP
        self._server.publish_feedback(fb)
        if goal.grasp_type is SGRCtrlGoal.GRASP_CLOSE:
            self.sgr_tool.gripper_catch()
        elif goal.grasp_type is SGRCtrlGoal.GRASP_OPEN:
            self.sgr_tool.gripper_open()

        # start planning
        fb.step = fb.PLANNING
        self._server.publish_feedback(fb)
        if goal.action_type is goal.ACTION_TYPE_DEFINE_STAY:
            # specify action
            fb.step = fb.EXEC_POSITION
            self._server.publish_feedback(fb)
            self.sgr_tool.pose_stay()
        elif goal.action_type is goal.ACTION_TYPE_DEFINE_SAVE:
            # specify action
            fb.step = fb.EXEC_POSITION
            self._server.publish_feedback(fb)
            self.sgr_tool.stop()
        else:
            if goal.action_type in [SGRCtrlGoal.ACTION_TYPE_XYZ_RPY, SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY, SGRCtrlGoal.ACTION_TYPE_PUT_XYZ_RPY]:
                # Specify end pose
                roll = goal. pos_roll
                pitch = goal. pos_pitch
                yaw = goal. pos_yaw
            else:
                # Dynamically calculate the end pose
                roll, pitch, yaw = self. sgr_tool. ee_xyz_get_rpy(goal. pos_x, goal. pos_y, goal. pos_z)
        
            x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(
                goal. pos_x, goal. pos_y, goal. pos_z,
                roll, pitch, yaw
            )

            # Check if pose is reachable
            try:
                if not self.sgr_tool.isPlanSuccess(x, y, z, roll, pitch, yaw):
                    raise
                if goal. action_type in [
                    SGRCtrlGoal. ACTION_TYPE_PICK_XYZ,
                    SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY,
                    SGRCtrlGoal. ACTION_TYPE_PUT_XYZ,
                    SGRCtrlGoal.ACTION_TYPE_PUT_XYZ_RPY
                ]:
                    if not self.sgr_tool.isPlanSuccess(x, y, z + 0.04, roll, pitch, yaw):
                        raise
                    if not self.sgr_tool.isPlanSuccess(x, y, z + 0.12, roll, pitch, yaw):
                        raise
            except:
                rospy.logwarn("Plan could not found")
                resp.result = resp.PLAN_NOT_FOUND
                self._server.set_aborted(resp)
                return
            
            # execute plan
            fb.step = fb.EXEC_POSITION
            self._server.publish_feedback(fb)
            try:
                # Grab and place actions
                if goal. action_type in [
                    SGRCtrlGoal. ACTION_TYPE_PICK_XYZ,
                    SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY,
                    SGRCtrlGoal. ACTION_TYPE_PUT_XYZ,
                    SGRCtrlGoal.ACTION_TYPE_PUT_XYZ_RPY
                ]:
                    if self._server.is_preempt_requested():
                        raise CecelException("Preempt !!!")
                    self.sgr_tool.to_pose_eular(1, x, y, z + 0.04, roll, pitch, yaw)

                    if self._server.is_preempt_requested():
                        raise CecelException("Preempt !!!")
                    self.sgr_tool.to_pose_eular(0.2, x, y, z, roll, pitch, yaw)

                    fb.step = fb.EXEC_GRASP
                    self._server.publish_feedback(fb)
                    if self._server.is_preempt_requested():
                        raise CecelException("Preempt !!!")
                    if goal.action_type in [SGRCtrlGoal.ACTION_TYPE_PICK_XYZ, SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY]:
                        self.sgr_tool.gripper_catch()
                    elif goal.action_type in [SGRCtrlGoal.ACTION_TYPE_PUT_XYZ, SGRCtrlGoal.ACTION_TYPE_PUT_XYZ_RPY]:
                        self.sgr_tool.gripper_open()
                
                    fb.step = fb.EXEC_POSITION
                    self._server.publish_feedback(fb)
                    if self._server.is_preempt_requested():
                        raise CecelException("Preempt !!!")
                    self.sgr_tool.to_pose_eular(0.2, x, y, z + 0.08, roll, pitch, yaw)

                    # if self._server.is_preempt_requested():
                    # raise CecelException("Preempt !!!")
                    # self. sgr_tool. pose_stay()

                    # Gripper judgment
                    if goal.action_type in [SGRCtrlGoal.ACTION_TYPE_PICK_XYZ, SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY]:
                        ret = self.servo_info_srv.call(ServoRtInfoRequest(servo_id=7))
                        if abs(ret.payload) < 24:
                            resp.result = resp.GRASP_FAILD
                            self._server.set_aborted(resp)
                            return

                # perform an action
                else:
                    if self._server.is_preempt_requested():
                        raise CecelException("Preempt !!!")
                    self.sgr_tool.to_pose_eular(0.2, x, y, z, roll, pitch, yaw)
            except CecelException as e:
                rospy. logwarn(e)
                resp.result = resp.PREEMPT
                self._server.set_aborted()
            except Exception as e:
                rospy.logerr(e)
                resp.result = resp.ERROR
                self._server.set_aborted()
            
        resp.result = resp.SUCCESS
        self._server.set_succeeded(resp)


if __name__ == '__main__':
    rospy.init_node("sgr_ctrl_action", anonymous=True)
    rospy. sleep(5)

    ser = SGRCtrlActionServer()

    rospy. spin()