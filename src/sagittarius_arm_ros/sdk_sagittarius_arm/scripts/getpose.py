#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy


class MoveItCartesianDemo:
    def __init__(self):
        #
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        # Do you need to use Cartesian space motion planning, get the parameters, if not set, the default is True, that is, go straight
        cartesian = rospy. get_param('~cartesian', False)

        # Initialize the arm group in the robotic arm that needs to be controlled by the move group
        arm = MoveGroupCommander('arm')

        # Initialize the gripper group in the robotic arm that needs to be controlled by the move group
        gripper = MoveGroupCommander('gripper')
        gripper1 = moveit_commander. MoveGroupCommander('gripper')
        # When motion planning fails, allow replanning
        arm. allow_replanning(True)

        # Set the reference coordinate system used by the target position
        arm.set_pose_reference_frame('sagittarius_arm_link')

        # Set the reference coordinate system used by the target position
        gripper.set_pose_reference_frame('sagittarius_arm_link')

        # Set the allowable error of position (unit: meter) and attitude (unit: radian)
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)

        gripper1.set_goal_joint_tolerance(0.001)
        # Set the maximum velocity and acceleration allowed
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # Get the name of the terminal link
        end_effector_link = arm. get_end_effector_link()

        while not rospy.is_shutdown():
            # Get the current pose data as the starting pose of the robot arm movement
            start_pose = arm.get_current_pose(end_effector_link).pose
            print(start_pose)
            rospy. sleep(0.1)

        # Close and exit moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass