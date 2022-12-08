#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from copy import deepcopy
import rospy
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
import math


def move2pose(times, px, py, pz, ox, oy, oz, ow):
    wpose = deepcopy(arm.get_current_pose(end_effector_link).pose) # copy object
    wpose.position.x = px
    wpose.position.y = py
    wpose.position.z = pz
    wpose.orientation.x = ox
    wpose.orientation.y = oy
    wpose.orientation.z = oz
    wpose.orientation.w = ow
    arm.set_pose_target(wpose) # free curve
    arm.go()
    rospy. sleep(times)


def eular2orientation(pitch, yaw, roll):
    ox = math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2) + \
        math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    oy = math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2) + \
        math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
    oz = math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2) - \
        math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    ow = math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2) - \
        math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
    return ox, oy, oz, ow


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize the ROS node
    rospy.init_node('moveit_cartesian_demo', anonymous=True)

    # Do you need to use Cartesian space motion planning, get the parameters, if not set, the default is True, that is, go straight
    cartesian = rospy. get_param('~cartesian', False)

    # Initialize the arm group in the robotic arm that needs to be controlled by the move group
    arm = MoveGroupCommander('sagittarius_arm')

    # Initialize the gripper group in the robotic arm that needs to be controlled by the move group
    gripper = MoveGroupCommander('sagittarius_gripper')
    gripper1 = moveit_commander. MoveGroupCommander('sagittarius_gripper')
    # When motion planning fails, allow replanning
    arm. allow_replanning(False)

    # Set the reference coordinate system used by the target position
    arm.set_pose_reference_frame('world')

    # Set the reference coordinate system used by the target position
    gripper.set_pose_reference_frame('world')

    # Set the allowable error of position (unit: meter) and attitude (unit: radian)
    arm.set_goal_position_tolerance(0.0001)
    arm.set_goal_orientation_tolerance(0.0001)

    gripper1.set_goal_joint_tolerance(0.001)
    # Set the maximum velocity and acceleration allowed
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    # Get the name of the terminal link
    end_effector_link = arm. get_end_effector_link()

    # reset
    arm.set_named_target('home')
    arm.go()
    rospy. sleep(1)

    # Get the current position as the starting point
    pose = deepcopy(arm. get_current_pose(end_effector_link). pose)
    px = pose.position.x
    py = pose.position.y
    pz = pose.position.z
    ox = pose.orientation.x
    oy = pose.orientation.y
    oz = pose.orientation.z
    ow = pose.orientation.w
    while not rospy.is_shutdown(): # Move to a square with four points
        px -= 0.1
        move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if rospy.is_shutdown():
            break

        py -= 0.1
        move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if rospy.is_shutdown():
            break

        pz -= 0.1
        move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if rospy.is_shutdown():
            break

        py += 0.20
        move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if rospy.is_shutdown():
            break

        pz += 0.1
        move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if rospy.is_shutdown():
            break

        break

    arm.set_named_target('home')
    arm.go()
    rospy. sleep(1)
    arm.set_named_target('sleep')
    arm.go()
    rospy. sleep(1)
    gripper1.set_joint_value_target([0, 0])
    gripper1.go()
    rospy. sleep(0.5)

    # Close and exit moveit
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)