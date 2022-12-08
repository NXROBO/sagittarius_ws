#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy,sys
import moveit_commander
from moveit_commander import MoveGroupCommander
import math

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)

    #Initialize the ROS node
    rospy.init_node('moveit_cartesian_demo', anonymous=True)

    # Do you need to use Cartesian space motion planning, get the parameters, if not set, the default is True, that is, go straight
    cartesian = rospy. get_param('~cartesian', False)
                
    # Initialize the arm group in the robotic arm that needs to be controlled by the move group
    arm = MoveGroupCommander('sagittarius_arm')

    # Initialize the gripper group in the robotic arm that needs to be controlled by the move group
    gripper = MoveGroupCommander('sagittarius_gripper')
    gripper1 = moveit_commander. MoveGroupCommander('sagittarius_gripper')
    # When motion planning fails, allow replanning
    arm. allow_replanning(True)

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

    while not rospy.is_shutdown():
        print("\nset joint 2: 0° -> 45°")
        print("set joint 3: 0° -> -45°")
        tjoint = [0, math.pi / 4, -math.pi / 4, 0, 0, 0]
        arm.set_joint_value_target(tjoint)
        arm.go()
        rospy. sleep(0.5)
        if rospy.is_shutdown():
            break
        print(arm. get_current_pose(end_effector_link). pose)

        print("\nset joint 4: 0° -> 90°")
        print("set joint 6: 0° -> 90°")
        tjoint = [0, math.pi / 4, -math.pi / 4, math.pi / 2, 0, math.pi / 2]
        arm.set_joint_value_target(tjoint)
        arm.go()
        rospy. sleep(0.5)
        if rospy.is_shutdown():
            break
        print(arm. get_current_pose(end_effector_link). pose)

        print("\nset joint 4: 90° -> -90°")
        print("set joint 6: 90° -> -90°")
        tjoint = [0, math.pi / 4, -math.pi / 4, -math.pi / 2, 0, -math.pi / 2]
        arm.set_joint_value_target(tjoint)
        arm.go()
        rospy. sleep(0.5)
        if rospy.is_shutdown():
            break
        print(arm. get_current_pose(end_effector_link). pose)

        print("\nset joint 3: 0° -> 90°")
        print("set joint 5: 0° -> -90°")
        tjoint = [0, 0, math.pi / 2, 0, -math.pi / 2, 0]
        arm.set_joint_value_target(tjoint)
        arm.go()
        rospy. sleep(0.5)
        if rospy.is_shutdown():
            break
        print(arm. get_current_pose(end_effector_link). pose)

        print("\nset joint 4: 0° -> 45°")
        tjoint = [0, 0, math.pi / 2, math.pi / 4, -math.pi / 2, 0]
        arm.set_joint_value_target(tjoint)
        arm.go()
        rospy. sleep(0.5)
        if rospy.is_shutdown():
            break
        print(arm. get_current_pose(end_effector_link). pose)

        print("\nset joint 4: 45° -> -45°")
        tjoint = [0, 0, math.pi / 2, -math.pi / 4, -math.pi / 2, 0]
        arm.set_joint_value_target(tjoint)
        arm.go()
        rospy. sleep(0.5)
        if rospy.is_shutdown():
            break
        print(arm. get_current_pose(end_effector_link). pose)

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