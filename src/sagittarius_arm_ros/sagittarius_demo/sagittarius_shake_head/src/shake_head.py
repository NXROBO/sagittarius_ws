#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import rospy
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import PoseStamped

## This example script mimics a human tiliting their head in curiosity,
## Then nodding agreeably.

rospy.init_node('sagittarius_arm_shake', anonymous=True)

rospy.sleep(2)

REFERENCE_FRAME = 'world'

# Initialize the moveit_commander module
moveit_commander.roscpp_initialize(sys.argv)

# Create a MoveGroupCommander object for the "sagittarius_arm" group
arm = moveit_commander.MoveGroupCommander("sagittarius_arm")
gripper = moveit_commander.MoveGroupCommander('sagittarius_gripper')

# Allows re-planning when motion planning fails
arm.allow_replanning(True)
# arm.set_planning_time(10)

# Set the reference coordinate system used by the target position
arm.set_pose_reference_frame(REFERENCE_FRAME)
gripper.set_pose_reference_frame(REFERENCE_FRAME)

# Set the allowable error of position (unit: meter) and attitude (unit: radian)
arm.set_goal_position_tolerance(0.01)
arm.set_goal_orientation_tolerance(0.01)
arm.set_goal_joint_tolerance(0.02)

# Set the maximum velocity and acceleration allowed
arm.set_max_acceleration_scaling_factor(0.5)
arm.set_max_velocity_scaling_factor(0.5)

# Get the name of the terminal link
end_effector_link = arm.get_end_effector_link()

arm.set_named_target('sleep')
arm.go()
rospy.sleep(1)

points_list = [
    #p_x, p_y, p_z, o_w, o_x, o_y, o_z
    [0.0, 0.0, 0.0, 0.0, 0.2, 0, 0.0],
    [0.0, 0.0, 0.0, 0.0, -0.2, 0, 0.0],
    [0.0, 0.0, 0.00, 0.0, 0.3, 0, 0.0],
    [0.0, 0.0, 0.00, 0.0, -0.3, 0, 0.0],
    [0.0, 0.0, 0.04, 0.0, 0, 0, 0.0],
    [0.0, 0.0, 0.03, 0.6, 0, 0, 0],
    [0.0, 0.0, -0.03, -0.6, 0, -0, 0],
    [0.0, 0.0, 0.03, 0.6, 0, 0, 0],
    [0.0, 0.0, -0.03, -0.6, 0, -0, 0],
    [0.0, 0.0, 0.03, 0.6, 0.0, 0, 0]
]

# Set the target pose for the arm to an upright position
target_pose = PoseStamped()
target_pose.header.frame_id = REFERENCE_FRAME
target_pose.header.stamp = rospy.Time.now()
target_pose.pose.position.x = 0.3
target_pose.pose.position.y = 0.00
target_pose.pose.position.z = 0.25
target_pose.pose.orientation.w = 1.0

waypoints = [target_pose.pose]
arm.set_pose_target(target_pose.pose, end_effector_link)
arm.go()
rospy.sleep(2)

# Add the points of orientation of the head shake to the list
for i in range(0, len(points_list) - 1, 1):
    wpose = deepcopy(target_pose.pose)
    wpose.position.x += points_list[i][0]
    wpose.position.y += points_list[i][1]
    wpose.position.z += points_list[i][2]
    wpose.orientation.w += points_list[i][3]
    wpose.orientation.x += points_list[i][4]
    wpose.orientation.y += points_list[i][5]
    wpose.orientation.z += points_list[i][6]
    waypoints.append(deepcopy(wpose))

fraction = 0.0 #path planning coverage
maxtries = 100 #Maximum number of planning attempts
attempts = 0 #The number of planning attempts has been made
# Set the current state of the robot arm as the initial state of motion
arm.set_start_state_to_current_state()
# Try to plan a path in Cartesian space, pass through all waypoints in turn, and complete the arc trajectory
while fraction < 1.0 and attempts < maxtries:
    (plan, fraction) = arm.compute_cartesian_path (
                            waypoints, # waypoint poses, list of waypoints
                            0.01, # eef_step, terminal step value
                            0.0, #jump_threshold，jump threshold
                            True) # avoid_collisions, obstacle avoidance planning
    # Accumulate the number of attempts
    attempts += 1
    # print motion planning process
    if attempts % 10 == 0:
        rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

# If the path planning is successful (coverage rate 100%), start to control the movement of the robotic arm
if fraction == 1.0:
    rospy.loginfo("Path computed successfully. Moving the arm.")
    arm.execute(plan)
    rospy.loginfo("Path execution complete.")
# If the path planning fails, print the failure message
else:
    rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

rospy.sleep(4)

arm.set_named_target('sleep')
arm.go()
rospy.sleep(1)

# Shutdown the moveit_commander module
moveit_commander.roscpp_shutdown()
