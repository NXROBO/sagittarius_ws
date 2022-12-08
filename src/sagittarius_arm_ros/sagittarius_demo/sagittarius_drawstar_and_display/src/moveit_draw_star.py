#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy,sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from std_msgs.msg import String


class MoveItDrawStarDemo:
     waypoints = []

     def __init__(self):
         # API for initializing move_group
         moveit_commander.roscpp_initialize(sys.argv)

         # Initialize the arm group in the robotic arm that needs to be controlled by the move group
         arm = MoveGroupCommander('sagittarius_arm')
         gripper = MoveGroupCommander('sagittarius_gripper')

         # Allows re-planning when motion planning fails
         arm. allow_replanning(True)
        
         # Set the reference coordinate system used by the target position
         reference_frame = 'world'
         arm.set_pose_reference_frame(reference_frame)
         gripper.set_pose_reference_frame(reference_frame) #gripper
                
         # Set the allowable error of position (unit: meter) and attitude (unit: radian)
         arm.set_goal_position_tolerance(0.01)
         arm.set_goal_orientation_tolerance(0.01)
         arm.set_goal_joint_tolerance(0.02)

         # Set the maximum velocity and acceleration allowed
         arm.set_max_acceleration_scaling_factor(0.5)
         arm.set_max_velocity_scaling_factor(0.5)
        
         # Get the name of the terminal link
         end_effector_link = arm. get_end_effector_link()

         # Control the robotic arm to return to the initial position first
         arm.set_named_target('home')
         arm.go()

         # Control the gripper of the robotic arm to grab the pen
         gripper.set_joint_value_target([-0.0197, -0.0197])
         gripper.go()
         rospy. sleep(2)

         # The position of the five-pointed star outline point relative to the reference point
         # (0.05, 0)
         # /// \\\
         # /// \\\
         # /// \\\
         # (0.035, 0.02)===(0.035, 0.005) (0.035, -0.005)===(0.035, -0.02)
         # \\\ ///
         # \\\ ///
         # \\\ ///
         # (0.025, 0.005) (0.025, -0.005)
         # /// \\\
         # /// (0.024, 0) \\\
         # /// /////////// \\\\\\\\\\\ \\\
         # (0.012, 0.015) (0.012, -0.015)
         # (0, 0) base point
         points_list = [
             [0.05, 0, 0.04],
             [0.05, 0, 0], # point above the head
             [0.035, 0.005, 0], # left shoulder dimple
             [0.035, 0.02, 0], # left shoulder bump
             [0.025, 0.005, 0], # left waist pit
             [0.012, 0.015, 0], # left foot
             [0.024, 0, 0],
             [0.012, -0.015, 0], # right foot
             [0.025, -0.005, 0], # right waist pit
             [0.035, -0.02, 0], # right shoulder bump
             [0.035, -0.005, 0], # right shoulder dimple
             [0.05, 0, 0], # point above the head
             [0.05, 0, 0.04]
         ]

         # Pentagram size multiple
         star_size = 4

         # Set the position of the five-pointed star reference point
         target_pose_base = PoseStamped()
         target_pose_base.header.frame_id = reference_frame
         target_pose_base.header.stamp = rospy.Time.now()
         target_pose_base.pose.position.x = 0.22
         target_pose_base.pose.position.y = 0.0
         target_pose_base.pose.position.z = 0.1
         target_pose_base.pose.orientation.w = 1.0

         # Go to the pen position, ready to pen
         wpose = deepcopy(target_pose_base.pose)
         wpose.position.x += points_list[0][0] * star_size
         wpose.position.y += points_list[0][1] * star_size
         wpose.position.z += points_list[0][2] * star_size

         # Set the target pose of the robot arm terminal motion
         arm.set_pose_target(wpose, end_effector_link)
         arm.go()
         rospy. sleep(2)

         # Initialize waypoint list
         waypoints = []

         # Add the points of the five-pointed star outline to the list
         for i in range(1, len(points_list) - 1, 1):
             wpose = deepcopy(target_pose_base.pose)
             wpose.position.x += points_list[i][0] * star_size
             wpose.position.y += points_list[i][1] * star_size
             wpose.position.z += points_list[i][2] * star_size
             waypoints.append(deepcopy(wpose))

         # Return to the space above the next pen position, this position is also added to the list
         wpose = deepcopy(target_pose_base.pose)
         wpose.position.x += points_list[0][0] * star_size
         wpose.position.y += points_list[0][1] * star_size
         wpose.position.z += points_list[0][2] * star_size
         waypoints.append(deepcopy(wpose))

         fraction = 0.0 #path planning coverage
         maxtries = 100 #Maximum number of planning attempts
         attempts = 0 #The number of planning attempts has been made
         # Set the current state of the robot arm as the initial state of motion
         arm.set_start_state_to_current_state()
         # Try to plan a path in Cartesian space, pass through all waypoints in turn, and complete the arc trajectory
         while fraction < 1.0 and attempts < maxtries:
             (plan, fraction) = arm. compute_cartesian_path (
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
             arm. execute(plan)
             rospy.loginfo("Path execution complete.")
         # If the path planning fails, print the failure message
         else:
             rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

         rospy. sleep(1)

         # Control the robotic arm back to the initial position
         arm.set_named_target('home')
         arm.go()
         rospy. sleep(1)

         # release pen
         gripper.set_named_target('open')
         gripper.go()
         rospy. sleep(2)

         # Control the robotic arm to return to the initial position first
         arm.set_named_target('sleep')
         arm.go()
         rospy. sleep(1)
        
         # Close and exit moveit
         moveit_commander.roscpp_shutdown()
         moveit_commander.os._exit(0)


if __name__ == "__main__":
     try:
         # Initialize the ROS node
         rospy.init_node('moveit_draw_star_demo', anonymous=True)

         rospy.wait_for_message("/start_topic", String)
         MoveItDrawStarDemo()
     except rospy.ROSInterruptException:
         pass