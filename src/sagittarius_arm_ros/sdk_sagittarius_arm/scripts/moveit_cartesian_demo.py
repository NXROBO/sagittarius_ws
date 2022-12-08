#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy, sys
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
        cartesian = rospy. get_param('~cartesian', True)
                    
        # Initialize the arm group in the robotic arm that needs to be controlled by the move group
        arm = MoveGroupCommander('arm')
    
        # When motion planning fails, allow replanning
        arm. allow_replanning(True)
    
        # Set the reference coordinate system used by the target position
        arm.set_pose_reference_frame('sagittarius_arm_link')
                
        # Set the allowable error of position (unit: meter) and attitude (unit: radian)
         arm.set_goal_position_tolerance(0.001)
         arm.set_goal_orientation_tolerance(0.001)
        
         # Set the maximum velocity and acceleration allowed
         arm.set_max_acceleration_scaling_factor(0.5)
         arm.set_max_velocity_scaling_factor(0.5)
        
         # Get the name of the terminal link
         end_effector_link = arm. get_end_effector_link()
 
         # Control the robotic arm to return to the initial position first
         arm.set_named_target('home')
         arm.go()
         rospy. sleep(1)
                                               
        # Get the current pose data as the starting pose of the robot arm movement
        start_pose = arm.get_current_pose(end_effector_link).pose
            
        # Initialize waypoint list
        waypoints = []

        # If True, add the initial pose to the waypoint list
        if cartesian:
            waypoints.append(start_pose)
        
        # Set the waypoint data and add to the waypoint list, all points are added
        wpose = deepcopy(start_pose)#copy object
        wpose.position.z -= 0.1

        if cartesian: #If set to True, then go straight
            waypoints.append(deepcopy(wpose))
        else: #otherwise follow the curve
            arm.set_pose_target(wpose) #free curve
            arm.go()
            rospy. sleep(1)
 
        wpose.position.x += 0.15
 
        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)
        
        wpose.position.y += 0.1
 
        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)
 
        wpose.position.x -= 0.15
        wpose.position.y -= 0.1
 
        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)
 
 
        # planning process
        if cartesian:
        fraction = 0.0 #path planning coverage
        maxtries = 100 #Maximum number of planning attempts
        attempts = 0 #The number of planning attempts has been made

        # Set the current state of the robot arm as the initial state of motion
        arm.set_start_state_to_current_state()

        # Try to plan a path in Cartesian space, passing through all waypoints in turn
        while fraction < 1.0 and attempts < maxtries:
                #Planning path, fraction returns 1 to indicate successful planning
        (plan, fraction) = arm. compute_cartesian_path (
                                        waypoints, # waypoint poses, list of waypoints, here are 5 points
                                        0.01, # eef_step, the terminal step value, calculate the inverse solution every 0.01m to judge whether it is reachable
                                        0.0, # jump_threshold, jump threshold, set to 0 means jumping is not allowed
                                        True) # avoid_collisions, obstacle avoidance planning
		    
            # Accumulate the number of attempts
            attempts += 1

            # print motion planning process
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

		# If the path planning is successful (100% coverage), start to control the movement of the robotic arm
		if fraction == 1.0:
		    rospy.loginfo("Path computed successfully. Moving the arm.")
		    arm.execute(plan)
		    rospy.loginfo("Path execution complete.")
		# If path planning fails, print failure information
		else:
		    rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
 
		rospy.sleep(1)
 
        # Control the robotic arm to return to the initial position first
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
        
        # close and exit MoveIt
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass

