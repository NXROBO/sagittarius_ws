#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy,sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy
import math

class MoveItCartesianDemo:
    global arm
    global cartesian
    global end_effector_link

    def move2pose_eular(self, times, px, py, pz, pitch, yaw, roll):
        global arm
        global cartesian
        global end_effector_link
        # Get the current pose data as the starting pose of the robot arm movement
        start_pose = arm.get_current_pose(end_effector_link).pose
        print(start_pose)
        # Initialize waypoint list
        waypoints = []

# If True, add the initial pose to the waypoint list
        if cartesian:
            waypoints.append(start_pose)
        
        # Set the waypoint data and add to the waypoint list, all points are added
        wpose = deepcopy(start_pose)#copy object

        wpose.position.z -= 0.25
        wpose.position.x = px
        wpose.position.y = py
        wpose.position.z = pz
        wpose.orientation.x=math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.cos(yaw/2) *math. sin(roll/2)
        wpose.orientation.y=math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.sin(yaw/2) *math. sin(roll/2)
        wpose.orientation.z=math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.cos(yaw/2) *math. sin(roll/2)
        wpose.orientation.w=math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.sin(yaw/2) *math. sin(roll/2)
        print(wpose)
        if cartesian: #If set to True, then go straight
            waypoints.append(deepcopy(wpose))
        else: #otherwise follow the curve
            arm.set_pose_target(wpose) #free curve
            arm.go()
            rospy. sleep(times)

    def __init__(self):
        global arm
        global cartesian
        global end_effector_link
        #
        moveit_commander.roscpp_initialize(sys.argv)

        #Initialize the ROS node
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
        arm.set_pose_reference_frame('world')

        # Set the reference coordinate system used by the target position
        gripper.set_pose_reference_frame('world')
            
        # Set the allowable error of position (unit: meter) and attitude (unit: radian)
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)

        gripper1.set_goal_joint_tolerance(0.001)
        # Set the maximum velocity and acceleration allowed
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
    
        # Get the name of the terminal link
        end_effector_link = arm. get_end_effector_link()
        # Control the robotic arm to return to the initial position first
        arm.set_named_target('home')
        arm.go()
        rospy. sleep(1)

        while not rospy.is_shutdown():
        
            # to A
            if not rospy.is_shutdown():
                self.move2pose_eular(2, 0.05, 0.15, 0.17, math.pi / 2, 0, 0)
            else:
                break
            gripper1.set_joint_value_target([0, 0])
            gripper1.go()
            rospy. sleep(2)

            # pick up A
            if not rospy.is_shutdown():
                self.move2pose_eular(1, 0.05, 0.15, 0.09, math.pi / 2, 0, 0)
            else:
                break
            gripper1.set_joint_value_target([-0.022, -0.022])
            gripper1.go()
            rospy. sleep(2)

            if not rospy.is_shutdown():
                self.move2pose_eular(1, 0.05, 0.15, 0.17, math.pi / 2, 0, 0)
            else:
                break

            # to tmp
            if not rospy.is_shutdown():
                self.move2pose_eular(2, 0.18, 0, 0.17, math.pi / 2, 0, 0)
            else:
                break

            # putdown A
            if not rospy.is_shutdown():
                self.move2pose_eular(1, 0.18, 0, 0.1, math.pi / 2, 0, 0)
            else:
                break
            gripper1.set_joint_value_target([0, 0])
            gripper1.go()
            rospy. sleep(2)
            if not rospy.is_shutdown():
                self.move2pose_eular(1, 0.18, 0, 0.17, math.pi / 2, 0, 0)
            else:
                break

            # to B
            if not rospy.is_shutdown():
                self.move2pose_eular(2, 0.05, -0.15, 0.17, math.pi / 2, 0, 0)
            else:
                break

            # pick up B
            if not rospy.is_shutdown():
                self.move2pose_eular(1, 0.05, -0.15, 0.09, math.pi / 2, 0, 0)
            else:
                break
            gripper1.set_joint_value_target([-0.022, -0.022])
            gripper1.go()
            rospy. sleep(2)

            if not rospy.is_shutdown():
                self.move2pose_eular(1, 0.05, -0.15, 0.17, math.pi / 2, 0, 0)
            else:
                break

            # to A
            if not rospy.is_shutdown():
                self.move2pose_eular(2, 0.05, 0.15, 0.17, math.pi / 2, 0, 0)
            else:
                break

            # put down B
            if not rospy.is_shutdown():
                self.move2pose_eular(1, 0.05, 0.15, 0.1, math.pi / 2, 0, 0)
            else:
                break
            gripper1.set_joint_value_target([0, 0])
            gripper1.go()
            rospy.sleep(2)

            if not rospy.is_shutdown():
                self.move2pose_eular(1, 0.05, 0.15, 0.17, math.pi / 2, 0, 0)
            else:
                break

            # to tmp
            if not rospy.is_shutdown():
                self.move2pose_eular(2, 0.18, 0, 0.17, math.pi / 2, 0, 0)
            else:
                break

            # pick up A
            if not rospy.is_shutdown():
                self.move2pose_eular(1, 0.18, 0, 0.09, math.pi / 2, 0, 0)
            else:
                break
            gripper1.set_joint_value_target([-0.022, -0.022])
            gripper1.go()
            rospy.sleep(2)
            if not rospy.is_shutdown():
                self.move2pose_eular(1, 0.18, 0, 0.17, math.pi / 2, 0, 0)
            else:
                break

            #  to B
            if not rospy.is_shutdown():
                self.move2pose_eular(2, 0.05, -0.15, 0.17, math.pi / 2, 0, 0)
            else:
                break

            # put down B
            if not rospy.is_shutdown():
                self.move2pose_eular(1, 0.05, -0.15, 0.1, math.pi / 2, 0, 0)
            else:
                break
            gripper1.set_joint_value_target([0, 0])
            gripper1.go()
            rospy.sleep(2)

            if not rospy.is_shutdown():
                self.move2pose_eular(1, 0.05, -0.15, 0.17, math.pi / 2, 0, 0)
            else:
                break

        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
        arm.set_named_target('sleep')
        arm.go()
        rospy.sleep(1)
        gripper1.set_joint_value_target([0, 0])
        gripper1.go()
        rospy.sleep(0.5)
        # close and exit moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

 
if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass

