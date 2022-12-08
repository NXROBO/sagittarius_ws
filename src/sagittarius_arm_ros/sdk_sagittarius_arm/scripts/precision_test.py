#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy,sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItCartesianDemo:
    global arm
    global cartesian
    global end_effector_link

    def move2pose(self, times, px, py, pz, ox, oy, oz, ow):
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
        print("to the top of the green square")
        wpose.position.x = px
        wpose.position.y = py
        wpose.position.z = pz
        wpose.orientation.x = ox
        wpose.orientation.y = oy
        wpose.orientation.z = oz
        wpose.orientation.w = ow
        print(wpose)
        if cartesian: #If set to True, then go straight
            waypoints.append(deepcopy(wpose))
        else: #otherwise follow the curve
            arm.set_pose_target(wpose) #free curve
            arm.go()
            rospy. sleep(times*0.95)

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
        # Control the jaws of the machine to open
        ''' gripper. set_named_target('open')
        gripper.go()
        rospy. sleep(1)

        # Control machine gripper closed
        gripper.set_named_target('close')
        gripper.go()
        rospy. sleep(1)
        print("gripper open")
        # Control the jaws of the machine to open
        gripper1. set_named_target('open')
        gripper1.go()
        rospy. sleep(3)'''
        # Control the robotic arm to return to the initial position first
        arm.set_named_target('home')
        arm.go()
        rospy. sleep(1)
        # clip up
        print("Clip up")
        gripper1.set_joint_value_target([-0.005, -0.005])
        gripper1.go()
        rospy.sleep(1) #print("gripper set_joint_value_target([0.1,0.1])")
        # Set the target position of the gripper and control the movement of the gripper
        #gripper1.set_joint_value_target([-0.01, -0.01])
        #gripper1.go()
        #rospy. sleep(1)
        # position:
        # x: 0.1
        #y: 0.15
        # z: 0.1
        # orientation:
        # x: 0
        #y: 0.689296120492
        # z: 0.00956006494475
        # w: 0.723968564058
        while not rospy.is_shutdown():
            if not rospy.is_shutdown():
                self.move2pose(1, 0.15, -0.1, 0.25, 0, 0, 0, 1)
            else:
                break
            if not rospy.is_shutdown():
                self.move2pose(1, 0.25, -0.1, 0.25, 0, 0, 0, 1)
            else:
                break
        

        arm.set_named_target('home')
        arm.go()
        rospy. sleep(1)
        arm.set_named_target('sleep')
        arm.go()
        rospy. sleep(1)
        # Close and exit moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass