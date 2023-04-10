#!/usr/bin/env python

import rospy
import logging
from geometry_msgs.msg import PoseStamped
from moveit_commander import RobotCommander, MoveGroupCommander

class StatePublisher:
    def __init__(self, debug=False, logger=None):
        self.debug = debug
        self.logger = logger
        self.arm_group = MoveGroupCommander("sagittarius_arm")
        self.robot = RobotCommander()
        self.pose_pub = rospy.Publisher("sagittarius_pose", PoseStamped, queue_size=10)
        self.rate = rospy.Rate(0.2)  # 100hz
        self.run()

    def run(self):

        while not rospy.is_shutdown():

            # Get current pose of arm and gripper
            arm_pose = self.arm_group.get_current_pose()

            # Create PoseStamped messages for arm and gripper poses
            arm_msg = PoseStamped()
            arm_msg.header.stamp = rospy.Time.now()
            arm_msg.header.frame_id = self.robot.get_planning_frame()
            arm_msg.pose = arm_pose.pose

            # Publish PoseStamped messages
            self.pose_pub.publish(arm_msg)

            # Print debugging output if debug flag is set
            if self.debug:
                logger.info("\nPOSITION:\nx: {:.3f}\ny: {:.3f}\nz: {:.3f}".format(
                    arm_pose.pose.position.x,
                    arm_pose.pose.position.y,
                    arm_pose.pose.position.z
                ) + "\n\nORIENTATION:\nx: {:.3f}\ny: {:.3f}\nz: {:.3f}\nw: {:.3f}".format(
                    arm_pose.pose.orientation.x,
                    arm_pose.pose.orientation.y,
                    arm_pose.pose.orientation.z,
                    arm_pose.pose.orientation.w
                ) + "\n")
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('state_publisher')

    # Get the value of the "debug" parameter from the launch file
    debug = rospy.get_param("~debug")
    logger = logging.getLogger('state_publisher')

    if debug:
        logger.setLevel(logging.DEBUG)

        # Add a console handler to send logs to the console
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)

        # Set the format of the logs
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)

        # Add the console handler to the logger object
        logger.addHandler(ch)

    rospy.sleep(2)
    StatePublisher(debug=debug, logger=logger)