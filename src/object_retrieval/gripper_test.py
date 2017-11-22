#!/usr/bin/env python
import rospy
# from baxter_interface import gripper as robot_gripper
from intera_interface import gripper as robot_gripper

rospy.init_node('gripper_test')

#Set up the right gripper
right_gripper = robot_gripper.Gripper('right')

#Calibrate the gripper (other commands won't work unless you do this first)
print('Calibrating...')
right_gripper.calibrate()
rospy.sleep(2.0)

#Open the right gripper
print('Opening...')
right_gripper.open()
rospy.sleep(5.0)
print('Done!')

#Close the right gripper
print('Closing...')
right_gripper.close()
rospy.sleep(1.0)

