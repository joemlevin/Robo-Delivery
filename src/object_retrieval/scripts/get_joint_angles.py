#!/usr/bin/env python
import sys
import rospy
import numpy as np
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import tf
import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION


def get_joint_angles():
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    limb = intera_interface.Limb(valid_limbs[0])
    joint_angles = limb.joint_angles()
    return joint_angles

def main():
    rospy.init_node('get_joint_angles')
    joint_angles = get_joint_angles()
    print(joint_angles)
    rospy.signal_shutdown()
if __name__ == '__main__':
    main()