#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import tf
import intera_interface 
from intera_interface import CHECK_VERSION

def main():
    try:
        rospy.init_node("head_turner")
        # head_turner = HeadTurner()
        # #rospy.on_shutdown(head_turner.clean_shutdown)
        # head_turner.set_neutral()
        # head_turner.turn_head(.4)
        command_rate = rospy.Rate(1)
        control_rate = rospy.Rate(100)
        head = intera_interface.Head()
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        print("Enabling robot... ")
        rs.enable()
        angle = 0.95
        while not rospy.is_shutdown(): #and (not (abs(head.pan() - angle) <= 
                # intera_interface.HEAD_PAN_ANGLE_TOLERANCE)):
            head.set_pan(angle, speed=0.3, timeout=0)
            print("Current head angle = {}".format(head.pan()))
            print("Desired head angle = {}".format(angle))
            control_rate.sleep()
        command_rate.sleep()
        #after_turn = head_turner.get_head_pan()
        #print("Angle after turn = {}".format(after_turn))
        #rospy.signal_shutdown("Head finished turning")
    except:
        print("bad things")
if __name__ == "__main__":
    main()