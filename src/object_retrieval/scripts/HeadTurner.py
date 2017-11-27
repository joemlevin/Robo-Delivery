#!/usr/bin/env python
import argparse
import random
import rospy 
import intera_interface 
from intera_interface import CHECK_VERSION

class HeadTurner(object):
    
    def __init__(self):

        self._orientation = 0.0
        self._head = intera_interface.Head()

        # verify robot is enabled
        print("initializing head turner")
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def clean_shutdown(self):
        """
       Exits example cleanly by moving head to neutral position and
       maintaining start state
       """
        print("\nExiting example...")
        #self.set_neutral()

    def set_neutral(self):
        """
        Sets the head back to its neutral pose
        """
        self._head.set_pan(0.0)

    def turn_head(self, angle):
        """
        Turns the head to the specified angle
        """
        command_rate = rospy.Rate(1)
        control_rate = rospy.Rate(100)
        while not rospy.is_shutdown() and (not (abs(self._head.pan() - angle) <= 
                intera_interface.HEAD_PAN_ANGLE_TOLERANCE)):
            self._head.set_pan(angle, speed=0.3, timeout=0)
            control_rate.sleep()
        command_rate.sleep()

    def get_head_pan(self):
        return self._head.pan()
        