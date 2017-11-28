#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import tf

def main():
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    #Start a node
    print("Initializing node")
    rospy.init_node('moveit_node')
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    rospy.sleep(2)

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 1.2
    p.pose.position.y = 0.0
    p.pose.position.z = 0.1
    print("Adding box to scene")
    scene.add_box("table", p, (0.5, 1.5, 0.6))
    rospy.sleep(2)
    objects = scene.get_objects(["table"])
    print(objects)
    print("Spinning...")
    rospy.spin()
if __name__ == '__main__':
    main()