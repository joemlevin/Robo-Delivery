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
    p.pose.position.x = 0.69
    p.pose.position.y = -.32
    p.pose.position.z = -.53
    print("Adding box to scene")
    scene.attach_box("base","table", p, (.25, .42, 0.75))
    p2 = PoseStamped()
    p2.pose.position.x = -0.55
    p2.pose.position.y = 0.91
    p2.pose.position.z = 0.432
    scene.attach_box("base", "wall1", p2, (1.6, .01, 5))
    print("Attaching first wall")
    p3 = PoseStamped()
    p3.pose.position.x = -1.02
    p3.pose.position.y = 0.2
    p3.pose.position.z = 0.15
    print("Attaching second wall")
    scene.attach_box("base", "wall2", p3, (0.01, 1.6, 5))
    rospy.sleep(2)
    objects = scene.get_objects(["table"])
    print("Scene constructed")
if __name__ == '__main__':
    main()