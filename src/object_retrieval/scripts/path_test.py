#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import tf

DEPTH_CORRECTION = .9

def main():
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')
    # Initialize tf listener
    listener = tf.TransformListener()
    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
#    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
#    left_arm.set_planner_id('RRTConnectkConfigDefault')
#    left_arm.set_planning_time(10)
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)

    #First goal pose ------------------------------------------------------
    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"

    #x, y, and z position
    while not rospy.is_shutdown():
        success = False
        raw_input('Press <Enter> to attempt to plan')
        while not success:
            try:
                (trans_ar_to_base, rot_ar_to_base) = listener.lookupTransform('/base', '/ar_marker_0', rospy.Time(0))
                (trans_head_to_base, rot_head_to_base) = listener.lookupTransform('/base', '/head_camera', rospy.Time(0))
                success = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        trans_corrected = [DEPTH_CORRECTION * x + (1 - DEPTH_CORRECTION) * y for x, y in zip(trans_ar_to_base, trans_head_to_base)]
        print(trans_ar_to_base)
        print(trans_corrected)
        gripper_pos = trans_corrected
        goal_1.pose.position.x = gripper_pos[0]
        goal_1.pose.position.y = gripper_pos[1]
        goal_1.pose.position.z = gripper_pos[2]
        
        #Orientation as a quaternion
        goal_1.pose.orientation.x = 0.0
        goal_1.pose.orientation.y = -1.0
        goal_1.pose.orientation.z = 0.0
        goal_1.pose.orientation.w = 0.0

        #Set the goal state to the pose you just defined
        right_arm.set_pose_target(goal_1)

        #Set the start state for the right arm
        right_arm.set_start_state_to_current_state()
        
        #Plan a path
        right_plan = right_arm.plan()

        #Execute the plan
        raw_input('Press <Enter> to move the right arm to goal pose 1 (path constraints are never enforced during this motion): ')
        right_arm.execute(right_plan)

if __name__ == '__main__':
    main()
