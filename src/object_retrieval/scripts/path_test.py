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

DEPTH_CORRECTION = .9

CUBE_ID_POSE = (0.569, -0.376, 0.105, -0.626, -0.362, 0.649, -0.238)

def plan_motion(commander, goal_pose):
    goal = PoseStamped()
    goal.header.frame_id = "base"
    goal.pose.position.x = goal_pose[0]
    goal.pose.position.y = goal_pose[1]
    goal.pose.position.z = goal_pose[2]
    goal.pose.orientation.x = goal_pose[3]
    goal.pose.orientation.y = goal_pose[4]
    goal.pose.orientation.z = goal_pose[5]
    goal.pose.orientation.w = goal_pose[6]
    commander.set_pose_target(goal)

    #Set the start state for the right arm
    commander.set_start_state_to_current_state()
    
    #Plan a path
    plan = commander.plan()
    #Return plan
    return plan

def plan_motion_constrained(commander, goal_pose):
    #Define Pose
    goal = PoseStamped()
    goal.header.frame_id = "base"
    goal.pose.position.x = goal_pose[0]
    goal.pose.position.y = goal_pose[1]
    goal.pose.position.z = goal_pose[2]
    goal.pose.orientation.x = goal_pose[3]
    goal.pose.orientation.y = goal_pose[4]
    goal.pose.orientation.z = goal_pose[5]
    goal.pose.orientation.w = goal_pose[6]
    commander.set_pose_target(goal)
    commander.set_start_state_to_current_state()
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;
    consts = Constraints()
    consts.orientation_constraints = [orien_const]
    right_arm.set_path_constraints(consts)
    #Plan a path
    plan = commander.plan()
    #Return plan
    return plan

# def get_joint_angles():
#     rp = intera_interface.RobotParams()
#     valid_limbs = rp.get_limb_names()
#     limb = intera_interface.Limb(valid_limbs[0])
#     joint_angles = limb.joint_angles()
#     return joint_angles

def set_initial_pose():
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    limb = intera_interface.Limb(valid_limbs[0])
    pose = {'right_j6': -4.71413671875, 'right_j5': -2.973470703125, 
        'right_j4': -2.5042646484375, 'right_j3': -2.0749814453125, 
        'right_j2': -2.24883984375, 'right_j1': 0.6924912109375, 'right_j0': -1.5153515625}
    success = False
    while not success and not rospy.is_shutdown():
        limb.set_joint_positions(pose)
        joint_angles = limb.joint_angles()
        joint_progess = []
        for key in pose.keys():
            joint_progess.append(np.isclose(joint_angles[key], pose[key], atol=1e-2))
        success = np.all(joint_progess)
        print(success)

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

    #Initialize goals
    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"
    # print(get_joint_angles())
    #x, y, and z position
    while not rospy.is_shutdown():
        success = False
        # Move arm to cube detection area
        raw_input('Press <Enter> to go to cube identification area')
        set_initial_pose()
        # Attempt to move arm above AR Cube with gripper pointed down
        while not success and not rospy.is_shutdown():
            print("Place cube within view of hand camera")
            raw_input("Press <Enter> to plan path to grasp cube")
            try:
                (trans_ar_to_base, rot_ar_to_base) = listener.lookupTransform('/base', '/ar_marker_0', rospy.Time(0))
                (trans_head_to_base, rot_head_to_base) = listener.lookupTransform('/base', '/head_camera', rospy.Time(0))
                success = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        trans_corrected = [DEPTH_CORRECTION * x + (1 - DEPTH_CORRECTION) * y for x, y in zip(trans_ar_to_base, trans_head_to_base)]
        cube_pose = trans_ar_to_base + [0, -1.0, 0, 0]
        cube_pose[2] += .2
        plan = plan_motion(right_arm, cube_pose)
        if plan.joint_trajectory.points:
            #Execute the plan
            raw_input('Press <Enter> to move the right arm to goal pose 1 (path constraints are never enforced during this motion): ')
            right_arm.execute(plan)
        else:
            print("Planning failed")
        rospy.sleep(2)
        # Attempt to lower gripper over Cube for pick up
        # TODO Find better way of doing this
        cube_pose[2] -= .2
        plan = plan_motion_constrained(right_arm, cube_pose)
        if plan.joint_trajectory.points:
            #Execute the plan
            raw_input('Press <Enter> to lower arm over cube')
            right_arm.execute(plan)
        else:
            print("Planning failed")

if __name__ == '__main__':
    main()
