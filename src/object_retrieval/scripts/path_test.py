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

INITIAL_POSITION = {'right_j6': 1.6959248046875, 'right_j5': 2.5555615234375, 
'right_j4': -0.101990234375, 'right_j3': -1.5755078125, 
'right_j2': -0.002775390625, 'right_j1': 0.4079228515625, 'right_j0': 0.2727294921875}

CUBE_ID_POSE = {'right_j6': 0.0456103515625, 'right_j5': 1.0447099609375, 
    'right_j4': -0.6278916015625, 'right_j3': -2.2208623046875, 
    'right_j2': 0.1072822265625, 'right_j1': 1.12991015625, 'right_j0': -0.6118203125}

CLEAR_OF_TABLE_POSITION = (0.593, 0.000, 0.322, 0.0, -1.0, 0.0, 0.0)
CUBE_DROP_OFF_POSITION = (0.653, 0.249, -0.397, 0.0, -1.0, 0.0, 0.0)


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
    commander.set_path_constraints(consts)
    #Plan a path
    plan = commander.plan()
    #Return plan
    return plan

def execute_motion(commander, plan):
    if not plan.joint_trajectory.points:
        return False
    else:
        commander.execute(plan)
        return True
# def get_joint_angles():
#     rp = intera_interface.RobotParams()
#     valid_limbs = rp.get_limb_names()
#     limb = intera_interface.Limb(valid_limbs[0])
#     joint_angles = limb.joint_angles()
#     return joint_angles
def set_pose(pose):
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    limb = intera_interface.Limb(valid_limbs[0])
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

    # Initialize gripper
    gripper = intera_interface.gripper.Gripper('right')
    gripper.calibrate()
    rospy.sleep(2.0)

    #Initialize goals
    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"
    # print(get_joint_angles())
    #x, y, and z position
    #while not rospy.is_shutdown():
    success = False
    at_pick_up = False
    cleared_table = False
    at_drop_off = False
    # Move arm to initial position
    print("Moving to initial position")
    rospy.sleep(2.0)
    set_pose(INITIAL_POSITION)
    # Move arm to cube detection area
    print("Moving to cube ID position")
    rospy.sleep(2.0)
    set_pose(CUBE_ID_POSE)

    # Attempt to move arm above AR Cube with gripper pointed down
    while not success and not rospy.is_shutdown():
        rospy.sleep(2.0)
        print("Attempting to find pick up location")
        try:
            (trans_ar_to_base, rot_ar_to_base) = listener.lookupTransform('/base', '/ar_marker_0', rospy.Time(0))
            (trans_head_to_base, rot_head_to_base) = listener.lookupTransform('/base', '/head_camera', rospy.Time(0))
            success = True
            print("Success!")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("AR Cube not detected.")
            success = False
    cube_pose = trans_ar_to_base + [0, -1.0, 0, 0]
    # Add in z-axis offset to account for gripper length
    cube_pose[2] += .2
    while not at_pick_up and not rospy.is_shutdown():
        print("Attempting to move to pick up location")
        rospy.sleep(2.0)
        plan = plan_motion(right_arm, cube_pose)
        at_pick_up = execute_motion(right_arm, plan)
    rospy.sleep(2)

    # Attempt to lower gripper over Cube for pick up
    cube_pose[2] -= 0.09

    plan = plan_motion_constrained(right_arm, cube_pose)
    if plan.joint_trajectory.points:
        #Execute the plan
        print("Attempting to lower onto cube")
        rospy.sleep(2.0)
        right_arm.execute(plan)
        gripper.close()
    else:
        print("Planning failed")

    # Raise arm back to clear cube of table
    cube_pose[2] += 0.09
    plan = plan_motion_constrained(right_arm, cube_pose)
    if plan.joint_trajectory.points:
        print("Attempting to raise arm up")
        rospy.sleep(2.0)
        right_arm.execute(plan)
    else:
        print("Planning failed")

    # Move arm to drop off location
    while not cleared_table and not rospy.is_shutdown():
        print("Attempting to move clear of table")
        rospy.sleep(2.0)
        plan = plan_motion(right_arm, CLEAR_OF_TABLE_POSITION)
        cleared_table = execute_motion(right_arm, plan)

    while not at_drop_off and not rospy.is_shutdown():
        print("Attempting to move to drop off")
        rospy.sleep(2.0)
        plan = plan_motion(right_arm, CUBE_DROP_OFF_POSITION)
        at_drop_off = execute_motion(right_arm, plan)
    gripper.open()


if __name__ == '__main__':
    main()
