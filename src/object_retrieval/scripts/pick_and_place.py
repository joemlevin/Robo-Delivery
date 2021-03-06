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

# Define constants
DEPTH_CORRECTION = .9
GRIPPER_OFFSET = 0.2 # offset to prevent gripper collision with cube
PICK_OFFSET = 0.09 # offset to bring cube into reach of gripper
DROP_OFFSET = 0.19 # drop off offset
INITIAL_POSITION = {'right_j6': 1.6959248046875, 'right_j5': 2.5555615234375, 
'right_j4': -0.101990234375, 'right_j3': -1.5755078125, 
'right_j2': -0.002775390625, 'right_j1': 0.4079228515625, 'right_j0': 0.2727294921875}

CUBE_ID_POSE = {'right_j6': 0.5001396484375, 'right_j5': 1.249232421875, 
'right_j4': -0.5017265625, 'right_j3': -1.939955078125, 
'right_j2': -0.083318359375, 'right_j1': 0.63296484375, 'right_j0': -0.5384716796875}

TURTLEBOT_ID_POSE = {'right_j6': 1.8170166015625, 
'right_j5': -0.1306240234375, 'right_j4': -0.0239052734375, 
'right_j3': -1.07894921875, 'right_j2': 0.093466796875, 
'right_j1': 1.1522158203125, 'right_j0': 0.34566015625}
### UNCOMMENT IN CASE OF EMERGENCIES
# CUBE_ID_POSE = {'right_j6': 0.0456103515625, 'right_j5': 1.0447099609375, 
#     'right_j4': -0.6278916015625, 'right_j3': -2.2208623046875, 
# 'right_j2': 0.1072822265625, 'right_j1': 1.12991015625, 'right_j0': -0.6118203125}

CLEAR_OF_TABLE_POSITION = (0.593, 0.000, 0.322, 0.0, -1.0, 0.0, 0.0)
CUBE_DROP_OFF_POSITION = (0.709, 0.228, -0.235, 0.0, -1.0, 0.0, 0.0)

def define_scene(robot, scene):
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
    commander.clear_path_constraints()
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

def find_tag(listener, marker="/ar_marker_0"):
    cube_detected = False
    success = False
    while not (cube_detected and success) and not rospy.is_shutdown():
        print("Attempting to find pick up location")
        try:
            if not cube_detected:
                (cube_pos, cube_orientation) = listener.lookupTransform('/base', marker, rospy.Time(0))
                cube_detected = True
                print("Cube Detected!")
                rospy.sleep(3.0)
            else:
                rospy.sleep(3.0)
                (cube_pos, cube_orientation) = listener.lookupTransform('/base', marker, rospy.Time(0))
                success = True
                print("Acquired cube position")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("AR Cube not detected.")
            cube_detected = False
            success = False
    return cube_pos        

def pick_up(robot, scene, right_arm, gripper, listener):
    # Instantiate flags for different phases
    print("Beginning turtlebot cube pick up")
    at_pick_up = False
    lowered = False
    # Move arm to initial position
    print("Moving to initial position")
    rospy.sleep(0.5)
    set_pose(INITIAL_POSITION)

    # Move arm to turtlebot detection area
    print("Moving to turtlebot detection position")
    rospy.sleep(0.5)
    set_pose(TURTLEBOT_ID_POSE)
    # sys.exit()
    # Locate cube
    cube_pos = find_tag(listener, '/ar_marker_10')
    # Add in downward facing orientation to position of AR Cube
    cube_pose = cube_pos + [0, -1.0, 0, 0]
    # Add in z-axis offset to account for gripper length
    cube_pose[2] += GRIPPER_OFFSET
    while not at_pick_up and not rospy.is_shutdown():
        print("Attempting to move to pick up location")
        rospy.sleep(0.5)
        plan = plan_motion(right_arm, cube_pose)
        at_pick_up = execute_motion(right_arm, plan)
    gripper.open()
    print("Success")
    rospy.sleep(2)
    # Attempt to lower gripper over Cube for pick up
    cube_pose[2] -= PICK_OFFSET

    plan = plan_motion_constrained(right_arm, cube_pose)
    while not lowered and not rospy.is_shutdown():
        print("Attempting to lower onto cube")
        plan = plan_motion_constrained(right_arm, cube_pose)
        if plan.joint_trajectory.points:
            print("plan found. Executing")
            #Execute the plan
            rospy.sleep(0.5)
            right_arm.execute(plan)
            rospy.sleep(0.2)
            gripper.close()
            lowered = True
        else:
            print("Planning failed")
            lowered = False
    # Bring gripper back up
    raised = False
    cube_pose[2] += PICK_OFFSET
    while not raised and not rospy.is_shutdown():
        print("Bring arm back up")
        plan = plan_motion(right_arm, cube_pose)
        if plan.joint_trajectory.points:
            print("Plan found. Executing")
            rospy.sleep(0.5)
            right_arm.execute(plan)
            raised = True
        else:
            print("Planning failed")
            raised = False
    # Move back to initial position
    print("Moving back to initial position")
    rospy.sleep(0.5)
    set_pose(INITIAL_POSITION)
    # Move arm to drop off detection position
    print("Moving to determine drop off location")
    rospy.sleep(0.5)
    set_pose(CUBE_ID_POSE)
    # Find drop off location
    drop_off_pos = find_tag(listener, '/ar_marker_11')
    drop_off_pose = drop_off_pos + [0.0, -1.0, 0.0, 0.0]
    drop_off_pose[2] += DROP_OFFSET + 0.05

    # Drop off cube
    at_drop_off = False
    while not at_drop_off and not rospy.is_shutdown():
        print("Dropping off cube")
        rospy.sleep(0.5)
        plan = plan_motion(right_arm, drop_off_pose)
        at_drop_off = execute_motion(right_arm, plan)
        rospy.sleep(0.2)

    drop_off_pose[2] -= 0.05
    at_drop_off = False

    while not at_drop_off and not rospy.is_shutdown():
        print("Lowering to drop off")
        rospy.sleep(0.5)
        plan = plan_motion_constrained(right_arm, drop_off_pose)
        at_drop_off = execute_motion(right_arm, plan)
        rospy.sleep(0.2)
    gripper.open()

    drop_off_pose[2] += 0.1

    at_drop_off = False
    while not at_drop_off and not rospy.is_shutdown():
        print("Raising to clear cube")
        rospy.sleep(0.5)
        plan = plan_motion_constrained(right_arm, drop_off_pose)
        at_drop_off = execute_motion(right_arm, plan)
        rospy.sleep(0.2)

    print("Pick up of delivered cube complete.")

def drop_off(robot, scene, right_arm, gripper, listener):
    # Instantiate flags for different phases
    print("Beginning drop off")

    cube_detected = False
    success = False
    at_pick_up = False
    cleared_table = False
    at_drop_off = False

    # Move arm to initial position
    print("Moving to initial position")
    rospy.sleep(0.5)
    set_pose(INITIAL_POSITION)

    # Move arm to cube detection area
    print("Moving to cube ID position")
    rospy.sleep(0.5)
    set_pose(CUBE_ID_POSE)

    ### This is here for debugging issues with cube detection
    # sys.exit()

    # Locate cube
    cube_pos = find_tag(listener)
    # Add in downward facing orientation to position of AR Cube
    cube_pose = cube_pos + [0, -1.0, 0, 0]
    # Add in z-axis offset to account for gripper length
    cube_pose[2] += GRIPPER_OFFSET
    while not at_pick_up and not rospy.is_shutdown():
        print("Attempting to move to pick up location")
        rospy.sleep(0.5)
        plan = plan_motion(right_arm, cube_pose)
        at_pick_up = execute_motion(right_arm, plan)
    gripper.open()
    rospy.sleep(2)

    # Attempt to lower gripper over Cube for pick up
    cube_pose[2] -= PICK_OFFSET

    plan = plan_motion_constrained(right_arm, cube_pose)
    if plan.joint_trajectory.points:
        #Execute the plan
        print("Attempting to lower onto cube")
        rospy.sleep(0.5)
        right_arm.execute(plan)
        gripper.close()
    else:
        print("Planning failed")

    # Raise arm back to clear cube of table
    cube_pose[2] += PICK_OFFSET
    plan = plan_motion_constrained(right_arm, cube_pose)
    if plan.joint_trajectory.points:
        print("Attempting to raise arm up")
        rospy.sleep(0.5)
        right_arm.execute(plan)
    else:
        print("Planning failed")

    # Move arm to drop off location
    while not cleared_table and not rospy.is_shutdown():
        print("Attempting to move clear of table")
        rospy.sleep(0.5)
        plan = plan_motion(right_arm, CLEAR_OF_TABLE_POSITION)
        cleared_table = execute_motion(right_arm, plan)

    rospy.sleep(0.5)
    set_pose(INITIAL_POSITION)

    rospy.sleep(0.5)
    set_pose(TURTLEBOT_ID_POSE)

    drop_off_pose = find_tag(listener, '/ar_marker_9') + [0, -1.0, 0, 0]
    rospy.sleep(0.5)
    set_pose(INITIAL_POSITION)
    drop_off_pose[2] += GRIPPER_OFFSET
    while not at_drop_off and not rospy.is_shutdown():
        print("Attempting to move to drop off")
        rospy.sleep(0.5)
        plan = plan_motion(right_arm, drop_off_pose)
        at_drop_off = execute_motion(right_arm, plan)
    rospy.sleep(1.0)
    gripper.open()

def main():
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start moveit_node
    rospy.init_node('moveit_node')

    # Initialize tf listener
    listener = tf.TransformListener()

    #Initialize arm and scene
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)

    # Initialize gripper
    gripper = intera_interface.gripper.Gripper('right')
    gripper.calibrate()
    rospy.sleep(0.5)

    # Construct scene for obstacle avoidance
    define_scene(robot, scene)
    # Loop and perform pick and and drop off procedures
    while not rospy.is_shutdown():
        # Initiate pick up procedure
        pick_up(robot, scene, right_arm, gripper, listener)

        # Initiate drop off procedure
        drop_off(robot, scene, right_arm, gripper, listener)

if __name__ == '__main__':
    main()
