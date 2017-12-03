#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf
import math
import time
import actionlib

from enum import Enum

#Import the String message type from the /msg directory of
#the std_msgs package.
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Time
from geometry_msgs.msg import *
from sensor_msgs.msg import Imu
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import *

class Stage(Enum):
  DELIVER = 1
  WAIT = 2
  RETURN = 3

class Status(Enum):
  SUCCESS = 1
  FAILED = 2

per_line_to_move = .80

# Important Information: 
# As long as you and the turtlebot is using the same "Master" 
# -- by inputting the following into terminal or .bashrc file 
# "export ROS_MASTER_URI=http://[TurtleID].local.11311 -- 
# then you can talk to the topics used in the turtlebot's machine. 

def move_to_goal(move_base, linear_pos, my_pos):
  print("Starting to make a goal...")
  
  ### Find the new coordinates ###
  x_2,y_2,_ = linear_pos
  x_1,y_1,_ = my_pos
  z = math.sqrt((x_2 - x_1)**2 + (y_2 - y_1)**2)
  m = (y_2 - y_1)/(x_2 - x_1)

  new_x = (per_line_to_move*z)/math.sqrt(1+m**2) + x_1
  new_y = (new_x - x_1)*m + y_1 

  ### Make the goal ###
  ar_goal = MoveBaseGoal()
  ar_goal.target_pose.header.frame_id = '/map'
  ar_goal.target_pose.header.stamp = rospy.Time.now()
  # Create the pose
  ar_goal.target_pose.pose.position.x = new_x 
  ar_goal.target_pose.pose.position.y = new_y
  ar_goal.target_pose.pose.orientation.w = 1 
  print("This is turtlebot position: %r" % my_pos)
  print("This is position of ar_tag: %r" % linear_pos)
  print("These are the calculations: x = %r ; y = %r" % (new_x, new_y))
  print("m=%r ; z=%r\n" % (m, z))
  
  ### Now move to the position ###
  success = False
  state = GoalStatus.PENDING
  time = rospy.Time()
  start = time.now().secs
  while((not (success and state == GoalStatus.SUCCEEDED) or (time.now().secs - start) < 10) and not rospy.is_shutdown()): 
    move_base.send_goal(ar_goal) 
    success = move_base.wait_for_result(rospy.Duration(60))
    state = move_base.get_state()

  if success and move_base.get_state() == GoalStatus.SUCCEEDED:
    return Status.SUCCESS
  return Status.FAILED

def turn(vel_pub):
  linear_motion = Vector3(0, 0, 0)
  angular_motion = Vector3(0,0,0.70)
  forward_motion = Twist(linear_motion, angular_motion)
  # Publish our velocity twist to the 'teleop' topic
  vel_pub.publish(forward_motion)
  return

# def move_up(vel_pub):
#   time = rospy.Time()
#   start = time.now().secs
#   while((time.now().secs - start < 3) and not rospy.is_shutdown()):
#     linear_motion = Vector3(.3,0,0)
#     angular_motion = Vector3(0,0,0)
#     forward_motion = Twist(linear_motion, angular_motion)
#     ######
#     # Publish our velocity twist to the 'teleop' topic
#     vel_pub.publish(forward_motion)

# Define the method which contains the main functionality of the node.
def mover():
  # Run this program as a new node in the ROS computation graph 
  # called /talker.
  rospy.init_node('mover', anonymous=True)

  status_channel = rospy.Publisher('turtlebot_status', String, queue_size=10)

  # Create an instance of the rospy.Publisher object which we can 
  # use to publish messages to a topic. This publisher publishes 
  # messages of type geometry_msgs/Twist to the topic /cmd_vel_mux/input/teleop
  vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

  # Tell the action client that we want to spin a thread by default
  move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  rospy.loginfo("wait for the action server to come up")
  # Allow up to 5 secs for the action server to come up
  move_base.wait_for_server(rospy.Duration(10))
  
  # Create a listener to listen to position of detected AR Tag.
  listener = tf.TransformListener()

  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz  

  # Set up the while loop and wait for the robot to warm up
  time.sleep(5)
  ar_tag = '/ar_marker_0'
  start_trans = [0,0,0]
  current_stage = Stage.DELIVER

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    if current_stage == Stage.DELIVER:
      ##### Deliver the object to destination #####
      # Get the trans and rotational positions of the AR Tag
      try: 
        (trans, _) = listener.lookupTransform('/map', ar_tag, rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        # If I cannot find an AR Tag, turn to search for it. 
        # Make the robot turn until it finds the AR Tag. 
        turn(vel_pub)
      else:
        # We have found an AR_Tag. Let's take move to it. #
        # Find my position
        (my_trans, _) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

        # Now move along that direction. 
        rospy.loginfo("Taking the motion for AR Tag: %r" % ar_tag)
        move_status = move_to_goal(move_base, trans, my_trans)
        if move_status != Status.SUCCESS:
          rospy.loginfo("Could not move to position.")
          break

        rospy.loginfo("We should have reached goal.")
        rospy.sleep(1) # Sleep for 1 seconds
        # Transition to the next state.
        if ar_tag == '/ar_marker_0':
          ar_tag = '/ar_marker_4'
        elif ar_tag == '/ar_marker_4':
          ar_tag = '/ar_marker_11'
        elif ar_tag == '/ar_marker_11':
          # move_up(vel_pub)
          pub_string = "Sawyer I am here! %s" % (rospy.get_time())
          status_channel.publish(pub_string)
          current_stage = Stage.WAIT
    elif current_stage == Stage.WAIT:
      ##### Wait until I get a signal from Saywer #####
      raw_input("Waiting. Press enter to continue.")
      current_stage = Stage.RETURN
    else:
      ##### Return to the starting point #####
      (my_trans, _) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
      move_status = move_to_goal(move_base, start_trans, my_trans)
      if move_status == Stage.SUCCESS:
        rospy.loginfo("Returned to start.")
      else:
        rospy.loginfo("Failed to move to start.")
      break

    # Sleep for a quick sec in order to prevent weird behaviors with the motions.
    r.sleep() 

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the mover method
  try:
    mover()
  except rospy.ROSInterruptException: pass                                                                