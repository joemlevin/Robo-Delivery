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

current_orientation = Quaternion(0,0,0,1)

# Important Information: 
# As long as you and the turtlebot is using the same "Master" 
# -- by inputting the following into terminal or .bashrc file 
# "export ROS_MASTER_URI=http://[TurtleID].local.11311 -- 
# then you can talk to the topics used in the turtlebot's machine. 

# Called when subscriber receives an imu_message
def imu_callback(imu_message):
  # Get the current orientation and set it in the Global variable.
  # The mover will need this variable to always be updated with the
  # latest orientation data.
  global current_orientation 
  current_orientation = imu_message.orientation

def move_to_goal(move_base, linear_pos):
  print("Starting to make a goal...")
  ### Make the goal ###
  ar_goal = MoveBaseGoal()
  ar_goal.target_pose.header.frame_id = '/map'
  ar_goal.target_pose.header.stamp = rospy.Time.now()
  # Create the pose
  ar_goal.target_pose.pose.position.x = linear_pos[0] - 0.1
  ar_goal.target_pose.pose.position.y = linear_pos[1] + 1.2
  print("This is my x %r" % linear_pos[0])
  print("This is my y %r" % linear_pos[1])
  ar_goal.target_pose.pose.orientation.w = 0.5
  # target.pose.orientation = current_orientation
  
  # Send our goal to client
  move_base.send_goal(ar_goal)

  # Allow Turtlebot up to 60 secs to complete task
  success = move_base.wait_for_result(rospy.Duration(60))
  if not success:
    move_base.cancel_goal()
    rospy.loginfo("The base failed to move forward to the tag for some reason.")
  else:
    print('It should have been successful')
    state = move_base.get_state()
    return state

#Define the method which contains the main functionality of the node.
def mover():
  curr_seq = 0 # For the header
  goal_made = False

  #Run this program as a new node in the ROS computation graph 
  #called /talker.
  rospy.init_node('mover', anonymous=True)

  # #Create an instance of the rospy.Publisher object which we can 
  # #use to publish messages to a topic. This publisher publishes 
  # #messages of type geometry_msgs/Twist to the topic /cmd_vel_mux/input/teleop
  # pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

  # Tell the action client that we want to spin a thread by default
  move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  rospy.loginfo("wait for the action server to come up")
  # Allow up to 5 secs for the action server to come up
  move_base.wait_for_server(rospy.Duration(10))
  
  #Create a listener to listen to position of detected AR Tag.
  listener = tf.TransformListener()

  #Create a listener to know the robot's current position
  data_listener = rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, imu_callback)

  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz  

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    # Get the trans and rotational positions of the AR Tag
    try: 
      print("Transforming...")
      (trans, rotation) = listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
      print("I have an exception")
      print(e)
      continue
    else:
      state = move_to_goal(move_base, trans)
      if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Made it to the goal!")
        break
      elif state == GoalStatus.ABORTED:
        rospy.loginfo('The goal was aborted during execution by the action server due to some failure')
        break
      elif state == GoalStatus.REJECTED:
        rospy.loginfo("The goal was rejected.")
        break
      elif state == GoalStatus.PREEMPTING:
        rospy.loginfo("The goal received a cancel request after exectution.")
        break
      elif state == GoalStatus.PENDING:
        rospy.loginfo("The goal has yet to be processed by the action server.")
        move_to_goal(move_base, trans)
      break

    
    # Use our rate object to sleep until it is time to publish again
    r.sleep()
      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the mover method
  try:
    mover()
  except rospy.ROSInterruptException: pass                                                                