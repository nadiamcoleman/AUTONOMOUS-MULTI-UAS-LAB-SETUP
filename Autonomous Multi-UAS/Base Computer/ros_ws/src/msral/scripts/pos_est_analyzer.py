#!/usr/bin/env python

'''
    pos_est_analyzer.py
    
    Written by Daniel McArthur
    Date: March 25, 2019
    
    Subscribes to position estimation topics (Vicon and PX4:EKF2) and 
    calculates the error in the estimated PX4:EKF2 position/orientation.
    
    Also subscribes to estimated clicked target position and tracks the 
    error in that position estimation.
'''

import rospy
import signal
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ActuatorControl
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import readline
import time
import threading as th


class PosEstAnalyzer( object ):
    def __init__(self):
        pass
    
        # ------------------ Initialize ROS ---------------------
        rospy.init_node('pos_est_analyzer', anonymous=True)
        rospy.loginfo("Subscribing to:\n  '/mavros/setpoint_position/local'" \
                    + ", '/mavros/actuator_control'")

        # Initialize publisher for mavros setpoint (required for OFFBOARD mode)
        pub_pose_sp = rospy.Publisher('/mavros/setpoint_position/local', \
                                    PoseStamped, queue_size=10)
        # Initialize publisher for actuator_control (to drive Boom-Prop)
        pub_act_control = rospy.Publisher('/mavros/actuator_control', \
                                        ActuatorControl, queue_size=10)

        # ------------------ Subscribe to data streams ---------------------
        rospy.Subscriber('mavros/state', State, state_cb)

# Get the MAVROS state
def state_cb(msg):
  global cur_state
  cur_state = msg

def setpoint_thread_cb(thread_name):
  global pub_pose_sp, pose_sp
  rate = rospy.Rate(20)
  while not rospy.is_shutdown():
    pub_pose_sp.publish(pose_sp)
    rate.sleep()

# Main Publishing Function
def run():
  global cur_state, pub_act_control, pub_pose_sp, pose_sp
  
  # Initialize ROS
  rospy.init_node('offboard_controller', anonymous=True)
  rospy.loginfo("Publishing:\n  '/mavros/setpoint_position/local'" \
                + ", '/mavros/actuator_control'")
  
  # Initialize publisher for mavros setpoint (required for OFFBOARD mode)
  pub_pose_sp = rospy.Publisher('/mavros/setpoint_position/local', \
                                PoseStamped, queue_size=10)
  # Initialize publisher for actuator_control (to drive Boom-Prop)
  pub_act_control = rospy.Publisher('/mavros/actuator_control', \
                                    ActuatorControl, queue_size=10)
  
  # ------------------ Subscribe to data streams ---------------------
  rospy.Subscriber('mavros/state', State, state_cb)
  
  #t = th.Thread(setpoint_thread_cb, 'thread-name')
  #t.start()
  # Setpoint publishing rate MUST be faster than 2Hz
  rate = rospy.Rate(20.0)

  # Wait for FCU connection
  while(not rospy.is_shutdown() and (cur_state is None or not cur_state.connected)):
    rospy.logwarn('Waiting for FCU connection...')
    rate.sleep()

  # Set up MAVROS services (for arming and setting flight mode)
  rospy.wait_for_service('mavros/cmd/arming')
  rospy.wait_for_service('mavros/set_mode')
  rospy.wait_for_service('mavros/cmd/takeoff')
  rospy.wait_for_service('mavros/cmd/land')
  try:
    arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    setMode = rospy.ServiceProxy('mavros/set_mode', SetMode)
  except rospy.ServiceException, e:
    print "Service call failed: %s" % e
  
  
  # Prepare service calls
  #arm_cmd = CommandBool()
  #arm_cmd.request.value = True
  #mode_cmd = SetMode()
  #mode_cmd
  # Prepare dummy pose sp
  
  
  #send a few setpoints before starting
  #pList = 10*[PoseStamped()]
  pose_sp = PoseStamped()
  pose_sp.pose.position.x = 5
  pose_sp.pose.position.y = 2
  pose_sp.pose.position.z = 2

  actuator_sp = ActuatorControl()
  actuator_sp.controls[5] = 0.5
  actuator_sp.group_mix = ActuatorControl.PX4_MIX_MANUAL_PASSTHROUGH

  i = 50
  while (not rospy.is_shutdown()) and i > 0:
    pub_pose_sp.publish(pose_sp);
    i = i - 1
    rate.sleep()
  
  # Loop until ROS Shutdown
  last_request = time.time()
  while not rospy.is_shutdown():
    #rospy.loginfo('state: ' + cur_state.mode)
    if( cur_state.mode != "OFFBOARD" and (time.time() - last_request > 1)):
      rospy.loginfo('attempting OFFBOARD')
      success = setMode(custom_mode="OFFBOARD")
      if( success and True ):# offb_set_mode.response.mode_sent){
        rospy.loginfo("Offboard enabled")
      last_request = time.time()
    else:
      if( not cur_state.armed and (time.time()- last_request > 1)):
        rospy.loginfo('attempting ARM')
        success = arm(True)
        if( success and True ): #arm_cmd.response.success){
          rospy.loginfo("Vehicle armed")

        last_request = time.time()

    
    pub_pose_sp.publish(pose_sp)
    #actuator_sp.controls[5] = 0.5
    #pub_act_control.publish(actuator_sp)
    rate.sleep()
    
    #try:
    #  userIN = raw_input("Type a gripper control command: \n"
    #                      + "  g<num>\n")
    #  pub_grip_cont.publish(userIN) 
    #except KeyboardInterrupt:
    #  break
    
  rospy.loginfo("offboard_controller exiting...")

  
if __name__ == '__main__':
    run()
