#!/usr/bin/env python

# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

import time

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('/intel1/mavros/cmd/arming')
        rospy.wait_for_service('/uvify1/mavros/cmd/arming')
        try:
            armService1 = rospy.ServiceProxy('/intel1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService2 = rospy.ServiceProxy('/uvify1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            response1 = armService1(True)
            response2 = armService2(True)
            return response1.success, response2.success
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e
            return False, False

    def setDisarm(self):
        rospy.wait_for_service('/intel1/mavros/cmd/arming')
        rospy.wait_for_service('/uvify1/mavros/cmd/arming')
        try:
            armService1 = rospy.ServiceProxy('/intel1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService2 = rospy.ServiceProxy('/uvify1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            response1 = armService1(False)
            response2 = armService2(False)
            return response1.success, response2.success
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e
            return False, False
    rospy.loginfo('Streaming home_sp at 20 Hz...')

    def setStabilizedMode(self):
        rospy.wait_for_service('/intel1/mavros/set_mode')
        rospy.wait_for_service('/uvify1/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy('/intel1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2 = rospy.ServiceProxy('/uvify1/mavros/set_mode', mavros_msgs.srv.SetMode) 
            response1 = flightModeService1(custom_mode='STABILIZED')
            response2 = flightModeService2(custom_mode='STABILIZED')
            return response1.mode_sent, response2.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e
            return False, False

    def setOffboardMode(self):
        rospy.wait_for_service('/intel1/mavros/set_mode')
        rospy.wait_for_service('/uvify1/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy('/intel1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2 = rospy.ServiceProxy('/uvify1/mavros/set_mode', mavros_msgs.srv.SetMode)
            response1 = flightModeService1(custom_mode='OFFBOARD')
            response2 = flightModeService2(custom_mode='OFFBOARD')
            return response1.mode_sent, response2.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e
            return False, False

    def setAltitudeMode(self):
        rospy.wait_for_service('/intel1/mavros/set_mode')
        rospy.wait_for_service('/uvify1/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy('/intel1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2 = rospy.ServiceProxy('/uvify1/mavros/set_mode', mavros_msgs.srv.SetMode)
            response1 = flightModeService1(custom_mode='ALTCTL')
            response2 = flightModeService2(custom_mode='ALTCTL')
            return response1.mode_sent, response2.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e
            return False, False, False

    def setPositionMode(self):
        rospy.wait_for_service('/intel1/mavros/set_mode')
        rospy.wait_for_service('/uvify1/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy('/intel1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2 = rospy.ServiceProxy('/uvify1/mavros/set_mode', mavros_msgs.srv.SetMode)
            response1 = flightModeService1(custom_mode='POSCTL')
            response2 = flightModeService2(custom_mode='POSCTL')
            return response1.mode_sent, response2.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e
            return False, False, False

    def setAutoLandMode(self):
        rospy.wait_for_service('/intel1/mavros/set_mode')
        rospy.wait_for_service('/uvify1/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy('/intel1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2 = rospy.ServiceProxy('/uvify1/mavros/set_mode', mavros_msgs.srv.SetMode)
            response1 = flightModeService1(custom_mode='AUTO.LAND')
            response2 = flightModeService2(custom_mode='AUTO.LAND')
            return response1.mode_sent, response2.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Autoland Mode could not be set."%e
            return False, False, False

# Main class: Converts joystick commands to position setpoints
class Controller:
    # initialization method
    def __init__(self):
      
        # Drone state
        self.state = State()
        
        # Altitude setpoint, [meters]
        self.ALT_SP1        = 1
        self.ALT_SP2        = 1
        
        
        # Instantiate setpoint messages
        self.sp1         = PoseStamped()
        self.sp2         = PoseStamped()
        self.home_sp1    = PoseStamped()
        self.home_sp2    = PoseStamped()      
        self.home_sp1.pose.position.x = 0
        self.home_sp1.pose.position.y = -1
        self.home_sp1.pose.position.z = self.ALT_SP1
        self.home_sp2.pose.position.x = 0
        self.home_sp2.pose.position.y = 1
        self.home_sp2.pose.position.z = self.ALT_SP2
        
        self.sp1.pose.position.x = 0
        self.sp1.pose.position.y = -1
        self.sp1.pose.position.z = self.ALT_SP1
        self.sp2.pose.position.x = 0
        self.sp2.pose.position.y = 1
        self.sp2.pose.position.z = self.ALT_SP2

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0.0)
        
        # Handler to set FCU modes
        self.modes = fcuModes()

        
        # Create waypoints for path traversal
        self.wp_pose1 = PoseStamped()
        self.wp_pose2 = PoseStamped()
        self.wp_pose1.pose.position.z = self.ALT_SP1
        self.wp_pose2.pose.position.z = self.ALT_SP2
        
        
        self.p1_waypoints = []
        self.p2_waypoints = []
        
        self.p1_waypoints.append([0,-1])
        self.p2_waypoints.append([0, 1])
        
        # Waypoint control variables
        self.wp_started  = False
        self.wp_finished = False
        self.wp_timer = 0
        self.wp_index = 0
          
    # Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp1.header.stamp = rospy.Time.now()
        self.sp2.header.stamp = rospy.Time.now()
        
        #self.sp1.pose.position.x = self.local_pos.x + self.STEP_SIZE*x
        #self.sp1.pose.position.y = self.local_pos.y + self.STEP_SIZE*y


# Main function
def main():

    # Initiate node
    rospy.init_node('offboard_ctl_node', anonymous=True)
    
    # Drone controller object
    cnt1 = Controller()
    cnt2 = Controller()
    
    # ROS loop rate, [Hz]
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('/intel1/mavros/state', State, cnt1.stateCb)
    rospy.Subscriber('/uvify1/mavros/state', State, cnt2.stateCb)
    # Subscribe to drone's local position
    rospy.Subscriber('/intel1/mavros/local_position/pose', PoseStamped, cnt1.posCb)
    rospy.Subscriber('/uvify1/mavros/local_position/pose', PoseStamped, cnt2.posCb)


    # Setpoint publisher
    sp_pub1 = rospy.Publisher('/intel1/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    sp_pub2 = rospy.Publisher('/uvify1/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    
    # Send some setpoint messages to activate OFFBOARD mode
    rospy.loginfo('Sending 20 setpoints...')
    i=0
    while i<20:
        sp_pub1.publish(cnt1.home_sp1)
        sp_pub2.publish(cnt2.home_sp2)

        rate.sleep()
        i = i+1

    # Arm the vehicle
    rospy.loginfo('Arming...')
    cnt1.modes.setArm()
    cnt2.modes.setArm()
    
    # Activate OFFBOARD mode
    rospy.loginfo('Switching to OFFBOARD mode...')
    cnt1.modes.setOffboardMode()
    cnt2.modes.setOffboardMode()

    # ROS main loop
    loop_t = time.time()
    landing = False
    ground_count = 0  # Count number of times position reports on ground
    rospy.loginfo('Streaming home_sp at 20 Hz...')
    while not rospy.is_shutdown():

        # Start waypoint path following
        if(not cnt1.wp_started and not cnt2.wp_started):
            cnt1.wp_started = True
            cnt2.wp_started = True

            cnt1.wp_index = 0
            cnt2.wp_index = 0

            cnt1.wp_timer = time.time()
            cnt2.wp_timer = time.time()

        
        # Update waypoint index every 5 seconds
        if(time.time() - cnt1.wp_timer > 8):
            cnt1.wp_index += 1
            cnt2.wp_index += 1

            cnt1.wp_timer = time.time() # Update wp timer
            cnt2.wp_timer = time.time()
            
            # Land at end of path (4 waypoints)
            if(cnt1.wp_index > (len(cnt1.p1_waypoints)-1)):
              cnt1.wp_finished = True
            if(cnt2.wp_index > (len(cnt2.p2_waypoints)-1)):
              cnt2.wp_finished = True 
            
        
        # Land at end of path (1 waypoints)
        if cnt1.wp_finished:
          if not landing:
            cnt1.modes.setAutoLandMode()
            landing = True
          else:
            if cnt1.local_pos.z < 0.3:
              ground_count += 1
              
            if ground_count >= 20:
              cnt1.modes.setDisarm()
              break
        else:
          # Set current waypoint based on index
          cnt1.wp_pose1.pose.position.x = cnt1.p1_waypoints[cnt1.wp_index][0];
          cnt1.wp_pose1.pose.position.y = cnt1.p1_waypoints[cnt1.wp_index][1];
          sp_pub1.publish(cnt1.wp_pose1)
          
                   
        # Land at end of path (4 waypoints)
        if cnt2.wp_finished:
          if not landing:
            cnt2.modes.setAutoLandMode()
            landing = True
          else:
            if cnt2.local_pos.z < 0.3:
              ground_count += 1
              
            if ground_count >= 20:
              cnt2.modes.setDisarm()
              break
        else:
          # Set current waypoint based on index
          cnt2.wp_pose2.pose.position.x = cnt2.p2_waypoints[cnt2.wp_index][0];
          cnt2.wp_pose2.pose.position.y = cnt2.p2_waypoints[cnt2.wp_index][1];
          sp_pub2.publish(cnt2.wp_pose2)
          
            
        rate.sleep()
             
    # Land the drone if CTRL+C
    cnt1.modes.setAutoLandMode()
    cnt2.modes.setAutoLandMode()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass