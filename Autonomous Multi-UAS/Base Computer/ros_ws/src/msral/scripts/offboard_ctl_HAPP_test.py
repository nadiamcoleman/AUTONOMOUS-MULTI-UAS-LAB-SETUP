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
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            response = armService(True)
            return response.success
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e
            return False

    def setDisarm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            response = armService(False)
            return response.success
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e
            return False

    def setStabilizedMode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            response = flightModeService(custom_mode='STABILIZED')
            return response.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e
            return False

    def setOffboardMode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            response = flightModeService(custom_mode='OFFBOARD')
            return response.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e
            return False

    def setAltitudeMode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            response = flightModeService(custom_mode='ALTCTL')
            return response.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e
            return False

    def setPositionMode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            response = flightModeService(custom_mode='POSCTL')
            return response.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e
            return False

    def setAutoLandMode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            response = flightModeService(custom_mode='AUTO.LAND')
            return response.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Autoland Mode could not be set."%e
            return False

# Main class: Converts joystick commands to position setpoints
class Controller:
    # initialization method
    def __init__(self):
      
        # Drone state
        self.state = State()
        
        # Altitude setpoint, [meters]
        self.ALT_SP        = 1
        
        # Instantiate setpoint messages
        self.sp         = PoseStamped()
        self.home_sp    = PoseStamped()
        
        self.home_sp.pose.position.x = 0
        self.home_sp.pose.position.y = 0
        self.home_sp.pose.position.z = self.ALT_SP
        
        self.sp.pose.position.x = 0
        self.sp.pose.position.y = 0
        self.sp.pose.position.z = self.ALT_SP

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0.0)
        
        # Handler to set FCU modes
        self.modes = fcuModes()
        
        # Create waypoints for path traversal
        self.wp_pose = PoseStamped()
        self.wp_pose.pose.position.z = self.ALT_SP
        #self.waypoints = []
        #self.waypoints.append([0.0, 0.5])
        #self.waypoints.append([0.0, -0.5])
        #self.waypoints.append([-1.0, -0.5])
        #self.waypoints.append([-1.0, 0.5])
        
        file = open("../textfiles/locn.txt","r") 

        self.waypoints = []

        for line in file:   
            sep_xy = line.split(" ")
            sep_xy[1] = sep_xy[1].replace("\n", "");
            self.waypoints.append((int(sep_xy[0]), int(sep_xy[1])))
            
        file.close()
        
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
        self.sp.header.stamp = rospy.Time.now()
        
        #self.sp.pose.position.x = self.local_pos.x + self.STEP_SIZE*x
        #self.sp.pose.position.y = self.local_pos.y + self.STEP_SIZE*y


# Main function
def main():

    # Initiate node
    rospy.init_node('offboard_ctl_node', anonymous=True)
    
    # Drone controller object
    cnt = Controller()

    # ROS loop rate, [Hz]
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('/mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Setpoint publisher
    sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)

    # Send some setpoint messages to activate OFFBOARD mode
    rospy.loginfo('Sending 20 setpoints...')
    i=0
    while i<20:
        sp_pub.publish(cnt.home_sp)
        rate.sleep()
        i = i+1

    # Arm the vehicle
    rospy.loginfo('Arming...')
    cnt.modes.setArm()
    
    # Activate OFFBOARD mode
    rospy.loginfo('Switching to OFFBOARD mode...')
    cnt.modes.setOffboardMode()

    # ROS main loop
    loop_t = time.time()
    landing = False
    ground_count = 0  # Count number of times position reports on ground
    rospy.loginfo('Streaming home_sp at 20 Hz...')
    while not rospy.is_shutdown():

        # Start waypoint path following
        if(not cnt.wp_started):
            cnt.wp_started = True
            cnt.wp_index = 0
            cnt.wp_timer = time.time()
        
        # Update waypoint index every 5 seconds
        if(time.time() - cnt.wp_timer > 8):
            cnt.wp_index += 1
            cnt.wp_timer = time.time() # Update wp timer
            
            # Land at end of path (4 waypoints)
            if(cnt.wp_index > 3):
              cnt.wp_finished = True
        
        # Land at end of path (4 waypoints)
        if cnt.wp_finished:
          if not landing:
            cnt.modes.setAutoLandMode()
            landing = True
          else:
            if cnt.local_pos.z < 0.3:
              ground_count += 1
              
            if ground_count >= 20:
              cnt.modes.setDisarm()
              break
        else:
          # Set current waypoint based on index
          cnt.wp_pose.pose.position.x = cnt.waypoints[cnt.wp_index][0];
          cnt.wp_pose.pose.position.y = cnt.waypoints[cnt.wp_index][1];
          sp_pub.publish(cnt.wp_pose)
          
        rate.sleep()
        
    # Land the drone if CTRL+C
    cnt.modes.setAutoLandMode()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass