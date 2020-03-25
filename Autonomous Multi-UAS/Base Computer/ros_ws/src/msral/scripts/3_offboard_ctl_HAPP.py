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
        rospy.wait_for_service('/uav1/mavros/cmd/arming')
        rospy.wait_for_service('/uav2/mavros/cmd/arming')
        rospy.wait_for_service('/uav3/mavros/cmd/arming')
        try:
            armService1 = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService2 = rospy.ServiceProxy('/uav2/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService3 = rospy.ServiceProxy('/uav3/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            response1 = armService1(True)
            response2 = armService2(True)
            response3 = armService3(True)
            return response1.success, response2.success, response3.success
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e
            return False, False, False

    def setDisarm(self):
        rospy.wait_for_service('/uav1/mavros/cmd/arming')
        rospy.wait_for_service('/uav2/mavros/cmd/arming')
        rospy.wait_for_service('/uav3/mavros/cmd/arming')
        try:
            armService1 = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService2 = rospy.ServiceProxy('/uav2/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService3 = rospy.ServiceProxy('/uav3/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            response1 = armService1(False)
            response2 = armService2(False)
            response3 = armService3(False)
            return response1.success, response2.success, response3.success
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e
            return False, False, False
    rospy.loginfo('Streaming home_sp at 20 Hz...')

    def setStabilizedMode(self):
        rospy.wait_for_service('/uav1/mavros/set_mode')
        rospy.wait_for_service('/uav2/mavros/set_mode')
        rospy.wait_for_service('/uav3/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2 = rospy.ServiceProxy('/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService3 = rospy.ServiceProxy('/uav3/mavros/set_mode', mavros_msgs.srv.SetMode) 
            response1 = flightModeService1(custom_mode='STABILIZED')
            response2 = flightModeService2(custom_mode='STABILIZED')
            response3 = flightModeService3(custom_mode='STABILIZED')
            return response1.mode_sent, response2.mode_sent, response3.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e
            return False, False, False

    def setOffboardMode(self):
        rospy.wait_for_service('/uav1/mavros/set_mode')
        rospy.wait_for_service('/uav2/mavros/set_mode')
        rospy.wait_for_service('/uav3/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2 = rospy.ServiceProxy('/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService3 = rospy.ServiceProxy('/uav3/mavros/set_mode', mavros_msgs.srv.SetMode)
            response1 = flightModeService1(custom_mode='OFFBOARD')
            response2 = flightModeService2(custom_mode='OFFBOARD')
            response3 = flightModeService3(custom_mode='OFFBOARD')
            return response1.mode_sent, response2.mode_sent, response3.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e
            return False, False, False

    def setAltitudeMode(self):
        rospy.wait_for_service('/uav1/mavros/set_mode')
        rospy.wait_for_service('/uav2/mavros/set_mode')
        rospy.wait_for_service('/uav3/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2 = rospy.ServiceProxy('/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService3 = rospy.ServiceProxy('/uav3/mavros/set_mode', mavros_msgs.srv.SetMode)
            response1 = flightModeService1(custom_mode='ALTCTL')
            response2 = flightModeService2(custom_mode='ALTCTL')
            response3 = flightModeService3(custom_mode='ALTCTL')
            return response1.mode_sent, response2.mode_sent, response3.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e
            return False, False, False

    def setPositionMode(self):
        rospy.wait_for_service('/uav1/mavros/set_mode')
        rospy.wait_for_service('/uav2/mavros/set_mode')
        rospy.wait_for_service('/uav3/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2 = rospy.ServiceProxy('/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService3 = rospy.ServiceProxy('/uav3/mavros/set_mode', mavros_msgs.srv.SetMode)
            response1 = flightModeService1(custom_mode='POSCTL')
            response2 = flightModeService2(custom_mode='POSCTL')
            response3 = flightModeService3(custom_mode='POSCTL')
            return response1.mode_sent, response2.mode_sent, response3.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e
            return False, False, False

    def setAutoLandMode(self):
        rospy.wait_for_service('/uav1/mavros/set_mode')
        rospy.wait_for_service('/uav2/mavros/set_mode')
        rospy.wait_for_service('/uav3/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2 = rospy.ServiceProxy('/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService3 = rospy.ServiceProxy('/uav3/mavros/set_mode', mavros_msgs.srv.SetMode)
            response1 = flightModeService1(custom_mode='AUTO.LAND')
            response2 = flightModeService2(custom_mode='AUTO.LAND')
            response3 = flightModeService3(custom_mode='AUTO.LAND')
            return response1.mode_sent, response2.mode_sent, response3.mode_sent
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
        self.ALT_SP2        = 2
        self.ALT_SP3        = 3
        
        
        # Instantiate setpoint messages
        self.sp1         = PoseStamped()
        self.sp2         = PoseStamped()
        self.sp3         = PoseStamped()
        self.home_sp1    = PoseStamped()
        self.home_sp2    = PoseStamped()
        self.home_sp3    = PoseStamped()      
        self.home_sp1.pose.position.x = 0
        self.home_sp1.pose.position.y = -1
        self.home_sp1.pose.position.z = self.ALT_SP1
        self.home_sp2.pose.position.x = 0
        self.home_sp2.pose.position.y = 0
        self.home_sp2.pose.position.z = self.ALT_SP2
        self.home_sp3.pose.position.x = 0
        self.home_sp3.pose.position.y = 1
        self.home_sp3.pose.position.z = self.ALT_SP3
        
        self.sp1.pose.position.x = 0
        self.sp1.pose.position.y = -1
        self.sp1.pose.position.z = self.ALT_SP1
        self.sp1.pose.position.x = 0
        self.sp1.pose.position.y = 0
        self.sp1.pose.position.z = self.ALT_SP2
        self.sp1.pose.position.x = 0
        self.sp1.pose.position.y = 1
        self.sp1.pose.position.z = self.ALT_SP3

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0.0)
        
        # Handler to set FCU modes
        self.modes = fcuModes()

        
        # Create waypoints for path traversal
        self.wp_pose1 = PoseStamped()
        self.wp_pose2 = PoseStamped()
        self.wp_pose3 = PoseStamped()
        self.wp_pose1.pose.position.z = self.ALT_SP1
        self.wp_pose2.pose.position.z = self.ALT_SP2
        self.wp_pose3.pose.position.z = self.ALT_SP3
        
        locn_file = open("../textfiles/3_locn.txt","r") 

        self.waypoints = []

        for locn_line in locn_file:   
            sep_xy = locn_line.split(" ")
            sep_xy[1] = sep_xy[1].replace("\n", "");
            self.waypoints.append((int(sep_xy[0]), int(sep_xy[1])))
            
        locn_file.close()
        
        fp_file = open("../textfiles/3_finalpath.txt","r")
        
        fp = []
        self.new_waypoints = []
        self.p1_waypoints = []
        self.p2_waypoints = []
        self.p3_waypoints = []
        for fp_line in fp_file:
            final_path = fp_line.split(" ")
            for i in range(len(final_path)-1):
                fp.append((int(final_path[i])))
                self.new_waypoints.append(self.waypoints[int(fp[i])-1])
            fp = []
            
        path_idx = len(self.new_waypoints)-(len(final_path)-1)        
        
        #self.p1_waypoints, self.p2_waypoints, self.p3_waypoints = self.new_waypoints[:path_idx], self.new_waypoints[path_idx:]
        self.p1_waypoints, self.p2_waypoints, self.p3_waypoints = self.new_waypoints[:5], self.new_waypoints[5:8], self.new_waypoints[8:]
                
        fp_file.close()
        
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
        self.sp3.header.stamp = rospy.Time.now()
        
        #self.sp1.pose.position.x = self.local_pos.x + self.STEP_SIZE*x
        #self.sp1.pose.position.y = self.local_pos.y + self.STEP_SIZE*y


# Main function
def main():

    # Initiate node
    rospy.init_node('offboard_ctl_node', anonymous=True)
    
    # Drone controller object
    cnt1 = Controller()
    cnt2 = Controller()
    cnt3 = Controller()
    
    # ROS loop rate, [Hz]
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('/uav1/mavros/state', State, cnt1.stateCb)
    rospy.Subscriber('/uav2/mavros/state', State, cnt2.stateCb)
    rospy.Subscriber('/uav3/mavros/state', State, cnt3.stateCb)
    # Subscribe to drone's local position
    rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, cnt1.posCb)
    rospy.Subscriber('/uav2/mavros/local_position/pose', PoseStamped, cnt2.posCb)
    rospy.Subscriber('/uav3/mavros/local_position/pose', PoseStamped, cnt3.posCb)


    # Setpoint publisher
    sp_pub1 = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    sp_pub2 = rospy.Publisher('/uav2/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    sp_pub3 = rospy.Publisher('/uav3/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    
    # Send some setpoint messages to activate OFFBOARD mode
    rospy.loginfo('Sending 20 setpoints...')
    i=0
    while i<20:
        sp_pub1.publish(cnt1.home_sp1)
        sp_pub2.publish(cnt2.home_sp2)
        sp_pub3.publish(cnt3.home_sp3)

        rate.sleep()
        i = i+1

    # Arm the vehicle
    rospy.loginfo('Arming...')
    cnt1.modes.setArm()
    cnt2.modes.setArm()
    cnt3.modes.setArm()
    
    # Activate OFFBOARD mode
    rospy.loginfo('Switching to OFFBOARD mode...')
    cnt1.modes.setOffboardMode()
    cnt2.modes.setOffboardMode()
    cnt3.modes.setOffboardMode()

    # ROS main loop
    loop_t = time.time()
    landing = False
    ground_count = 0  # Count number of times position reports on ground
    rospy.loginfo('Streaming home_sp at 20 Hz...')
    while not rospy.is_shutdown():

        # Start waypoint path following
        if(not cnt1.wp_started and not cnt2.wp_started and not cnt3.wp_started):
            cnt1.wp_started = True
            cnt2.wp_started = True
            cnt3.wp_started = True

            cnt1.wp_index = 0
            cnt2.wp_index = 0
            cnt3.wp_index = 0

            cnt1.wp_timer = time.time()
            cnt2.wp_timer = time.time()
            cnt3.wp_timer = time.time()
                
                      
        if(time.time() - cnt1.wp_timer > 6):
            if(cnt1.wp_index < (len(cnt1.p1_waypoints)-1)):
              cnt1.wp_index += 1
            if(cnt2.wp_index < (len(cnt2.p2_waypoints)-1)):
              cnt2.wp_index += 1
            if(cnt3.wp_index < (len(cnt3.p3_waypoints)-1)):
              cnt3.wp_index += 1

            cnt1.wp_timer = time.time() # Update wp timer
            cnt2.wp_timer = time.time()
            cnt2.wp_timer = time.time()
            
            # Land at end of path (4 waypoints)
            if(cnt1.wp_index > (len(cnt1.p1_waypoints)-1)):
              cnt1.wp_finished = True
            if(cnt2.wp_index > (len(cnt2.p2_waypoints)-1)):
              cnt2.wp_finished = True  
            if(cnt3.wp_index > (len(cnt3.p3_waypoints)-1)):
              cnt3.wp_finished = True 
        
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
          
         
        # Land at end of path (4 waypoints)
        if cnt3.wp_finished:
          if not landing:
            cnt3.modes.setAutoLandMode()
            landing = True
          else:
            if cnt3.local_pos.z < 0.3:
              ground_count += 1
              
            if ground_count >= 20:
              cnt3.modes.setDisarm()
              break
        else:
          # Set current waypoint based on index
          cnt3.wp_pose3.pose.position.x = cnt3.p3_waypoints[cnt3.wp_index][0];
          cnt3.wp_pose3.pose.position.y = cnt3.p3_waypoints[cnt3.wp_index][1];
          sp_pub3.publish(cnt3.wp_pose3)
          
            
        rate.sleep()
             
    # Land the drone if CTRL+C
    cnt1.modes.setAutoLandMode()
    cnt2.modes.setAutoLandMode()
    cnt3.modes.setAutoLandMode()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass