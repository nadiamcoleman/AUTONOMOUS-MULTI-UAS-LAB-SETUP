#!/usr/bin/env python

import rospy
from mavros_msgs.msg import HomePosition
from geometry_msgs.msg import Point, PoseStamped, Quaternion

viconDat = None
def viconCB(msg):
    global viconDat
    viconDat = msg

rospy.init_node('set_home_node', anonymous = True)
rospy.loginfo("Publishing: '/mavros/home_position/set'")
rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, viconCB)

# Publish home position
pubHome = rospy.Publisher('/mavros/home_position/set', HomePosition, queue_size=10)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
	#if viconDat is not None:
    if viconDat is None:
        msg = HomePosition()
        #msg.position = #viconDat.pose.position
        #msg.orientation = #viconDat.pose.orientation
        msg.header.stamp = rospy.Time.now()
        pubHome.publish(msg)
