#! /usr/bin/env python

'''
    ibc_image_receiver.py
     
    Written by Daniel McArthur
    Date: May 1, 2019
     
    Subscribes to the 'ibc_cam_img' CompressedImage topic and displays
    it with imshow.
    
    The XY pixel location of a user's clicks are recorded and published
    as a geometry_msgs/Point topic: 'ibc_img_click' with x,y being the
    click location and z being the sequence number for the image topic.
    
    NOTE: including the sequence number allows the click tracker
    program to maintain a buffer of previous images to apply the image
    projections from the clicked location on the actual image that was
    clicked (to reduce errors due to latency of image transfer)
    
    ROS Topic Published:
        'ibc_img_click'    (Point): XY click location, Z: img seq #
        
'''
 
import cv2
from geometry_msgs.msg import Point
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

 
# *****************************************************************************
class VideoRecorder ( object ):
    '''
    Records images or videos from a RealSense Stream 
    '''
    def __init__ (self):
        pass
     
# -----------------------------------------------------------------------------
    
class IBCImageReceiver:
 
    def __init__(self):
        '''Initialize ROS and Publishers/Subscribers'''
        rospy.init_node('image_receiver', anonymous=True)
        rospy.loginfo('Initializing image_receiver...')
        rospy.loginfo('Publishing /ibc_img_click...')
        self.clickPub = rospy.Publisher("/ibc_img_click",
 Point, queue_size=10)
        rospy.loginfo('Subscribing to /ibc_cam_img/compressed...')
        self.imgSub = rospy.Subscriber("/ibc_cam_img/compressed",
 CompressedImage,
                                       self.imgCB,  queue_size = 1)
        
        # Store the latest image sequence number
        self.latestImg = None

        # Store clicked pixel with image sequence number
        self.clickPt = Point(x=-1, y=-1, z=-1)

        cv2.namedWindow('I-BC Onboard Camera')
        cv2.setMouseCallback("I-BC Onboard Camera", self.mouseCB)
        
    def mouseCB(self, event, x, y, flags, param):
        ''' Handle OpenCV Mouse events '''

        # Left mouse button released
        if event == cv2.EVENT_LBUTTONUP:
            # Check if a valid image has been received
            if self.latestImg is not None:
                self.clickPt.x = x
                self.clickPt.y = y
                self.clickPt.z = self.latestImg.header.seq
            else:
                self.clickPt.x = -1
                self.clickPt.y = -1
                self.clickPt.z = -1
                
            # Publish the clicked point
            self.clickPub.publish(self.clickPt)
        
        # Right mouse button released (send -10, -10)
        if event == cv2.EVENT_RBUTTONUP:
            # Check if a valid image has been received
            if self.latestImg is not None:
                self.clickPt.x = -10
                self.clickPt.y = -10
                self.clickPt.z = self.latestImg.header.seq
            else:
                self.clickPt.x = -1
                self.clickPt.y = -1
                self.clickPt.z = -1
                
            # Publish the clicked point
            self.clickPub.publish(self.clickPt)

      
    def imgCB(self, imgMsg):
        '''Callback function for ibc_cam_img topic. 
        
        Decompresses images and displays with imshow'''
        
        self.latestImg = imgMsg
 
        # Decompress image and convert to CV2 format 
        np_arr = np.fromstring(imgMsg.data, np.uint8)
        
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
         
         # Display image
        cv2.imshow('I-BC Onboard Camera', image_np)
        cv2.waitKey(2)

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down ROS Image feature detector module"
            
        cv2.destroyAllWindows()
    
 
if __name__ == '__main__':
    receiver = IBCImageReceiver()
    receiver.run()