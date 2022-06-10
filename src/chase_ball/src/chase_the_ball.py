#!/usr/bin/python
"""
Gets the position of the blob and it commands to steer the wheels

Subscribes to 
    /blob/point_blob
    
Publishes commands to 
    /dkcar/control/cmd_vel    

"""
import math, time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2
import cv2

K_LAT_DIST_TO_STEER     = 0.001
K_LAT_DIST_TO_THROTTLE     = 0.0001

def saturate(value, min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)


class ChaseBall():
    def __init__(self):
        
        self.rel_blob_x         = 0.0
        self.rel_blob_y         = 0.0
        self.abs_blob_x         = 0.0
        self.abs_blob_y         = 0.0
        self._time_detected = 0.0
        
        self.sub_center = rospy.Subscriber("/blob/rel_point_blob", Point, self.rel_update_ball)
        rospy.loginfo("Rel Subscribers set")

        self.sub_position = rospy.Subscriber("/blob/abs_point_blob", Point, self.abs_update_ball)
        rospy.loginfo("Abs Subscribers set")
        
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")
        
        self._message = Twist()
        
        self._time_steer        = 0
        self._steer_sign_prev   = 0

        #for depth
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/camera/depth/image_raw', msg_Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.imageDepthInfoCallback)
        self.intrinsics = None
        self.result = [0,0,0]
        
    @property
    def is_detected(self): return(time.time() - self._time_detected < 1.0)
        
    def rel_update_ball(self, message):
        self.rel_blob_x = message.x
        self.rel_blob_y = message.y
        self._time_detected = time.time()
        rospy.loginfo("Ball detected: %.1f  %.1f "%(self.rel_blob_x, self.rel_blob_y))

    def abs_update_ball(self, message):
        self.abs_blob_x = message.x /1.5  #in camera/depth res
        self.abs_blob_y = message.y /1.5

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        
        Steer will be added to the commanded throttle
        throttle will be multiplied by the commanded throttle
        """
        steer_action    = 0.0
        throttle_action = 0.0
        
        if self.is_detected:
            #--- Apply steering, proportional to how close is the object
            steer_action   = K_LAT_DIST_TO_STEER*self.rel_blob_x
            steer_action   = saturate(steer_action, -0.5, 0.5)
            rospy.loginfo("Steering command %.2f"%steer_action)
            if self.result[2] > 1000:
                throttle_action = self.result[2]*K_LAT_DIST_TO_THROTTLE
                throttle_action = saturate(throttle_action, 0, 0.2)
                rospy.loginfo("Throttling command %.2f"%throttle_action)
            else:
                throttle_action = 0.0
        else:
            steer_action    = 0.0
            throttle_action = 0.0

        return (steer_action, throttle_action)
        
    def run(self):
        
        #--- Set the control rate
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            #-- Get the control action 
            steer_action, throttle_action   = self.get_control_action() 
            
            rospy.loginfo("linear x = %3.1f"%(throttle_action))
            rospy.loginfo("angular z = %3.1f"%(steer_action))
            
            #-- update the message
            self._message.linear.x  = throttle_action
            self._message.angular.z = steer_action
            
            #-- publish it
            self.pub_twist.publish(self._message)

            rate.sleep()

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (int(self.abs_blob_x), int(self.abs_blob_y))
            #sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
            #sys.stdout.flush()
            if self.intrinsics:
                depth = cv_image[pix[1], pix[0]]
                self.result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                #rospy.loginfo(result)
                #sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
                #sys.stdout.flush()

        except CvBridgeError as e:
            print(e)
            return

    def imageDepthInfoCallback(self, cameraInfo):
        try:
            # import pdb; pdb.set_trace()
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            #self.intrinsics.coeffs = cameraInfo.D

        except CvBridgeError as e:
            print(e)
            return        
            
if __name__ == "__main__":

    rospy.init_node('chase_ball')

    chase_ball = ChaseBall()
    chase_ball.run()            