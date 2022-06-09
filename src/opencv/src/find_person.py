#!/usr/bin/env python

"""

   0------------------> x (cols) Image Frame
   |
   |        c    Camera frame
   |         o---> x
   |         |
   |         V y
   |
   V y (rows)


SUBSCRIBES TO:
    /raspicam_node/image: Source image topic
    
PUBLISHES TO:
    /blob/image_blob : image with detected blob and search window
    /blob/point_blob : blob position in adimensional values wrt. camera frame

"""


#--- Allow relative importing
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    
import sys
import rospy
import cv2
import time

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from include.color_detection  import simple_detect_bbox


class SimpleColorDetector:

    def __init__(self):
        
        self._t0 = time.time()
        self.blob_point = Point()
    
        print (">> Publishing image to topic image_blob")
        self.image_pub = rospy.Publisher("/blob/image_blob",Image,queue_size=1)
        print (">> Publishing position to topic point_blob")
        self.blob_pub  = rospy.Publisher("/blob/point_blob",Point,queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
        print ("<< Subscribed to topic /camera/color/image_raw")
        
        
    def callback(self,data):
        #--- Assuming image is 320x240
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            #--- Detect blobs
            cv_image, x, y, w, h = simple_detect_bbox(cv_image)
            if x is not None:
                # calculate delta x, y
                center_x = 0.5*rows
                center_y = 0.5*cols
                delta_x = (x - center_x)/center_x
                delta_y = -1 * (y - center_y)/center_y

                self.blob_point.x = delta_x
                self.blob_point.y = delta_y
                self.blob_pub.publish(self.blob_point) 
            
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)
                    
            fps = 1.0/(time.time()-self._t0)
            self._t0 = time.time()
            

if __name__ == '__main__':
    
    rospy.init_node('blob_detector', anonymous=True)
    detector = SimpleColorDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()
