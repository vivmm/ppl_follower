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
    /blob/rel_point_blob : blob position in adimensional values wrt. camera frame
    /blob/abs_point_blob : absolute blob position

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
        self.rel_blob_point = Point()
        self.abs_blob_point = Point()
    
        print (">> Publishing image to topic image_blob")
        self.image_pub = rospy.Publisher("/blob/image_blob",Image,queue_size=1)
        print (">> Publishing position to topic rel_point_blob")
        self.rel_blob_pub  = rospy.Publisher("/blob/rel_point_blob",Point,queue_size=1)
        print (">> Publishing position to topic abs_point_blob")
        self.abs_blob_pub  = rospy.Publisher("/blob/abs_point_blob",Point,queue_size=1)


        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
        print ("<< Subscribed to topic /camera/color/image_raw")
        
        
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if cv_image is not None :
            (rows,cols,channels) = cv_image.shape
            #--- Detect blobs
            cv_image, x, y, w, h = simple_detect_bbox(cv_image, "blue")

            # calculate delta x, y
            delta_x = ((x+w/2) - cols / 2)
            delta_y = -1 * ((y+h/2) - rows / 2)

            self.abs_blob_point.x = x+w/2
            self.abs_blob_point.y = y+h/2
            self.abs_blob_pub.publish(self.abs_blob_point)

            self.rel_blob_point.x = delta_x
            self.rel_blob_point.y = delta_y
            self.rel_blob_pub.publish(self.rel_blob_point)

            # visualize
            # center of object
            cv2.circle(cv_image, (int(x+w/2), int(y+h/2)), 7, (0, 0, 255), -1)
            # center of image
            cv2.circle(cv_image, (int(cols/2), int(rows/2)), 7, (255, 0, 0), -1)
            cv2.putText(cv_image, "({},{})".format(delta_x, delta_y), (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 0, 255))
            
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
