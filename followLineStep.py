#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from lidar import moveRobot

class lineFollower(object):

    def __init__(self):
        self.bridgeObject = CvBridge()
        self.imageSub = rospy.Subscriber("/camera/image",Image,self.camera_callback)
        self.moveRobotObject = Move()

    def camera_callback(self,data):
        try:
            cvImage = self.bridgeObject.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        height, width, channels = cvImage.shape
        cropImg = cvImage[:,:]
        
        hsv = cv2.cvtColor(cropImg, cv2.COLOR_BGR2HSV)
        lower = np.array([])
        upper = np.array([])

        cv2.imshow("RES", res)
        
        cv2.waitKey(1)
        
        mask = cv2.inRange(hsv, lower, upper)
        
        # Cacular momentos de inercia
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        
        res = cv2.bitwise_and(cropImg,cropImg, mask=mask)
        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)

        cv2.imshow("Original", cvImage)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.imshow("RES", res)
        
        cv2.waitKey(1)
        
        errorX = cx - width/2;
        twistObject = Twist();
        twistObject.linear.x = 0.2;
        twistObject.angular.z = -errorX/100;
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(twistObject.angular.z))

        self.moveRobotObject.move_robot(twistObject)
        
    def cleanUp(self):
        self.moveRobotObject.cleanClass()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('lineFollowingNode', anonymous=True)
    lineFollowerObject = lineFollower()
   
    rate = rospy.Rate(5)
    ctrlC = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        lineFollowerObject.cleanUp()
        rospy.loginfo("shutdown time!")
        ctrlC = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrlC:
        rate.sleep()
    
if __name__ == '__main__':
    main()

