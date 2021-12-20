import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import numpy as np 
import sys, os 

class readData(object):
    def __init__(self): 
        self.initScan()    
        self.laserSub = rospy.Subscriber("/scan",LaserScan,self.scanCallback)

    def initScan(self):
        self._scan= None
        while self._scan is None:
            try:
                self._scan = rospy.wait_for_message("/scan",LaserScan,timeout=1)
            except:
                rospy.loginfo("/scan topic is not ready yet, retrying")

        rospy.loginfo("/scan topic is ready")
    
    def scanCallback(self,msg):
        self._scan=msg

    def getData(self):
        return self._scan.range
