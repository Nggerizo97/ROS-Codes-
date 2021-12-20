#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
from sensor_msgs.msg import Image # Funcion de opencv encargada de convertir el mensaje en una imagen de
from cv_bridge import CvBridge, CvBridgeError # opencv y viceversa
import numpy as np 
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image



class imageReader:

    def __init__(self):
        # Creamos el topico en el que publicaremos la imagen resultado
        self.imagePub = rospy.Publisher("opencTopic", Image, queue_size=1)
        self.velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bridge = CvBridge()  # Creamos un objeto para realizar la conversion de la imagen
        # Creamos el subcriptor al topico de la camara
        self.imageSub = rospy.Subscriber("/camera/image", Image, self.callback)
        
    def publishCmdVel(self,twistObject):
          self.velPublisher.publish(twistObject)

    def callback(self, data):
        try:
            # Con CvBridge convertimos el mensaje recibido del topico
            cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # en una imagaen de opencv
        except CvBridgeError as e:
            print(e)  # En caso de que suceda un error, el sistema imprimira una e

        # Realizamos una conversion a escala de grises de la imagen

        cv2.imageshow('img1',cvImage)
        cv2.waitKey(1)
        '''twistObject = Twist();
        twistObject.linear.x = 0.2
        twistObject.angular.z = 0.2
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(twistObject.angular.z))
        self.publishCmdVel(twistObject)'''

def main(args):
    ir = imageReader()  # Iniciamos la clase
    rospy.init_node('imageReading', anonymous=True)  # Creamos el nodo
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)