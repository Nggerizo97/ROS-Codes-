#!/usr/bin/env python
# coding=utf-8

from __future__ import print_function
import sys, os
import rospy
import cv2
from sensor_msgs.msg import Image # Funcion de opencv encargada de convertir el mensaje en una imagen de
from cv_bridge import CvBridge, CvBridgeError # opencv y viceversa
import numpy as np 
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from lidar import readData



class imageReader:

    def __init__(self):
        # Creamos el topico en el que publicaremos la imagen resultado
        #self.imagePub = rospy.Publisher("opencTopic", Image, queue_size=1)
        self.velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bridge = CvBridge()  # Creamos un objeto para realizar la conversion de la imagen
        # Creamos el subcriptor al topico de la camara
        self.imageSub = rospy.Subscriber("/camera/image", Image, self.callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometryCb)
        
        
        self.laserSub = rospy.Subscriber("/scan",LaserScan,self.scanCallback)

    
        
    def scanCallback(self,msg): # Se define una funcion para almacena el mensaje en una variable global de la clase
        self._scan = msg

    def getData(self): # Se define una funcion para la lectura de los datos
        return self._scan.ranges

    def publishCmdVel(self,twistObject):
          self.velPublisher.publish(twistObject)

    def callback(self,data):
        try:
            # Con CvBridge convertimos el mensaje recibido del topico
            cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # en una imagaen de opencv
        except CvBridgeError as e:
            print(e)  # En caso de que suceda un error, el sistema imprimira una e

        # Realizamos una conversion a escala de grises de la imagen
        
        
        IMAGE_H = 240
        IMAGE_W = 320
        '''
        src = np.float32([[0, IMAGE_H], # Inferior izquierda
                  [23, IMAGE_H], # Ingerior derecha
                  [0, 0],  # superior izquierda
                  [IMAGE_W, 0]]) # superior
        #cv2.imshow('image',cvImage) derecha
        dst = np.float32([[68, IMAGE_H], # Inferior izquierda
                  [78, IMAGE_H], # Ingerior derecha
                  [0, 0], # superior izquierda
                  [IMAGE_W, 0]]) # superior derecha
        
        M = cv2.getPerspectiveTransform(src, dst) # Matrix de transformacion
        Minv = cv2.getPerspectiveTransform(dst, src) # Matrix de transformacion inversa

        cvImage = cvImage[(IMAGE_H/2-15):IMAGE_H, 0:IMAGE_W] # Seleccion de una seccion de la imagen
        cvImage1 = cv2.warpPerspective(cvImage, M, (IMAGE_W, IMAGE_H)) # Transformacion de la imagen
        
        '''
        #cv2.imshow('image',cvImage)
        #cv2.imshow('image1',cvImage1)
        #cv2.waitKey(1)
        
        lower_yellow=np.array([5,255,255])
        upper_yellow=np.array([65,255,255])
        maskyellow = cv2.inRange(cvImage, lower_yellow, upper_yellow)

        lower_white=np.array([253,253,253])
        upper_white=np.array([255,255,255])
        maskwhite = cv2.inRange(cvImage, lower_white, upper_white)

        lower_red=np.array([30,30,190])
        upper_red=np.array([55,48,220])
        maskred = cv2.inRange(cvImage, lower_red, upper_red)

        lower_blue=np.array([170,100,0])
        upper_blue=np.array([200,125,0])
        maskblue = cv2.inRange(cvImage, lower_blue, upper_blue)
        
        lower_redt=np.array([41,10,230])
        upper_redt=np.array([70,30,250])
        maskredt = cv2.inRange(cvImage, lower_redt, upper_redt)

        lower_yellowt=np.array([1,205,253])
        upper_yellowt=np.array([20,219,255])
        maskyellowt = cv2.inRange(cvImage, lower_yellowt, upper_yellowt)

         
        canny = cv2.Canny(cvImage, 150,200, apertureSize=3)  # filtro canny
        
        mask2=maskyellowt+maskredt
        

        
        mask=maskwhite+maskyellow
        mask = mask[170:230, :]
        
        maskIz = mask[:, 0:IMAGE_W/2]
        maskDer = mask[:, IMAGE_W/2:IMAGE_W]
    
        cv2.imshow('mask2', maskIz)
        cv2.imshow('mask3', maskDer)
        #cv2.imshow('imagen',cvImage)
        #cv2.imshow('redt', maskredt)
        #cv2.imshow('white', maskwhite)
        #cv2.imshow('image3',maskred)
        #cv2.imshow('iz',maskIz)
        #cv2.imshow('der',maskDer)
    
        cv2.waitKey(1)
      
        '''m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = IMAGE_W/2, IMAGE_H/2

        
        errorX = cx - IMAGE_W/2;'''
        

        '''twistObject = Twist();
        twistObject.linear.x = 0.1
        if np.mean(maskIz) == 0: #and bandera == 0
            twistObject.angular.z = 0.5
        elif np.mean(maskDer) == 0:
            twistObject.angular.z = -0.5
        else:
            twistObject.angular.z = 0.0
            

        if np.mean(maskred)>0 and np.mean(maskDer)==0 and np.mean(maskIz) ==0:
            twistObject.angular.z = 0.0
        
        if np.mean(mask2)>3:
            twistObject.angular.z = 0.0
            twistObject.linear.x = 0.0
        
        
        
        if np.mean(maskblue)>3.5 and np.mean(maskblue)<6.5:
            twistObject.angular.z = -0.4
            twistObject.linear.x = 0.08
        elif np.mean(maskblue)>6.5 and np.mean(maskblue)<18:
            twistObject.angular.z = 0.4
            twistObject.linear.x = 0.08
        elif np.mean(maskblue)>16:
            activo=False

        if np.mean(maskblue)<3.5:
            twistObject.angular.z = -0.9
            twistObject.linear.x = -0.8

        rospy.loginfo("ANGULAR VALUE SENT===>"+str(twistObject.angular.z))
        
        self.publishCmdVel(twistObject)'''

    def move(self, side=0.0): # Se crea una funcion para la operacion
        while True:
            distances = self.getData() # Se almacena en una variable los datos leidos desde la clase de lectura de datos
            print (distances)    
    
    #def odometryCb(self,msg):
        #pass
    
        #print (msg.pose.pose)
        #
	
                
    '''if pendejo[pendejo>0 or pendejo< 1]:
                    twistObject.angular.z = 0.4
                    twistObject.linear.x = 0.3
                else:
                    twistObject.angular.z = 0.0
                    twistObject.linear.x = 0.3
    

           # distances[distances>1 or distances< 5] = 2

            x=distances
            w=[msg]

            while x > 0:   
                move(x,w)

                reload(x,zt,w)
                #load(w)
            
            while w < -1 :
                for i = ut:
                    dist= x-i
                    w< Norm(dist,ut)'''




def main(args):
    ir = imageReader()  # Iniciamos la clase
    rospy.init_node('imageReading', anonymous=True)  # Creamos el nodo
    ir.move(0.0)
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
    
    
    
    

