#!/usr/bin/env python
# coding=utf-8

from __future__ import print_function
import cv2
import math
from sensor_msgs.msg import Image # Funcion de opencv encargada de convertir el mensaje en una imagen de
from cv_bridge import CvBridge, CvBridgeError # opencv y viceversa
from nav_msgs.msg import Odometry
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import numpy as np
import sys, os
from cv2 import sqrt


class readData(object): # Creo una clase para leer los datos
    def __init__(self): # Inicio el constructor
        self.initScan() # Define una propiedad para inicial el escaneo
        self.laserSub = rospy.Subscriber("/scan",LaserScan,self.scanCallback) # Creo el subsciptor
        

    def initScan(self): # Inicio el escaneo
        self._scan = None # Creo una variable de control
        while self._scan is None: # Mientras la condicion se cumpla
            try:                
                self._scan = rospy.wait_for_message("/scan", LaserScan, timeout=1) # Se espera a la lectura de un mensaje
            except:
                rospy.loginfo("/scan topic is not ready yet, retrying") # Sino se tiene exito se espera a su lectura
        
        rospy.loginfo("/scan topic READY") # Se informa de su lectura

    def scanCallback(self,msg): # Se define una funcion para almacena el mensaje en una variable global de la clase
        self._scan = msg

    def getData(self): # Se define una funcion para la lectura de los datos
        return self._scan.ranges # La funcion al ser invocada retorna los datos que se encuentran dentro de ranges

class moveRobot(object): # Se crea una clase para realizar los movimientos del robot
    def __init__(self):
        self.scanObject = readData() # Se crea una variable global en la clase que invoque la la clase de lectura de datos
        self.velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # Se crea el publicador
        self.ctrl_c = False # Se crea una variable de control
        #rospy.on_shutdown(self.shutdownhook) # Se determina el estado de rospy
        self.rate = rospy.Rate(10) # Se define una velocidad de operacion
        self.bridge = CvBridge()  # Creamos un objeto para realizar la conversion de la imagen
        self.imageSub = rospy.Subscriber("/camera/image", Image, self.callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometryCb)
        self.odom_al() 

    def odom_al(self):
        self.omsg = None
        while self.omsg is None: # Mientras la condicion se cumpla
            try:                
                self.omsg = rospy.wait_for_message("/scan", Odometry, timeout=1) # Se espera a la lectura de un mensaje
            except:
                rospy.loginfo("/scan topic is not ready yet, retrying") # Sino se tiene exito se espera a su lectura
        
        rospy.loginfo("/scan topic READY") # Se informa de su lectura

    def odometryCb(self,msg):
        self.omsg = msg

    def odomget(self):
        return self.omsg

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
        cv2.waitKey(1)

        twistObject = Twist();
        '''twistObject.linear.x = 0.1
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
            twistObject.linear.x = 0.0'''

        #rospy.loginfo("ANGULAR VALUE SENT===>"+str(twistObject.angular.z))
        
        self.publishCmdVel(twistObject)
    
    def odometryCb(self,msg):
        pass
        self.odom_al=msg
        
        #print(msg.pose.pose.position.x)
        

        #print (msg.pose.pose) #d(A,B) = √((XB – XA)^2)

    def move(self, side=0.0): # Se crea una funcion para la operacion
        while True:
            distances = self.scanObject.getData() # Se almacena en una variable los datos leidos desde la clase de lectura de datos
            #print (distances)
            twistObject = Twist();
            distances2 =np.asarray(distances)
            distances2[(distances2>0) & (distances2<0.8)] = 1
            distances2[distances2>0.8] = 2
            odomx = self.odom_al.pose.pose.position.x
            odomy = self.odom_al.pose.pose.position.y
            #print(distances2)
            xi = 0.244892746506
            yi = -1.7863897705
            zi = -0.00100242711874

            xf= -1.77453399164
            yf= -0.0762674207818
            zf= -0.00100253866552

            A=(xi,yi)

            B=(xf,yf)

            d1=sqrt((xi-xf)**2+(yi-yf)**2)
            d2=sqrt((xi-np.mean(odomx))**2+(yi-np.mean(odomy))**2)
            diferencia=d1-d2
            #print(diferencia)
            #print(type(d1))
            print(type(d2))


            # Inicio de slam y posicionamiento del robot
            '''if d2 > 2.3:
                twistObject.angular.z = d1-np.mean(diferencia)
                twistObject.linear.x = 0.2
            if np.mean(distances2) == 1:
                twistObject.angular.z = 0.5
                twistObject.linear.x = 0.0'''


            '''if np.mean(distances2) == 1:
                twistObject.angular.z = 0.5
                twistObject.linear.x = 0.0
            elif np.mean(distances2) == 2:
                twistObject.angular.z = 0.0
                twistObject.linear.x = 0.3'''


 
            #rospy.loginfo("ANGULAR VALUE SENT===>"+str(twistObject.angular.z))
        
            self.publishCmdVel(twistObject)
        

               

if __name__ == '__main__': # Se inicia la operacion del codigo
    rospy.init_node('test', anonymous=True) # Se inicia el nodo
    moveObject = moveRobot() # Se invoca la clase de movimiento
    try:
        moveObject.move() # Se invoca la funcion de operacion que se encuentra dentro de la clase de movimiento
    except rospy.ROSInterruptException:
        pass
