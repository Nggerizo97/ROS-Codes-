#!/usr/bin/env python
import rospy # Libreria para la comunicacion con ROS
from geometry_msgs.msg import Twist # Mensaje utilizado para el movimiento de los motores
import time
import readData

class Move(): # Se define la clase
    
    def __init__(self): # Constructor de la clase, con este se crea una nueva instancia en la clase
        self.velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # Se crea un objeto propia de la clase que sera el Publisher que dara las ordenes al robot
        self.ctrlC = False # Se define una propiedad de control para activacion de las acciones
        rospy.on_shutdown(self.shutdownhook) # Se define la propiedad de apagado de laclase
        self.rate = rospy.Rate(10) # 10hz # Se define la propiedad que determina la velocidad de operacion (Hz)
        self.data = readData.readData()

    def publishCmdVel(self, cmd): # Se crea una funcion porpia de la clase, la cual se encargara de publicar los comandos de velocidad, este recibe opera sobre la clase (self) y recibe el mensaje a publicar
        while not self.ctrlC: # El ciclo se ejecuta mientras la condicion sea cumploda
            connections = self.velPublisher.get_num_connections() # Con get_num_connections se determina la cantidad de conecciones que tiene el Publisher
            if connections > 0: # En caso de que halla conexiones
                self.velPublisher.publish(cmd) # Se publica el mensaje
                rospy.loginfo("Cmd Published") # Se da un mensaje de informe al sistema
                break # Y finalizo el ciclo
            else: # En caso de no existir conexiones
                self.rate.sleep() # Se genera una pausa 
    
    
    def shutdownhook(self): # Se define la funcion de apagado que finalizara detendra el robot 
            # works better than the rospy.is_shut_down()
            self.stop() # Se invoca la propiedad de detenerce
            self.ctrlC = True # Y se cambiar el parametro de control a verdadero para que el sistema se encuentre listo a realizar movimientos futuros

    def stop(self): # Se define la propidad que detiene el robot 
        rospy.loginfo("shutdown time! Stop the robot") # Se da un mensaje de informacion al sistema
        cmd = Twist() # Se define la variable que contendra el mensaje en el que punlicar
        cmd.linear.x = 0.0 # Se define la velidad lineal
        cmd.angular.z = 0.0 # Y angular
        rospy.loginfo("Stoping") # Se da un mensaje de informacion al sistema
        self.publishCmdVel(cmd) # Se invoca la propiedad del Publisher para dar las ordenes al robot

    def moveXTime(self, movingTime, linearSpeed, angularSpeed): # Se define la propiedad de movimiento
        cmd = Twist() # Se define la variable que contendra el mensaje en el que punlicar
        cmd.linear.x = linearSpeed # Se define la velidad lineal
        cmd.angular.z = angularSpeed # Y angular
        if linearSpeed!=0.0:
	        rospy.loginfo("Moving Forwards") # Se da un mensaje de informacion al sistema
        elif angularSpeed!=0.0:
	        rospy.loginfo("Turning") # Se da un mensaje de informacion al sistema
        #rospy.loginfo("Moving Forwards") # Se da un mensaje de informacion al sistema
        self.publishCmdVel(cmd) # Se invoca la propiedad del Publisher para dar las ordenes al robot
        #time.sleep(movingTime) # Se da un tiempo de espera para el movimiento
        rospy.sleep(movingTime)
        self.stop() # Se invoca la propiedad que detiene el robot
        rospy.loginfo("######## Finished Moving Forwards")
    
    
    
    ###########################################33
    # Se definenen los pasos a realizar por el robot deacuerdo con los requerimientos del sistema, invocando las propiedades anteriormente programadas    
    
    def moveRobot(self):
        while True:
            valor = self.scanobject.getData()
            print valor
        


        # Mover hacia adelante
        #self.moveXTime(movingTime=3, linearSpeed=0.5, angularSpeed=0.0)
        # Girar
        #self.moveXTime(movingTime=3, linearSpeed=0.0, angularSpeed=0.6)

        #self.moveXTime(movingTime=3, linearSpeed=0.5, angularSpeed=0.0)
        # Girar
        #self.moveXTime(movingTime=3, linearSpeed=0.0, angularSpeed=-0.0)

        #self.moveXTime(movingTime=3, linearSpeed=0.5, angularSpeed=0.0)
        # Girar
        #self.moveXTime(movingTime=3, linearSpeed=0.0, angularSpeed=-0.6)

        #self.moveXTime(movingTime=3, linearSpeed=0.5, angularSpeed=0.0)
        # Girar
        #self.moveXTime(movingTime=3, linearSpeed=0.0, angularSpeed=0.0)

         
        rospy.loginfo("######## Finished Moving in a Square")
            
if __name__ == '__main__':
    rospy.init_node('move', anonymous=True) # Se inicia el nodo
    moveObject = Move() # Se invoca el objeto que tontiene todas las operaciones
    try:
        moveObject.moveRobot() # Se invoca la propiedad del provimiento programada dentro de la clase
    except rospy.ROSInterruptException:
        pass
