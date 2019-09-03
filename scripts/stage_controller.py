#!/usr/bin/env python
# Curso> ELEC204 Sistemas Mecatronicos y Robotica

# En lugar de 'testpy', deberia ir el nombre del paquete creado.
import roslib; roslib.load_manifest('ELEC-204')
import robot_utilities
import time
import rospy
import math
import numpy
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry     
from geometry_msgs.msg import Twist
from std_msgs.msg import String 

#def printme( str ):
#	print str
#	return


class Controller:

	#def printme( self, str ):
	#	print str
	#	return

	#def printme( self, nL, nR ):
	#	print str
	#	return x,y
	
	def __init__(self):

		# inicializa nodo ROS
		rospy.init_node('stage_controller');

		# robotID = ID del robot que se desea controlar
		# si hay solo un robot, robotID debe ser ""
		# si hay mas de un robot, robotID debe ser de la forma "/robot_0", "/robot_1", etc.
		robotID = ""

		# Posicion y orientacion iniciales.
		# IMPORTANTE: deben ser consistentes con la del archivo .world
		init_x = 0.0
		init_y = 0.0
		init_angle = 90.0

		# creacion de un objeto de tipo Robot
		R = robot_utilities.Robot(robotID, init_x, init_y, init_angle)

		# Subscripcion a los topicos de interes: odometria (Odometry) y sensor de distancia (LaserScan)
		rospy.Subscriber(robotID+'/r1/odom', Odometry, R.odom_callback)
		#rospy.Subscriber(robotID+'/base_scan', LaserScan, R.ranger_callback)
		# Observacion: Las funciones de callback se ejecutan cada vez que el programa recibe informacion 
		# desde un topico. Se utilizan, generalmente, para actualizar datos.
		# Pueden utilizarse funciones de callback programadas por el alumno, 
		# diferentes a las provistas por la clase Robot

		# Se recomienda que este ciclo vaya antes de todas las instrucciones de control
		# Sirve para asegurar que el Robot recibio su primera lectura de sensor
		#while not rospy.is_shutdown() and len(R.distances)==0:
			#R.rate.sleep()

		# Ciclo de ejemplo. 
		# Si se va a controlar el robot con una funcion de robot_utilities en vez de con el ciclo:
		# opcion 1: cambiar True por False en la condicion del while
		# opcion 2: comentar el bloque de codigo
		while not rospy.is_shutdown() and False:
			# Define velocidad lineal [metros/segundo] del robot durante la iteracion
			R.cmd.linear.x = 0.2
			
			# Velocidad angular [radianes/segundo]; rango aceptable: [-pi/2, pi/2 ], aprox. [-1.57, 1.57]
			R.cmd.angular.z = 0.0 

			# indica a Stage la velocidad que el robot tendra durante un ciclo
			R.cmd_pub.publish(R.cmd)
			
			# Ejemplo de como rescatar datos captados por los sensores
			# En este caso solo se imprimen, pero la idea es utilizarlos en decisiones de control
			#print "_______________________"
			#print 'Primera lectura del laser [m]: '+ str(R.distances[0])
			#print 'Posicion robot [m]: x = ' + str(R.pose_x)+'; y = ' +str(R.pose_y)
			#print 'Orientacion robot [grados]: ' + str(R.angle)
			#R.show_distance()

			# Mantiene la regularidad temporal (de acuerdo al rate definido en robot_utilities.Robot)
			R.rate.sleep()

		# Funciones de ejemplo (uncomment para usar): 
		R.nSteps(500, 500)
		R.show_distance()
        
if __name__ == "__main__": Controller()
