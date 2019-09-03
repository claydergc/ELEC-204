#!/usr/bin/env python
# Curso> ELEC204 Sistemas mecatronicos y robotica

# En lugar de 'testpy', deberia ir el nombre del paquete creado.
import roslib; roslib.load_manifest('ELEC-204')
import time
import rospy
import math
import numpy
import random
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry     
from geometry_msgs.msg import Twist
from std_msgs.msg import String 


# Si solo hay un robot en Stage, robotTag = ""
# Si hay mas de un robot en Stage, robotTag = "/robot_0", "/robot_1", etc.
class Robot:
	def __init__(self, robotID, init_x, init_y, init_angle):

		# posicion y angulo
		self.pose_x=init_x
		self.pose_y=init_y
		self.angle = init_angle

		# posicion y angulo iniciales
		self.init_x=init_x
		self.init_y=init_y
		self.angle = init_angle

		# cambio en la posicion en la ultima iteracion
		self.delta_x=0.0
		self.delta_y=0.0
		
		# distancia recorrida
		self.perim = 0.0

		# distancias medidas por el laser
		self.distances = []

		# arreglo con los angulos a los que corresponde cada muestra del sensor
		self.laser_angles = []

		# variable auxiliar
		self.laser_angles1 = []
		
		# matriz de parametros de ruido odometrico, de distancia (ranger) y angular (ranger)
		# odomNoise_ON/OFF={0.0, 1.0}		odomNoise_mu		odomNoise_sigma
		# rangerNoise_ON/OFF={0.0, 1.0}		rangerNoise_mu		rangerNoise_sigma
		# angularNoise_ON/OFF={0.0, 1.0}	angularNoise_mu		angularNoise_sigma
		self.noise = [[0,0,0],[0,0,0],[0,0,0]]

		# indica si se ha recibido la primera lectura de sensor
		self.distances_OK = False

		# comunicacion con ROS
		self.cmd_pub=rospy.Publisher(robotID+'/r1/cmd_vel', Twist, queue_size=10)
		self.cmd = Twist()
		self.rate = rospy.Rate(20.0)


	#########################################
	############### FUNCIONES ###############
	#########################################

	# El robot se mueve hasta recorrer una distancia igual a perimeter 
	def moveTill(self, perimeter, lin_vel, ang_vel):

		while not rospy.is_shutdown() and self.perim<=perimeter:
			self.cmd.linear.x = lin_vel
			self.cmd.angular.z = ang_vel
			self.cmd_pub.publish(self.cmd)
			self.rate.sleep()

	# El robot se mueve (nL, nR) pulsos de encoder
	def nSteps(self, nL, nR):
		if nL == 0 and nR == 0:
			return None
		P = 150.0
		R = 0.09
		N = 500
		t_sim = (1.0*max(abs(nL),(nR)))/P
		trcs = 0.334
		aux = (1.0*min(abs(nL),abs(nR)))/(1.0*max(abs(nR),abs(nL))) 
		if abs(nL)>=abs(nR):
			VL = math.copysign(1.0,nL) * P * 2.0 * math.pi * R / N
			VR = math.copysign(1.0,nR) * abs(VL) * aux
		else:
			VR = math.copysign(1.0,nR) * P * 2.0 * math.pi * R / N
			VL = math.copysign(1.0,nL) * abs(VR) * aux
	
		v_lin = (VL + VR)/2.0
		v_ang = (VR - VL)/trcs

		tm = rospy.get_rostime()
		while tm.secs == 0 and tm.nsecs == 0:
			tm = rospy.get_rostime()

		while not rospy.is_shutdown():
			cur = rospy.get_rostime()
			self.cmd.linear.x = v_lin
			self.cmd.angular.z = v_ang
			self.cmd_pub.publish(self.cmd)
			self.rate.sleep()
			if not (cur - tm) < rospy.Duration(t_sim):
				break

		self.cmd.linear.x = 0.0
		self.cmd.angular.z = 0.0
		self.cmd_pub.publish(self.cmd)

	# Muestra distancia total recorrida
	def show_distance(self):
		print 'Distancia recorrida [m]: ' + str(self.perim)

	# Resetea distancia recorrida
	def reset_perim(self):
		self.perim = 0.0

	# Transforma angulos leidos en odom_callback en angulos en el rango [0, 360)
	# Uso: self.angle_tf(self.alpha, self.alpha_w); R.angle_tf(R.alpha, R.alpha_w)
	def angle_tf(self, alpha_z, alpha_w):
		if alpha_z>=0:
			return alpha_z
		else:
			return 360 + alpha_z

	# Funcion de callback: se debe ejecutar cada vez que el subscriptor recibe informacion (/odom)
	# Actualiza posicion, camino recorrido, orientacion en funcion de los datos odometricos recibidos
	def odom_callback(self, data):
		self.delta_x = data.pose.pose.position.x - self.pose_x
		self.delta_y = data.pose.pose.position.y - self.pose_y
		self.pose_x = data.pose.pose.position.x + self.noise[0][0]*random.gauss(self.noise[0][1],self.noise[0][2])
		self.pose_y = data.pose.pose.position.y + self.noise[0][0]*random.gauss(self.noise[0][1],self.noise[0][2])
		alpha_z = numpy.arcsin(data.pose.pose.orientation.z)*360/math.pi
		alpha_w = numpy.arccos(data.pose.pose.orientation.w)*360/math.pi
		self.angle = self.angle_tf(alpha_z, alpha_w)
		self.perim = self.perim + (self.delta_x**2 + self.delta_y**2)**0.5

	# Funcion de callback para (/base_scan)
	# Actualiza las distancias medidas
	def ranger_callback(self, data):

		if self.distances_OK == False:
			self.distances = data.ranges
			for i in range(0,len(data.ranges)):
				self.laser_angles1.append(data.angle_min+i*data.angle_increment)
			self.laser_angles = self.laser_angles1
			if len(self.distances)>0:
				self.distances_OK = True

			else:
				return None

		if self.noise[1][0] == 1:
			aux_noise_r = [random.gauss(self.noise[1][1],self.noise[1][2]) for i in xrange(len(data.ranges))]
			self.distances = map(sum, zip(data.ranges, aux_noise_r))
		else:
			self.distances = data.ranges
		if self.noise[2][0] == 1:
			aux_noise_a = [random.gauss(self.noise[2][1],self.noise[2][2]) for i in xrange(len(data.ranges))]
			self.laser_angles = map(sum, zip(self.laser_angles1, aux_noise_a))
		else:			
			self.laser_angles = self.laser_angles1

