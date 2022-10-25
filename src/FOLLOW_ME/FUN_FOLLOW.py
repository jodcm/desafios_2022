#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from duckietown_msgs.msg import Twist2DStamped
from geometry_msgs.msg import Twist, Point# importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy


class Template(object):
	def __init__(self, args):
		
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber("/duckiebot/dist_bot",Point,self.procesar_dis)
		self.sub1= rospy.Subscriber("/duckiebot/filtro", Twist2DStamped,self.procesar_twist)
		self.publisher = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = 1)
		self.dz=300
	#def publicar(self):

	#def callback(self,msg):

	def procesar_dis(self, dis):
		self.dz=dis.z
		return self.dz
	def procesar_twist(self, twist):
		print self.dz
		if self.dz<=10:
			twist.v= 0
			twist.omega=0
		self.publisher.publish(twist)

#pato 		   

		
def main():
	rospy.init_node('FUN_FOLLOW') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
