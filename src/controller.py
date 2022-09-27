#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist, Point# importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy


class Template(object):
	def __init__(self, args):
		
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber("/duckiebot/posicionPato",Point,self.procesar_dis)
		self.sub= rospy.Suscriber("/duckiebot/filtro", Twist2DStamped,self.procesar_twist)
		self.publisher = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = "x")
	#def publicar(self):

	#def callback(self,msg):

	def procesar_dis(self, dis):
		if dis.z<=15:
		#procesar_twist(self, self.twist):
		   self.twist.v=0
		   self.twist.omega=0
		   self.publisher.publish(self.twist)
		   


		
def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
