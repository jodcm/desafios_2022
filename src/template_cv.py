#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy


class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber("/duckiebot/camera_node/image/raw",Image,self.procesar_img)
		self.publisher = rospy.Publisher("/duckiebot/camera_node/image/trans",Image, queue_size=10)


	#def publicar(self):

	#def callback(self,msg):

	def procesar_img(self, msg):
		# Cambiar espacio de color
		bridge=CvBridge()
		image_in= bridge.imgmsg_to_cv2(msg, "bgr8")				
		image_out= cv2.cvtColor(image_in,cv2.COLOR_BGR2HSV)
		lower_limit=np.array([20,40,200])
		upper_limit=np.array([40,255,255])
		mask=cv2.inRange(image_out,lower_limit,upper_limit)
		# Filtrar rango util
		

		# Aplicar mascara

		# Aplicar transformaciones morfologicas

		# Definir blobs

		# Dibujar rectangulos de cada blob

		# Publicar imagen final
		msg = bridge.cv2_to_imgmsg(image_out, "bgr8")
	        self.publisher.publish(msg)

def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
