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
		self.sub = rospy.Subscriber("/duckiebot/camera_node/image/raw",Image,self.procesar_img)
		self.publisher = rospy.Publisher("/duckiebot/detecciones",Image, queue_size=10)
		self.sub1 = rospy.Subscriber("/duckiebot/camera_node/image/rect",Image,self.procesar_img)
		self.publisher1 = rospy.Publisher("/duckiebot/detecciones",Image, queue_size=10)
    		self.publisher2 = rospy.Publisher("/duckiebot/posicionPato",Point, queue_size=10)
	#def publicar(self):

	#def callback(self,msg):

	def procesar_img(self, imsg):
		# Cambiar espacio de color
		bridge=CvBridge()
		image_in= bridge.imgmsg_to_cv2(msg, "bgr8")				
		image_out= cv2.cvtColor(image_in,cv2.COLOR_BGR2HSV)
		lower_limit=np.array([20,100,200])
		upper_limit=np.array([60,255,255])
		mask=cv2.inRange(image_out,lower_limit,upper_limit)
		kernel= np.ones((5,5), np.uint8)
		mask = cv2.erode(mask,kernel, iterations = 1)
		mask = cv2.dilate(mask,kernel, iterations = 2)
		image_od=cv2.bitwise_and(image_in,image_in,mask=mask)
		# Filtrar rango util
		

		# Aplicar mascara

		# Aplicar transformaciones morfologicas

		# Definir blobs
		
                _,contours, hierarchy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:    
		# Dibujar rectangulos de cada blob
                  x,y,w,h=cv2.boundingRect(cnt)
                     msg1 = bridge.cv2_to_imgmsg(image_od, "bgr8")
	             self.publisher1.publish(msg1)
	             z=(101.85916357881302*3)/h
	             print(z)
	             dis=Point[]
	             dis.x=(x+w)/2
	             dis.y=(y+h)/2
	             dis.z=z
	             self.publisher2.publish(dis)
                
                  if cv2.contourArea(cnt) < 200:
                	continue 
                	
                  cv2.rectangle(image_od,(x,y),(x+w,y+h),(0,0,255),2)
                
		# Publicar imagen final
		msg = bridge.cv2_to_imgmsg(image_od, "bgr8")
	        self.publisher.publish(msg)

		msg1 = bridge.cv2_to_imgmsg(image_od, "bgr8")
	        self.publisher1.publish(msg1)
	        z=(101.85916357881302*3)/h
	        print(z)
	        msg2 = Point((x+w)/2),(y+h)/2,z)
	        self.publisher2.publish(msg2)
		
def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
