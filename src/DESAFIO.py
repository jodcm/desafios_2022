#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge 
class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber("/duckiebot/camera_node/image/rect",Image,self.procesar_img)
		self.bridge = 	CvBridge()
		self.detector = cv2.CascadeClassifier("/home/duckiebot/duckietown/catkin_ws/src/desafios_2022/src/cascade3_LBP.xml")
		self.publisher = rospy.Publisher("/duckiebot/camera_node/image/dets",Image, queue_size=1)
		
	#def publicar(self):

	def procesar_img(self,msg):
		image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
		image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		dets=self.detector.detectMultiScale(image_gray, 1.3,10)
		print(dets)
		#Dibujar detecciones
		for cnt in dets:    
		# Dibujar rectangulos de cada blob
			x,y,w,h=cnt
                	cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,255),2)
                
		msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
		self.publisher.publish(msg)

def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba
	

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
