#!/usr/bin/env python
import numpy as np
from duckietown_msgs.msg import Twist2DStamped
import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Joy



class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber("/duckiebot/joy",Joy,self.callback)
	        self.publisher= rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = "x")

	
	#def publicar(self):

	def callback(self,msg):
		var1 = msg.axes[1]
		var2=msg.axes[0]
		boton_b=msg.buttons[1]
		if boton_b==1:
		   var1=var1*0
		   var2=var2*0
		print("eje y:",var1*10,var1*(3.14/2),str(mapeo(var1, -1, 1, 0,1)))
		print("eje x:",var2*10,var2*(3.14/2),str(mapeo(var2,-1, 1, 0,1)))
		self.twist= Twist2DStamped()
		self.twist.v=var1*(-5)
		self.twist.omega=var2*(10)
		self.publisher.publish(self.twist)
def mapeo(valor, inf, sup, out_inf, out_sup):
	return np.interp(valor,[inf,sup],[out_inf,out_sup])
def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
