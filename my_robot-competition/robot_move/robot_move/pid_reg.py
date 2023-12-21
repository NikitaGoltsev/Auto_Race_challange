import cv2
import matplotlib.pyplot as plt
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from math import pi

class Controller(Node):

	def __init__(self):
		super().__init__('pid')
		self.declare_parameters( namespace='',
            				parameters=[
					('Kp', 0.002),
					('Ki', 0.0001),
					('Kd', 0.000),
					('desiredV', 0.25),])
					
		self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
		self.subscription = self.create_subscription(Float64, '/detect/line', self.move_Controller, 5)
		self.subscript_traffic = self.create_subscription(Image, '/color/image', self.traffic_light, 1)

		self.yolo_traffic = self.create_subscription(Int32, '/comand', self.sign_detect, 10)

		self.br = CvBridge()
		self.twist = Twist()
		self.subscription # Prevent unused variable warn

		self.len_stack = 15 # Integral error length
		self.E = [0]*self.len_stack  # Cummulative error

		self.old_e = 0  # Previous error

		self.PidUp = True
		self.light = False
		self.target = None
		self.right = None
		self.count_max = 4
		self.counter = self.count_max
		

	def sign_detect(self, msg):
		if msg.data == 2:
			self.right = True
		elif msg.data == 1:
			self.right = False
		else:
			self.right = None

	def traffic_light(self, image):
		if self.light == False:
			cv_image = self.br.imgmsg_to_cv2(image, "bgr8") # blue green red 255
			weight = cv_image.shape[1]
			high = cv_image.shape[0]
			k_w = 11/15
			k_h = 5/8.5
			delta = 5
			green_pred = cv_image[int(high*k_h)-delta:int(high*k_h)+delta, int(weight*k_w)-delta:int(weight*k_w)+delta]
			
			green_pred = np.mean(green_pred, axis = (0,1))

			if green_pred[0] in list(range(0, 21)) and green_pred[1] in list(range(90, 120)) and green_pred[2] in list(range(0, 21)):
				self.light = True
				self.get_logger().info(f'Гонка началась! Цвет светофора: {str(green_pred)}')
		
	def move_Controller(self, msg):
		if self.light==True:
			if (self.right!= None and self.counter==self.count_max) or (self.counter < self.count_max and self.counter > 0):	
				w = ((-1) if self.right else (1))*pi/6
				x = 0.085
				self.twist.linear.x = x
				self.twist.angular.z = float(w)
				self.publisher_.publish(self.twist)
				self.counter -= 1
				side = 'направо' if self.right else 'налево'
				self.get_logger().info(f'Поворачиваю: {side} {self.counter}')
				self.old_e = 0
				self.E = [0]*self.len_stack 

			if self.counter == 0:
				self.counter = self.count_max

			if self.PidUp==True and self.counter==self.count_max:
				self.iteratePID(msg)
			
	def iteratePID(self, msg):
		'''This PID controller only calculates the angular. Velocity with constant speed of v. 
		   The value of v can be specified by giving in parameter or using the pre-defined value defined above.'''
		   
		#Get params PID 
		self.Kp = self.get_parameter("Kp").get_parameter_value().double_value
		self.Ki = self.get_parameter("Ki").get_parameter_value().double_value
		self.Kd = self.get_parameter("Kd").get_parameter_value().double_value
		self.desiredV = self.get_parameter("desiredV").get_parameter_value().double_value
		
		# Get target value
		if self.target == None:
			self.target = msg.data + 40 

		# Get actual error
		err = self.target - msg.data

		# Compute PID's values
		e_P = err
		e_I = sum(self.E) + err
		e_D = err - self.old_e
			
		# Compute new angle value depending on the velocity value
		w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D
		w_max = 2.0

		k_w_max = 0.8
		if w > w_max: w = w_max
		if w < -w_max: w = -w_max
		#self.get_logger().info(f'Message data: {w}')
		# Update nonlinear system
		self.E.pop(0)
		self.E.append(err)
		self.old_e = err
			
		# Update
		self.twist.linear.x = self.desiredV *  (1 - k_w_max*abs(w)/w_max)
		self.twist.angular.z = float(w)
			#self.get_logger().info('Message data: %lf' % self.twist.linear.x)
			#self.get_logger().info('Message data: %lf' % self.twist.angular.z)
			#self.get_logger().info('Я еду')
		self.publisher_.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    robot_app = Controller()
    rclpy.spin(robot_app)
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

