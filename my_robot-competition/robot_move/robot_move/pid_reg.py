import cv2
import matplotlib.pyplot as plt
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from math import pi

class Controller(Node):

	def __init__(self):
		super().__init__('pid')
		self.declare_parameters( namespace='',
            				parameters=[
					('Kp', 0.750),
					('Ki', 0.042),
					('Kd', 0.002),
					('desiredV', 0.2),])
					
		self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
		self.subscription = self.create_subscription(Float64, '/detect/line', self.start_PID, 20)
		self.subscript_traffic = self.create_subscription(Image, '/color/image', self.traffic_light, 5)

		self.yolo_traffic = self.create_subscription(Bool, '/comand', self.block, 20)

		self.br = CvBridge()
		self.twist = Twist()
		self.subscription # Prevent unused variable warn
		self.len_stack = 10
		self.E = [0]*self.len_stack     # Cummulative error
		self.old_e = 0  # Previous error
		self.start = False
		self.light = False

	def block(self, msg):
		if self.light == True:
			self.start = msg.data

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
			#cv2.imshow('camera', cv_image)
			#cv2.waitKey(1)
			if green_pred[0] in list(range(0, 21)) and green_pred[1] in list(range(90, 120)) and green_pred[2] in list(range(0, 21)):
				self.start = True
				self.light = True
				self.get_logger().info(f'Старт и цвет: {str(green_pred)}')
		
	def start_PID(self, msg):
		if self.start:
			self.iteratePID(msg)
		else:
			self.old_e = 0
			self.E = [0]*self.len_stack 
			
	def iteratePID(self, msg):
		'''This PID controller only calculates the angular. Velocity with constant speed of v. 
		   The value of v can be specified by giving in parameter or using the pre-defined value defined above.'''
		   
		#Get params PID 
		self.Kp = self.get_parameter("Kp").get_parameter_value().double_value
		self.Ki = self.get_parameter("Ki").get_parameter_value().double_value
		self.Kd = self.get_parameter("Kd").get_parameter_value().double_value
		self.desiredV = self.get_parameter("desiredV").get_parameter_value().double_value
		
		# Get target value
		target = 405
		
		# Get actual error
		err = (target - msg.data)/100

		# Compute PID's values
		e_P = err
		e_I = sum(self.E) + err
		e_D = err - self.old_e
			
		# Compute new angle value depending on the velocity value
		w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D
		w_max = 6.0
		k_w_max = 0.85
		if w > w_max: w = w_max
		if w < -w_max: w = -w_max

		# Update nonlinear system
		self.E.pop(0)
		self.E.append(err)
		self.old_e = err
			
		# Update
		self.twist.linear.x = self.desiredV *  (1- k_w_max*w/w_max)
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

