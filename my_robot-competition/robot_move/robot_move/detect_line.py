import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Bool, UInt8
from sensor_msgs.msg import Image
import os

class DetectLane(Node):

	def __init__(self):
		super().__init__('line')
		self.publisher = self.create_publisher(Float64, '/detect/line', 5)
		self.subscription = self.create_subscription(Image, '/color/image_projected_compensated', self.cbFindLane, 5)
		self.br = CvBridge()
		self.subscription # prevent unused variable warn
		self.last_cx_white = 0
		self.last_cx_yellow = 600
		

	def cbFindLane(self, image_msg):
		cv_image = self.br.imgmsg_to_cv2(image_msg, image_msg.encoding)

		# Find White and Yellow Lanes
		lower_yellow = np.array([20, 100, 100])
		upper_yellow = np.array([30, 255, 255])
		lower_white = np.array([0, 0, 250])
		upper_white = np.array([0, 0, 255]) 

		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

		# Создание масок для белого и жёлтого цветов
		mask_white = cv2.inRange(hsv, lower_white, upper_white)
		mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

		mask_combined = cv2.bitwise_or(mask_white, mask_yellow)

    	# Наложение объединенной маски на исходное изображение
		masked_image = cv2.bitwise_or(cv_image, cv_image, mask=mask_combined)

		 # Вычисление моментов для белых и жёлтых масок
		moments_white = cv2.moments(mask_white)
		moments_yellow = cv2.moments(mask_yellow)

		# Вычисление центроидов с использованием формулы моментов
		if moments_white['m00'] != 0 and moments_yellow['m00'] != 0:
			cx_white = int(moments_white['m10'] / moments_white['m00'])
			cx_yellow = int(moments_yellow['m10'] / moments_yellow['m00'])	
			self.last_cx_yellow = cx_yellow
			self.last_cx_white = cx_white
		
		if moments_white['m00'] == 0 and moments_yellow['m00'] != 0:	
			cx_white  = self.last_cx_white + 5
			cx_yellow = int(moments_yellow['m10'] / moments_yellow['m00'])

		if moments_yellow['m00'] == 0 and moments_white['m00'] != 0:
			cx_white  = int(moments_white['m10'] / moments_white['m00'])
			cx_yellow = self.last_cx_yellow - 5

		if moments_yellow['m00'] == 0 and moments_white['m00'] == 0:
			cx_white  = self.last_cx_white
			cx_yellow  = self.last_cx_white

		cv2.imshow('camera', masked_image)
		cv2.waitKey(1)
		
		
		msg_desired_center = Float64()
		if cx_yellow < cx_white:
			msg_desired_center.data = (cx_yellow + cx_white)/2
		self.publisher.publish(msg_desired_center)
		

def main(args=None):
    rclpy.init(args=args)
    robot_app = DetectLane()
    rclpy.spin(robot_app)
	
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

