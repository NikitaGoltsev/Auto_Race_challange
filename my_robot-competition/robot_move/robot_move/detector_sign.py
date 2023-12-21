# ROS2 file for detecting objects from the image. Used Yolov5 model
import cv2
import rclpy
import torch
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber

class Controller(Node):
	'''A class that implements live image object detection and 
		publishes classes of detected objects in a topic'''
	def __init__(self):
		super().__init__('detect')
		# Message init
		self.br = CvBridge()
		self.twist = Twist()
		self.sign = Int32()
		
		# Creating publsihers to the processing
		self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
		self.yolo_publisher = self.create_publisher(Int32, '/comand', 10)

       	# Creating subscribers to the camera top
		color_sub = Subscriber(self, Image, '/color/image')
		depth_sub = Subscriber(self, Image, '/depth/image')

		self.work_direct = self.declare_parameter('model_path', '').get_parameter_value().string_value
		weight_direct = self.work_direct + '/best.pt' 

		# Model Init Yolo
		self.model = torch.hub.load(self.work_direct+'/yolov5', 'custom', path=weight_direct, source='local')
		self.model.load_state_dict(torch.load(weight_direct), strict=False)

  		# Creating synchronizer with approximate time
		ats = ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=5, slop=0.1)
		ats.registerCallback(self.callback)

		self.color_image, self.depth_image = None, None

	def yolo_detect(self):
    	# Reading the image and sending it to yolo
		image_rgb = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB) 
		predict = self.model(image_rgb)

    	# Receiving detection results
		detections = predict.xyxy[0].cpu().numpy()
		results = {}

    	# Processing of results
		for *xyxy, conf, cls in detections:
			if xyxy:
				results[cls] = (xyxy, conf)
		return results
	
	def callback(self, color_msg, depth_msg):

		self.color_image = self.br.imgmsg_to_cv2(color_msg, color_msg.encoding)
		self.depth_image = self.br.imgmsg_to_cv2(depth_msg, depth_msg.encoding)
		self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
		if (self.color_image is not None) and (self.depth_image is not None):

			classes_bbox = self.yolo_detect()
			decoder_class = {'red':0 , 'green':1, 'yellow':2, 'stage_1':3,
							'left':4, 'right':5, 'stage_3':6, 'stage_2':7, 
							'car':8, 'man':9, 'stage_4':10, 'stage_5':11}
			
			coder_class = {0:'red' , 1:'green', 2:'yellow', 3:'stage_1',
							4:'left', 5:'right', 6:'stage_3', 7:'stage_2', 
							8:'car', 9:'man', 10:'stage_4', 11:'stage_5'}
			
			# Dictionaries for storing data distants and objects confidence
			class_distances = {}
			class_conf = {}

			# Size of image
			img_height, img_width = self.depth_image.shape[:2]
			
			# проходим по каждому классу
			for class_name, (bbox, conf) in classes_bbox.items():

				x_min, y_min, x_max, y_max = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])

				# извлекаем соответствующий bbox из изображения глубины
				depth_bbox = self.depth_image[y_min:y_max, x_min:x_max]
				
				if class_name in coder_class:
					name = coder_class[class_name]
				else:
					name = "unknown"
				
				if conf > 0.65:
					cv2.rectangle(self.color_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
					cv2.putText(self.color_image, f'{name}: {conf}', (x_min, y_min-10), cv2.FONT_HERSHEY_SIMPLEX,  1.0, (0, 255, 0), 2)
				
				# проверяем, есть ли пиксели в depth_bbox
				if depth_bbox.size > 0:
					# заменяем значения Nan и infinity на среднее значение остальных пикселей
					depth_bbox = np.where(np.isfinite(depth_bbox), depth_bbox, np.nan)
					mean_val = np.nanmean(depth_bbox)
					depth_bbox = np.where(np.isnan(depth_bbox), mean_val, depth_bbox)

					# вычисляем среднее значение глубины для данного bbox
					mean_depth = np.mean(depth_bbox)

					# добавляем результат в словарь
					class_distances[class_name] = mean_depth
					class_conf[class_name] = conf
				else:
					self.get_logger().info(f'bbox для {class_name} не содержит пикселей.')
					
			# Image show
			cv2.imshow('YOLO Detections', self.color_image)
			cv2.waitKey(1)
			#self.get_logger().info('Message data: %s' % 'Found classes:')
			#self.get_logger().info('Message data: %s' % str(class_distances))

			if (decoder_class['right'] in class_distances) and (class_conf[decoder_class['right']] > 0.65) and\
													(class_distances[decoder_class['right']] <= 0.65) and (class_distances[decoder_class['right']] >= 0.45):
				d = class_distances[decoder_class['right']]
				conf = class_conf[decoder_class['right']]
				#self.get_logger().info(f'Right, {d}, {conf}')
				self.sign.data = 2

			elif (decoder_class['left'] in class_distances) and (class_conf[decoder_class['left']] > 0.65) and\
				 									 (class_distances[decoder_class['left']] <= 0.75)  and (class_distances[decoder_class['left']] >= 0.525):
				d = class_distances[decoder_class['left']]
				conf = class_conf[decoder_class['left']]
				#self.get_logger().info(f'Left, {d}, {conf}')
				self.sign.data = 1

			else:
				self.sign.data = 0
			self.yolo_publisher.publish(self.sign)
	
		# Images reset
		self.color_image = None
		self.depth_image = None

def main(args=None):
    rclpy.init(args=args)
    robot_app = Controller()
    rclpy.spin(robot_app)
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()