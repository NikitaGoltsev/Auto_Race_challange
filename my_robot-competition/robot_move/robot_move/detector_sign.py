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
import glob
import subprocess
import shutil
from math import pi
from message_filters import ApproximateTimeSynchronizer, Subscriber
import torch

class Controller(Node):
	def __init__(self):
		super().__init__('detect')
		self.br = CvBridge()
		self.twist = Twist()
		self.bool = Bool()
		self.color_image = None
		self.depth_image = None
		self.publisher = self.create_publisher(Twist, '/cmd_vel', 15)

		self.yolo_publisher = self.create_publisher(Bool, '/comand', 20)

        # Создаем подписчиков
		color_sub = Subscriber(self, Image, '/color/image')
		depth_sub = Subscriber(self, Image, '/depth/image')

		self.work_direct = '/home/sear/ws/src/my_robot-competition/robot_move/robot_move/'
		direct = self.work_direct+'yolov5/runs/train/exp/weights/best.pt' 

		self.model = torch.hub.load('ultralytics/yolov5', 'custom', direct)

		self.model.load_state_dict(torch.load(direct), strict=False)

  		# Создаем синхронизатор с приближенным временем
		ats = ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=5, slop=0.1)
		ats.registerCallback(self.callback)


	def yolo_detect(self):
    	# Передайте изображение в модель YOLOv5
		image_rgb = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB) 
		predict = self.model(image_rgb)

    	# Получите результаты детекции
		detections = predict.xyxy[0].cpu().numpy()
		results = {}
    	# Обработайте результаты детекции
		for *xyxy, conf, cls in detections:
			if xyxy:
				results[cls] = (xyxy, conf)
		return results
	
	def callback(self,color_msg, depth_msg):
		self.color_image = self.br.imgmsg_to_cv2(color_msg, "bgr8")
		self.depth_image = self.br.imgmsg_to_cv2(depth_msg, "32FC1")
		if (self.color_image is not None) and (self.depth_image is not None):

			classes_bbox = self.yolo_detect()
			decoder_class = {'red':0 , 'green':1, 'yellow':2, 'stage_1':3,
							'left':4, 'right':5, 'stage_3':6, 'stage_2':7, 
							'car':8, 'man':9, 'stage_4':10, 'stage_5':11}
			coder_class = {0:'red' , 1:'green', 2:'yellow', 3:'stage_1',
							4:'left', 5:'right', 6:'stage_3', 7:'stage_2', 
							8:'car', 9:'man', 10:'stage_4', 11:'stage_5'}
			# создаем новый словарь для хранения результатов
			class_distances = {}

			# размеры изображения
			img_height, img_width = self.depth_image.shape[:2]
			class_conf = {}
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
					
			# Покажите изображение
			cv2.imshow('YOLO Detections', self.color_image)
			cv2.waitKey(1)
			#self.get_logger().info('Message data: %s' % 'Найдены классы:')
			#self.get_logger().info('Message data: %s' % str(class_distances))

			angle = pi/7

			if (decoder_class['right'] in class_distances) and (class_conf[decoder_class['right']] > 0.65) and (class_distances[decoder_class['right']] <= 0.72):
				d = class_distances[decoder_class['right']]
				conf = class_conf[decoder_class['right']]
				self.get_logger().info(f'Right, {d}, {conf}')
				self.twist.angular.z = -1*float(angle)
				self.twist.linear.x = float(0.04)
				self.bool.data = False
				self.yolo_publisher.publish(self.bool)
				self.publisher.publish(self.twist)

			elif (decoder_class['left'] in class_distances) and (class_conf[decoder_class['left']] > 0.65) and (class_distances[decoder_class['left']] <= 0.38):
				d = class_distances[decoder_class['left']]
				conf = class_conf[decoder_class['left']]
				self.get_logger().info(f'Left, {d}, {conf}')
				self.twist.angular.z = float(angle)
				self.twist.linear.x = float(0.05)
				self.bool.data = False
				self.yolo_publisher.publish(self.bool)
				self.publisher.publish(self.twist)
			else:
				self.bool.data = True
				self.yolo_publisher.publish(self.bool)
	
		# после обработки изображений, сбросьте их, чтобы быть готовыми к следующему набору изображений
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
