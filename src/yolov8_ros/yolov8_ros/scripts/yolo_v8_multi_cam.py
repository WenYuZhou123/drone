#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np
from ultralytics import YOLO
from time import time
from cv_bridge import CvBridge,CvBridgeError
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes


class Yolo_Dect:
    def __init__(self):
        # load parameters
        weight_path = rospy.get_param('~weight_path', '')
        image_topic_1 = rospy.get_param('~image_topic_1', '/camera_1/color/image_raw')
        image_topic_2 = rospy.get_param('~image_topic_2', '/camera_2/color/image_raw')
        pub_topic_1 = rospy.get_param('~pub_topic_1', '/yolov8/camera_1/BoundingBoxes')
        pub_topic_2 = rospy.get_param('~pub_topic_2', '/yolov8/camera_2/BoundingBoxes')
        self.camera_frame_1 = rospy.get_param('~camera_frame_1', '')
        self.camera_frame_2 = rospy.get_param('~camera_frame_2', '')
        conf = rospy.get_param('~conf', '0.5')
        self.visualize = rospy.get_param('~visualize', 'True')

        # which device will be used
        if rospy.get_param('/use_cpu', 'false'):
            self.device = 'cpu'
        else:
            self.device = 'cuda'

        self.bridge = CvBridge()
        self.model = YOLO(weight_path)
        self.model.to(self.device)
        self.model.conf = conf

        # Load class color
        self.classes_colors = {}

        # image subscribers for two cameras
        self.color_sub_1 = rospy.Subscriber(image_topic_1, Image, self.image_callback_1, queue_size=1, buff_size=52428800)
        self.color_sub_2 = rospy.Subscriber(image_topic_2, Image, self.image_callback_2, queue_size=1, buff_size=52428800)

        # output publishers for two cameras
        self.position_pub_1 = rospy.Publisher(pub_topic_1, BoundingBoxes, queue_size=1)
        self.position_pub_2 = rospy.Publisher(pub_topic_2, BoundingBoxes, queue_size=1)

        self.image_pub_1 = rospy.Publisher('/yolov8/camera_1/detection_image', Image, queue_size=1)
        self.image_pub_2 = rospy.Publisher('/yolov8/camera_2/detection_image', Image, queue_size=1)

    def image_callback_1(self, image):
        self.process_image(image, self.position_pub_1, self.image_pub_1, self.camera_frame_1)

    def image_callback_2(self, image):
        self.process_image(image, self.position_pub_2, self.image_pub_2, self.camera_frame_2)

    def process_image(self, image, position_pub, image_pub, camera_frame):
        boundingBoxes = BoundingBoxes()
        boundingBoxes.header = image.header
        boundingBoxes.image_header = image.header
        boundingBoxes.len = 0

        color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        
        results = self.model(color_image)
        frame = results[0].plot()
        fps = 1000.0 / results[0].speed['inference']
        cv2.putText(frame, f'FPS: {int(fps)}', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        
        for result in results[0].boxes:
            if result.conf.item() < self.model.conf:
                continue
            boundingBox = BoundingBox()
            boundingBox.xmin = int(result.xyxy[0][0].item())
            boundingBox.ymin = int(result.xyxy[0][1].item())
            boundingBox.xmax = int(result.xyxy[0][2].item())
            boundingBox.ymax = int(result.xyxy[0][3].item())
            boundingBox.Class = results[0].names[result.cls.item()]
            boundingBox.probability = result.conf.item()
            boundingBoxes.bounding_boxes.append(boundingBox)
            boundingBoxes.len += 1

        position_pub.publish(boundingBoxes)
        self.publish_image(frame, image_pub, camera_frame, image.height, image.width)

        if self.visualize:
            cv2.imshow(f'YOLOv8 {camera_frame}', frame)
            cv2.waitKey(3)

    def publish_image(self, imgdata, image_pub, camera_frame, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        image_pub.publish(image_temp)


def main():
    rospy.init_node('yolov8_ros_dual_camera', anonymous=True)
    yolo_dect = Yolo_Dect()
    rospy.spin()


if __name__ == "__main__":
    main()
