#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes


class Yolo_Dect:
    def __init__(self):
        # 加载参数
        weight_path = rospy.get_param('~weight_path', '')
        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        pub_topic = rospy.get_param('~pub_topic', '/yolov8/BoundingBoxes')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        conf = float(rospy.get_param('~conf', 0.5))
        self.visualize = rospy.get_param('~visualize', True)

        # 初始化模型并设置设备
        self.model = YOLO(weight_path)
        if rospy.get_param('/use_cpu', False):
            self.device = 'cpu'
            self.model.to(self.device)
        else:
            self.device = 'cuda'
            self.model.to(self.device)

        self.bridge = CvBridge()
        self.color_image = None
        self.getImageStatus = False

        # 订阅图像话题
        self.color_sub = rospy.Subscriber(
            image_topic, Image, self.image_callback,
            queue_size=1, buff_size=52428800
        )

        # 发布检测结果以及可视化图像
        self.position_pub = rospy.Publisher(
            pub_topic, BoundingBoxes, queue_size=1
        )
        self.image_pub = rospy.Publisher(
            '/yolov8/detection_image', Image, queue_size=1
        )

        # 如果一直没有收到图像，则等待
        while not self.getImageStatus and not rospy.is_shutdown():
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image_msg):
        # 收到图像后，将其转成OpenCV格式并进行检测
        self.getImageStatus = True
        self.color_image = np.frombuffer(
            image_msg.data, dtype=np.uint8
        ).reshape(image_msg.height, image_msg.width, -1)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        # 推理并可视化
        results = self.model(self.color_image)
        self.dectshow(results, image_msg.height, image_msg.width, image_msg.header)

        cv2.waitKey(3)

    def dectshow(self, results, height, width, header):
        # 创建一个空的BoundingBoxes用于存放所有检测框
        bb_msg = BoundingBoxes()
        bb_msg.header = header
        bb_msg.image_header = header

        # 如果检测到至少一个目标
        if len(results[0].boxes) > 0:
            # 在可视化图像上绘制检测框并组装消息
            self.frame = results[0].plot()
            for box in results[0].boxes:
                b = BoundingBox()
                b.xmin = int(box.xyxy[0][0].item())
                b.ymin = int(box.xyxy[0][1].item())
                b.xmax = int(box.xyxy[0][2].item())
                b.ymax = int(box.xyxy[0][3].item())
                b.Class = results[0].names[box.cls.item()]
                b.probability = box.conf.item()
                bb_msg.bounding_boxes.append(b)
            bb_msg.len = len(results[0].boxes)
        else:
            # 未检测到任何目标，发布一个空的列表，len=0
            self.frame = results[0].plot()  # 依然产生一张可视化图（纯背景）
            bb_msg.len = 0

        # 发布检测结果（包含空结果时也会发布）
        self.position_pub.publish(bb_msg)

        # 始终发布带框图（即使为空，也要保持流不断裂）
        self.publish_image(self.frame, height, width)

        # 可视化显示
        if self.visualize:
            cv2.imshow('YOLOv8', self.frame)

    def publish_image(self, imgdata, height, width):
        # 将OpenCV图片转回ROS Image并发布
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)


def main():
    rospy.init_node('yolov8_ros', anonymous=True)
    yolo_dect = Yolo_Dect()
    rospy.spin()


if __name__ == "__main__":
    main()
